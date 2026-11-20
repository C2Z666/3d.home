"""Bounded Stair heading feedback in one immutable FAST-LIO odometry frame.

No map relocalization, lateral steering, integral term, reverse or in-place turn.
The caller owns fresh SDK/odometry checks, mode permissions and physical stopping.
"""
import collections
import math
import time


DEFAULTS = dict(heading_deadband_rad=.035, heading_slow_rad=.20,
                lateral_slow_m=.12, slow_speed=.40, min_climb_speed=.40, lateral_severe_m=.30, yaw_kp=.6, yaw_kd=.15,
                filter_tau_sec=.15, max_yaw_rate=.05, max_yaw_accel=.10,
                source_stall_sec=.25, max_sample_gap_sec=.3,
                jump_translation_m=.10, jump_speed_mps=1.5,
                jump_yaw_rad=.20, jump_yaw_rate=1.,
                no_progress_sec=8., progress_step_m=.04,
                divergence_sec=1., divergence_growth_rad=.06)


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def monitor_config(connection):
    config={**DEFAULTS, **connection.get('monitor', {})}
    if set(config)!=set(DEFAULTS):raise ValueError('unknown stair monitor parameter')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in config.values()):
        raise ValueError('stair monitor parameters must be positive finite numbers')
    ctl=connection['control']
    if not config['heading_deadband_rad']<config['heading_slow_rad']<ctl['max_heading_error_rad']:
        raise ValueError('stair heading thresholds must be ordered')
    if config['lateral_slow_m']>=ctl['max_lateral_m'] or config['slow_speed']>ctl['cruise_speed']:
        raise ValueError('invalid stair slowdown thresholds')
    if not .4<=config['min_climb_speed']<=ctl['cruise_speed'] or not config['lateral_slow_m']<config['lateral_severe_m']<ctl['max_lateral_m']:
        raise ValueError('climbing minimum or lateral severity thresholds invalid')
    if config['max_yaw_rate']>.05 or config['max_yaw_accel']>.10:
        raise ValueError('stair correction exceeds validated initial software envelope')
    return config


class StairMonitorFault(RuntimeError):
    def __init__(self,reason,evidence):
        super().__init__(reason)
        self.evidence={**evidence, 'state':'FAULT', 'fault':reason}


class StairMonitor:
    def __init__(self,connection,origin,reference_yaw,now=None):
        self.ctl=connection['control'];self.cfg=monitor_config(connection)
        self.origin=dict(origin);self.reference=wrap(reference_yaw)
        self.previous=dict(origin);self.filtered=wrap(origin['yaw']-self.reference)
        self.rate=0.;self.command=0.;self.last_unique=time.monotonic() if now is None else now
        self.progress_stamp=origin['stamp'];self.best_forward=0.;self.best_rise=0.;self.rows=collections.deque()
        self.evidence={};self.last_output=None

    def fail(self,reason):
        self.command=0.
        raise StairMonitorFault(reason,self.evidence)

    def update(self,sample,base_speed,now=None):
        now=time.monotonic() if now is None else now
        keys=('x','y','z','yaw','roll','pitch','stamp')
        if any(not isinstance(sample.get(k),(int,float)) or not math.isfinite(sample[k]) for k in keys):
            self.fail('楼梯里程计含无效值')
        dt=sample['stamp']-self.previous['stamp']
        if dt<0:self.fail('楼梯里程计时间倒退，禁止沿旧参考继续运动')
        if dt==0:
            if now-self.last_unique>self.cfg['source_stall_sec']:self.fail('楼梯里程计源时间戳停止更新')
            return self.last_output or (0.,0.,dict(state='WAIT_NEW_SAMPLE'))
        if dt>self.cfg['max_sample_gap_sec']:self.fail('楼梯里程计连续性中断')
        delta=math.dist([sample[k] for k in ('x','y','z')],[self.previous[k] for k in ('x','y','z')])
        dyaw=wrap(sample['yaw']-self.previous['yaw'])
        if delta>self.cfg['jump_translation_m']+self.cfg['jump_speed_mps']*dt or abs(dyaw)>self.cfg['jump_yaw_rad']+self.cfg['jump_yaw_rate']*dt:
            self.fail('楼梯里程计突跳，固定方向参考不再可信')
        dx,dy=sample['x']-self.origin['x'],sample['y']-self.origin['y']
        forward=dx*math.cos(self.reference)+dy*math.sin(self.reference)
        lateral=-dx*math.sin(self.reference)+dy*math.cos(self.reference)
        error=wrap(sample['yaw']-self.reference)
        alpha=1.-math.exp(-dt/self.cfg['filter_tau_sec'])
        self.filtered=wrap(self.filtered+alpha*wrap(error-self.filtered))
        self.rate+=alpha*(dyaw/dt-self.rate)
        self.evidence=dict(reference_yaw=self.reference,yaw=sample['yaw'],heading_error_rad=error,
            filtered_heading_error_rad=self.filtered,yaw_rate=self.rate,forward_m=forward,
            lateral_m=lateral,rise_m=sample['z']-self.origin['z'],roll=sample['roll'],pitch=sample['pitch'],
            source_stamp=sample['stamp'],sample_dt=dt,correction_enabled=self.ctl['heading_correction'])
        if abs(error)>self.ctl['max_heading_error_rad']:self.fail('楼梯航向超过硬限制')
        if abs(lateral)>self.ctl['max_lateral_m']:self.fail('楼梯横向偏移超过硬限制')
        if abs(sample['roll'])>self.ctl['max_roll_rad'] or abs(sample['pitch'])>self.ctl['max_pitch_rad']:
            self.fail('楼梯机身姿态超过硬限制')
        if forward<-.2 or sample['z']-self.origin['z']<-.2:self.fail('上楼出现反向或向下位移')
        rise=sample['z']-self.origin['z']
        if forward>self.best_forward+self.cfg['progress_step_m'] or rise>self.best_rise+self.cfg['progress_step_m']:
            self.best_forward=max(self.best_forward,forward);self.best_rise=max(self.best_rise,rise)
            self.progress_stamp=sample['stamp']
        if base_speed>0 and sample['stamp']-self.progress_stamp>self.cfg['no_progress_sec']:
            self.fail('持续前进指令下未取得有效进展')
        lateral_warning=abs(lateral)>self.cfg['lateral_slow_m']
        # Small lateral drift is an alert, not a reason to remove climbing speed.
        slow=abs(self.filtered)>self.cfg['heading_slow_rad'] or abs(lateral)>self.cfg['lateral_severe_m']
        speed=min(base_speed,self.cfg['slow_speed']) if slow else base_speed
        if base_speed>=self.cfg['min_climb_speed']:speed=max(self.cfg['min_climb_speed'],speed)
        dead=self.cfg['heading_deadband_rad']
        effective=math.copysign(max(0.,abs(self.filtered)-dead),self.filtered)
        desired=-self.cfg['yaw_kp']*effective-self.cfg['yaw_kd']*self.rate if effective else 0.
        # Do not command through zero and create alternating steering near target.
        if desired*self.filtered>0:desired=0.
        cap=self.cfg['max_yaw_rate'];desired=max(-cap,min(cap,desired))
        if not self.ctl['heading_correction'] or speed<=0:desired=0.
        step=self.cfg['max_yaw_accel']*min(dt,.1)
        self.command+=max(-step,min(step,desired-self.command))
        if speed<=0:self.command=0.
        if self.ctl['heading_correction'] and abs(self.filtered)>max(.07,2*self.cfg['heading_deadband_rad']):
            self.rows.append((sample['stamp'],self.filtered,self.command))
            while len(self.rows)>1 and sample['stamp']-self.rows[1][0]>=self.cfg['divergence_sec']:self.rows.popleft()
            if self.rows and sample['stamp']-self.rows[0][0]>=self.cfg['divergence_sec']:
                old=self.rows[0][1]
                if self.filtered*old>0 and abs(self.filtered)-abs(old)>self.cfg['divergence_growth_rad'] and all(e*w<0 and abs(w)>.015 for _,e,w in self.rows):
                    self.fail('纠偏后航向持续恶化，可能侧滑或转向响应异常')
        else:self.rows.clear()
        self.evidence.update(state='SLOW_CORRECT' if slow else 'LATERAL_WARNING' if lateral_warning else 'CORRECT' if abs(self.command)>.001 else 'TRACK',
                             speed=speed,yaw_command=self.command,lateral_warning=lateral_warning)
        self.previous=dict(sample);self.last_unique=now
        self.last_output=(speed,self.command,dict(self.evidence))
        return self.last_output
