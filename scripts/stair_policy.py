"""Time limits stop a segment. They do not prove arrival at the landing."""
import math
class StairPolicy:
    def __init__(self,speed,duration,now):
        if not math.isfinite(speed) or not 0 < speed <= .8:
            raise ValueError('speed must be in (0, 0.8] m/s')
        if not math.isfinite(duration) or not 0 < duration <= 180:
            raise ValueError('duration must be in (0, 180] seconds')
        self.speed,self.duration=speed,duration
        self.state,self.reason='WAIT_READY',''
        self.deadline=now+20
        self.session=self.epoch=self.sequence=self.request_id=self.started=None
        self.stop_sequences=set()
    def fail(self,reason):
        self.state,self.reason='FAULT',reason
        return 'stop'
    def update(self,s,now,age):
        if self.state in ('FAULT','DONE'): return 'stop'
        if now > self.deadline and self.state in ('WAIT_READY','WAIT_MODE'):
            return self.fail('SDK readiness or Stair confirmation timed out')
        if age > .5 or not isinstance(s,dict):
            return self.fail('bridge heartbeat lost') if self.state!='WAIT_READY' else 'stop'
        if self.session is not None and (s.get('bridge_session')!=self.session or s.get('safety_epoch')!=self.epoch):
            return self.fail('bridge restarted or safety event occurred')
        safe=all(s.get(k) is True for k in ('sdk_connected','control_held','state_fresh'))
        safe=safe and all(s.get(k) is False for k in ('soft_stop','hardware_stop','nav_handoff_hold','flat_test_lock'))
        state_age=s.get('state_age_sec',math.inf)
        safe=safe and isinstance(state_age,(int,float)) and math.isfinite(state_age) and 0<=state_age<2
        if not safe: return self.fail('SDK safety/control check failed') if self.state!='WAIT_READY' else 'stop'
        if self.state=='WAIT_READY':
            if s.get('motion_pending') is not False or s.get('command_zero') is not True: return 'stop'
            values=[s.get(k) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
            if any(not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>.03 for v in values):
                self.stop_sequences.clear(); return 'stop'
            cap=s.get('stair_speed_cap',0)
            if not isinstance(cap,(int,float)) or not math.isfinite(cap) or cap<self.speed:
                return self.fail('bridge Stair speed cap is lower than requested speed')
            self.stop_sequences.add(s.get('state_sequence'))
            if len(self.stop_sequences)<2: return 'stop'
            if not isinstance(s.get('state_sequence'),int) or not s.get('bridge_session'): return 'stop'
            self.session=s['bridge_session']; self.epoch=s['safety_epoch']
            self.sequence=s['state_sequence']; self.request_id=s.get('pose_request_id',-1)
            self.state='WAIT_MODE'; self.deadline=now+12
            return 'request_stair'
        if self.state=='WAIT_MODE':
            if (s.get('pose_request_id',-1)>self.request_id and s.get('pose_confirmed_id')==s.get('pose_request_id')
                and s.get('state_sequence',-1)>self.sequence and s.get('motion')=='Stair' and s.get('motion_pending') is False):
                self.state='WAIT_Y'; return 'prompt'
            return 'stop'
        if s.get('motion')!='Stair' or s.get('motion_pending') is not False:
            return self.fail('confirmed Stair posture lost')
        if self.state=='RUN':
            if now-self.started>=self.duration:
                self.state='DONE'; self.reason='time limit reached; landing NOT inferred'; return 'stop'
            return 'forward'
        return 'stop'
    def answer(self,answer,now):
        if self.state!='WAIT_Y': return False
        if answer.strip().lower()!='y': self.fail('operator declined or input closed'); return False
        self.state='RUN'; self.started=now; return True
