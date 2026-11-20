#!/usr/bin/env python3
"""One leased mission: existing flat navigation, isolated Stair, next floor."""
import argparse, collections, fcntl, json, math, os, signal, subprocess, sys, threading, time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from floor_routes import load_registry, read_state, write_state
from geometry import pose_matrix, imu_from_base, navigation_pose
from stair_completion import LandingDetector
from landing_line import bind_arrival_line
from stair_monitor import StairMonitor, StairMonitorFault, monitor_config, wrap


def main():
    root=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true',required=True)
    p.add_argument('--path-file',type=Path,required=True)
    p.add_argument('--max-runtime',type=float,required=True)
    p.add_argument('--owner-pid',type=int)
    a=p.parse_args()
    if not math.isfinite(a.max_runtime) or a.max_runtime<=0:p.error('invalid runtime')
    approved=json.loads(a.path_file.read_text());plan=approved['plan'];mission=plan['mission']
    from scene_registry import scene_bundle
    from site_identity import verify
    scene=scene_bundle(root,plan['scene']);registry=load_registry(root,scene)
    if verify(root)!=plan['map_id'] or registry['revision']!=mission['registry_revision']:
        raise ValueError('跨层配置或地图已变化，请重新规划')
    lease=(root/'work/navigation_mission.lock').open('a')
    fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
    task_id=a.path_file.name.removesuffix('.path.json')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,Twist
    from std_msgs.msg import String,Bool
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n=rclpy.create_node('d1_multifloor_mission');stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    values={};raw_rows=collections.deque(maxlen=30);I_B=imu_from_base(root/'config/calibration.yaml')
    runtime=yaml.safe_load((root/'config/runtime.yaml').read_text())
    def jsonmsg(key,m):
        try:values[key]=(json.loads(m.data),time.monotonic())
        except ValueError:pass
    def rawmsg(m):
        try:
            if m.header.frame_id!=runtime['odom_frame'] or m.child_frame_id!=runtime['body_frame']:raise ValueError('raw frame mismatch')
            T=navigation_pose(pose_matrix(m.pose.pose)@I_B)
            roll,pitch,yaw=Rotation.from_matrix(T[:3,:3]).as_euler('xyz')
            sample=dict(x=float(T[0,3]),y=float(T[1,3]),z=float(T[2,3]),roll=float(roll),pitch=float(pitch),yaw=float(yaw),
                        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
            values['raw']=(sample,time.monotonic());raw_rows.append(sample)
        except (ValueError,TypeError):values.pop('raw',None)
    def posemsg(m):
        if m.header.frame_id=='map':
            q=m.pose.orientation
            values['pose']=(dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9),time.monotonic())
    n.create_subscription(String,'/d1/sdk/d1_motion_status',lambda m:jsonmsg('sdk',m),1)
    n.create_subscription(String,'/d1/localization/status',lambda m:jsonmsg('loc',m),1)
    n.create_subscription(Odometry,'/Odometry',rawmsg,qos_profile_sensor_data)
    n.create_subscription(PoseStamped,'/d1/localization/pose6d',posemsg,10)
    spin=threading.Thread(target=rclpy.spin,args=(n,),daemon=True);spin.start()
    child=None;velocity=permit=action=None;identity=None;loc_identity=None;connection=None
    deadline=time.monotonic()+a.max_runtime;phase='PREPARING'
    def event(name,reason='',**extra):
        nonlocal phase
        phase=name
        print('[MULTIFLOOR_EVENT] '+json.dumps(dict(task_id=task_id,state=name,reason=reason,**extra),ensure_ascii=False),flush=True)
    def check():
        if stop.is_set():raise InterruptedError('操作者取消')
        if time.monotonic()>=deadline:raise RuntimeError('跨层任务总时限到达，未确认完成')
        if a.owner_pid:
            try:os.kill(a.owner_pid,0)
            except OSError:raise RuntimeError('任务所属网页进程已退出')
    def fresh(key,age=.4):
        value,at=values.get(key,({},-math.inf))
        if time.monotonic()-at>age:raise RuntimeError(key+' 数据过期')
        if key in ('raw','pose'):
            source=n.get_clock().now().nanoseconds*1e-9-value['stamp']
            if not -.1<=source<.5:raise RuntimeError(key+' 源时间戳过期')
        return value
    def sdk_safe():
        s=fresh('sdk')
        if not all(s.get(k) is True for k in ('sdk_connected','control_held','state_fresh')) or any(s.get(k) is not False for k in ('soft_stop','hardware_stop')):
            raise RuntimeError('SDK 控制权、急停或状态检查未通过')
        if identity and (s.get('bridge_session'),s.get('safety_epoch'))!=identity:raise RuntimeError('SDK 会话或安全状态变化')
        return s
    def zero():
        if velocity:velocity.publish(Twist());permit.publish(Bool(data=False))
    def stationary():
        s=sdk_safe();r=fresh('raw')
        rows=[x for x in list(raw_rows) if r['stamp']-x['stamp']<=1.2]
        if len(rows)<5 or rows[-1]['stamp']-rows[0]['stamp']<.8:return False
        if any(np.linalg.norm([x[k]-rows[0][k] for k in ('x','y','z')])>.03 for x in rows):return False
        return s.get('command_zero') is True and all(isinstance(s.get(k),(float,int)) and abs(s[k])<.03 for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate'))
    def settle():
        until=time.monotonic()+12;sequences=set()
        while True:
            check();zero()
            if stationary():sequences.add(sdk_safe()['state_sequence'])
            else:sequences.clear()
            if len(sequences)>=2:return
            if time.monotonic()>until:raise RuntimeError('未在时限内确认停稳')
            time.sleep(.04)
    def mode(request,expected):
        s=sdk_safe();before=s['pose_request_id'];sequence=s['state_sequence']
        action.publish(String(data=request));until=time.monotonic()+12
        while True:
            check();zero();s=sdk_safe()
            if (s.get('pose_request_id',-1)>before and s.get('pose_confirmed_id')==s.get('pose_request_id') and
                s['state_sequence']>sequence and s['motion']==expected and s.get('motion_pending') is False):return
            if time.monotonic()>until:raise RuntimeError(expected+' 实际模式未确认')
            time.sleep(.04)
    def flat(goal,fid,path=None):
        nonlocal child
        check();pose=fresh('pose')
        command=[sys.executable,str(root/'scripts/navigate_to_goal.py'),'--live','--scene',plan['scene'],
            '--floor',fid,'--x',str(goal['x']),'--y',str(goal['y']),'--yaw',str(goal['yaw']),
            '--z',str(pose['z']),'--max-runtime',str(max(.1,deadline-time.monotonic())),
            '--owner-pid',str(os.getpid()),'--lease-fd',str(lease.fileno())]
        if path:command+=['--path-file',str(path)]
        child=subprocess.Popen(command,cwd=root,pass_fds=(lease.fileno(),))
        while child.poll() is None:check();time.sleep(.1)
        code=child.returncode;child=None
        if code:raise RuntimeError('平地导航未成功，退出码 '+str(code))
    try:
        until=time.monotonic()+8
        while not all(k in values for k in ('sdk','raw','loc','pose')):
            check()
            if time.monotonic()>until:raise RuntimeError('等待定位与 SDK 超时')
            time.sleep(.05)
        s=sdk_safe();identity=(s['bridge_session'],s['safety_epoch'])
        loc=fresh('loc');loc_identity=(loc.get('session'),loc.get('epoch'),loc.get('map_id'))
        if loc_identity!=(plan['localization_session'],plan['localization_epoch'],plan['map_id']):raise RuntimeError('规划定位身份已变化')
        if read_state(root,registry)['actual_floor']!=mission['source_floor']:raise RuntimeError('出发楼层已变化')
        for c in mission['connections']:
            if s.get('stair_speed_cap',0)<c['control']['cruise_speed']:raise RuntimeError('桥接楼梯速度上限不足')
            if c['control']['heading_correction'] and s.get('stair_yaw_rate_cap',0)<monitor_config(c)['max_yaw_rate']:
                raise RuntimeError('桥接不支持配置的楼梯航向纠偏上限')
        print('[START] 跨楼层任务开始，总时限包含平地、上楼与停稳。',flush=True)
        for index,c in enumerate(mission['connections']):
            connection=c
            write_state(root,registry,target_floor=mission['target_floor'],connection_id=c['id'],phase='APPROACH')
            event('APPROACH','导航至上楼预备点',connection_id=c['id'])
            flat(c['entry'],c['from_floor'],a.path_file if index==0 else None)
            event('STAIR_PREPARE','确认到点、停稳，准备切换 Stair')
            # Publishers are created only AFTER the flat child relinquishes control.
            velocity=n.create_publisher(Twist,'/d1/sdk/stair_cmd_vel',1)
            permit=n.create_publisher(Bool,'/d1/sdk/stair_motion_permit',1)
            action=n.create_publisher(String,'/d1/sdk/d1_action',1)
            settle()
            pnow=fresh('pose')
            angle=math.atan2(math.sin(pnow['yaw']-c['entry']['yaw']),math.cos(pnow['yaw']-c['entry']['yaw']))
            if math.hypot(pnow['x']-c['entry']['x'],pnow['y']-c['entry']['y'])>.35 or abs(angle)>.10:
                raise RuntimeError('预备点位置或朝向复核未通过')
            mode('stair','Stair')
            write_state(root,registry,actual_floor=None,phase='STAIR',from_floor=c['from_floor'],to_floor=c['to_floor'])
            # Freeze the calibrated map heading in continuous LIO odometry ONCE.
            # Match raw yaw to the map pose timestamp; never follow later map yaw corrections.
            aligned_pose=fresh('pose');origin=dict(fresh('raw'))
            reference_sample=min(list(raw_rows),key=lambda r:abs(r['stamp']-aligned_pose['stamp']))
            if abs(reference_sample['stamp']-aligned_pose['stamp'])>.10:
                raise RuntimeError('预备点地图与里程计时间未对齐，不能建立楼梯方向基准')
            reference_yaw=wrap(reference_sample['yaw']+wrap(c['entry']['yaw']-aligned_pose['yaw']))
            monitor=StairMonitor(c,origin,reference_yaw)
            event('STAIR','固定FAST-LIO方向上楼，启用限幅航向监控',reference_yaw=reference_yaw,heading_correction=c['control']['heading_correction'])
            detector=LandingDetector(c);detector.origin=origin;detector.reference_yaw=reference_yaw
            detector.line=bind_arrival_line(c,aligned_pose,reference_sample,origin)
            last_print=0.;began=time.monotonic();ctl=c['control'];candidate_since=None
            while True:
                check();s=sdk_safe();sample=dict(fresh('raw'));loc=fresh('loc')
                if (loc.get('session'),loc.get('epoch'),loc.get('map_id'))!=loc_identity:raise RuntimeError('上楼期间定位身份发生变化')
                if loc.get('tracking_valid') is not True:raise RuntimeError('上楼期间三维里程计跟踪失效')
                if s['motion']!='Stair' or s.get('motion_pending') is not False or s.get('stair_fault') is True:raise RuntimeError('Stair 模式丢失')
                if n.count_publishers('/d1/sdk/stair_cmd_vel')>1 or n.count_publishers('/d1/sdk/supervised_cmd_vel'):
                    raise RuntimeError('楼梯段出现其他运动发布者')
                if time.monotonic()-began>ctl['max_runtime']:
                    raise RuntimeError('楼梯执行时限到达，未确认平台')
                sample['connection_id']=c['id']
                try:sample['map_pose']=fresh('pose') if loc.get('ready') is True else None
                except RuntimeError:sample['map_pose']=None
                # Target floor evidence is never the XY coordinate alone.
                expected_z=registry['floors'][c['to_floor']].get('base_z')
                if expected_z is not None and sample['map_pose']:
                    sample['candidate_floor']=c['to_floor'] if abs(sample['map_pose']['z']-expected_z)<.3 else None
                elif detector.evidence.get('posture_pass'):
                    sample['candidate_floor']=c['to_floor']
                finished=detector.update(sample)
                base_speed=ctl['landing_speed'] if detector.evidence.get('posture_pass') else ctl['cruise_speed']
                velocity_value,yaw_value,heading_evidence=monitor.update(sample,base_speed)
                if heading_evidence.get('forward_m',0)>ctl['max_travel_m'] or heading_evidence.get('rise_m',0)>ctl['max_rise_m']:
                    raise RuntimeError('楼梯距离或高度上限到达，未确认平台')
                if finished:break
                permit.publish(Bool(data=True));command=Twist();command.linear.x=velocity_value;command.angular.z=yaw_value;velocity.publish(command)
                if time.monotonic()-last_print>1:
                    event('STAIR','楼梯方向监控：偏航 %.1f°，横偏 %.2fm，纠偏 %.3frad/s，速度 %.2fm/s'%(math.degrees(heading_evidence.get('heading_error_rad',0)),heading_evidence.get('lateral_m',0),yaw_value,velocity_value),evidence=detector.evidence,heading_monitor=heading_evidence,speed=velocity_value);last_print=time.monotonic()
                time.sleep(.04)
            event('LANDING_STOP','平台候选条件满足，零速复核',evidence=detector.evidence)
            zero();settle()
            if not (abs(fresh('raw')['roll'])<=c['completion']['level_roll_rad'] and abs(fresh('raw')['pitch'])<=c['completion']['level_pitch_rad']):
                raise RuntimeError('停车后未保持水平，不能交接平地导航')
            event('WALK_CONFIRM','停稳后恢复 Walk');mode('standup','Walk');settle()
            n.destroy_publisher(velocity);n.destroy_publisher(permit);n.destroy_publisher(action)
            velocity=permit=action=None
            pose=fresh('pose');loc=fresh('loc')
            if loc.get('ready') is not True:raise RuntimeError('平台定位未就绪，已停止，等待手动初值')
            st=read_state(root,registry)
            write_state(root,registry,actual_floor=c['to_floor'],phase='MAP_SWITCH',source='stair_landing_inferred',
                base_z_by_floor={**st.get('base_z_by_floor',{}),c['to_floor']:pose['z']})
            event('MAP_SWITCH','保持定位，加载目标楼层导航地图')
            from floor_map_control import activate_maps
            activate_maps(n,root,scene,registry,c['to_floor'])
            write_state(root,registry,phase='ON_FLOOR',connection_id=None)
        if not mission.get('transition_only'):
            event('DESTINATION','在目标楼层继续平地导航');flat(plan['goal'],mission['target_floor'])
        write_state(root,registry,phase='ON_FLOOR',target_floor=None,connection_id=None)
        event('SUCCEEDED','跨层任务完成并确认停稳')
        return 0
    except (Exception,KeyboardInterrupt) as e:
        zero()
        if isinstance(e,StairMonitorFault):event('STAIR_MONITOR_FAULT',str(e),heading_monitor=e.evidence)
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
        if connection:
            write_state(root,registry,phase='CANCELED' if stop.is_set() else 'FAULT',reason=str(e))
        event('CANCELED' if stop.is_set() else 'FAILED','FAULT: '+str(e))
        print('FAULT: '+str(e),flush=True)
        return 130 if stop.is_set() else 1
    finally:
        until=time.monotonic()+.5
        while time.monotonic()<until:zero();time.sleep(.04)
        n.destroy_node();rclpy.try_shutdown();spin.join(timeout=2);lease.close()


if __name__=='__main__':sys.exit(main())
