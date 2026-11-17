#!/usr/bin/env python3
"""Navigate to an explicit map-frame goal and finish after stopping."""
import argparse,json,math,signal,sys,time,os,fcntl
from pathlib import Path
import yaml

MAX_LINEAR_SPEED = 0.15
MAX_ANGULAR_SPEED = 0.4


class WalkGate:
 def __init__(self):
  self.request_before=None; self.sequences=set()
 def reset_confirmation(self):
  self.sequences.clear()
 def update(self,sdk):
  if sdk.get('motion_pending') is not False:
   self.reset_confirmation(); return 'wait'
  if sdk.get('motion')!='Walk':
   self.reset_confirmation()
   if self.request_before is None:
    self.request_before=sdk.get('pose_request_id',-1); return 'request'
   return 'wait'
  if self.request_before is not None and (sdk.get('pose_request_id',-1)<=self.request_before or sdk.get('pose_confirmed_id')!=sdk.get('pose_request_id')):
   self.reset_confirmation(); return 'wait'
  seq=sdk.get('state_sequence')
  if not isinstance(seq,int):return 'wait'
  self.sequences.add(seq)
  return 'ready' if len(self.sequences)>=2 else 'wait'


def main():
 root=Path(__file__).resolve().parents[1]; cfg=yaml.safe_load((root/'config/site.yaml').read_text())
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--live',action='store_true',required=True)
 for key in ('x', 'y', 'yaw'):
  p.add_argument('--' + key, type=float, required=True)
 p.add_argument('--z', type=float, default=0., help='Expected base height in map coordinates, metres')
 p.add_argument('--scene', default='old')
 p.add_argument('--max-runtime',type=float,default=10.,help='Maximum seconds from goal submission through stopped confirmation (default: 10)')
 p.add_argument('--path-file',type=Path,help='Server-owned approved path; execute FollowPath without replanning')
 p.add_argument('--owner-pid',type=int,help='Stop if the owning web process exits')
 args=p.parse_args()
 if not math.isfinite(args.max_runtime) or args.max_runtime<=0: p.error('--max-runtime must be finite and positive')
 target={key:getattr(args,key) for key in ('x','y','z','yaw')}
 if not all(math.isfinite(v) for v in target.values()): p.error('target coordinates must be finite')
 (root/'work').mkdir(exist_ok=True)
 lease=(root/'work/navigation_mission.lock').open('a')
 try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:p.error('another navigation supervisor already owns this repository')
 approved=None
 if args.path_file:
  approved=json.loads(args.path_file.read_text())
  values=approved.get('poses',[])
  if not values or not all(all(isinstance(v.get(k),(float,int)) and math.isfinite(v[k]) for k in ('x','y','yaw')) for v in values):p.error('invalid approved path')
  if math.hypot(values[-1]['x']-target['x'],values[-1]['y']-target['y'])>.001:p.error('path endpoint does not match goal')
 from scene_registry import scene_bundle
 from site_identity import verify
 scene=scene_bundle(root,args.scene)
 map_id=verify(root) if args.scene=='old' else scene['map_id']
 if not cfg.get('maps_validated') or not cfg.get('scan_source_validated'):
  p.error('review the selected map and current scan, then run validate_site.py')
 import rclpy
 from rclpy.node import Node
 from rclpy.action import ActionClient
 from geometry_msgs.msg import PoseStamped,Twist
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import LaserScan
 from nav2_msgs.action import NavigateToPose,FollowPath
 from nav_msgs.msg import Path as NavPath
 from std_msgs.msg import String
 from action_msgs.msg import GoalStatus
 from rclpy.signals import SignalHandlerOptions
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n=Node('d1_nav_supervisor')
 interrupted=False
 def on_signal(*_):
  nonlocal interrupted
  interrupted=True
 signal.signal(signal.SIGINT,on_signal); signal.signal(signal.SIGTERM,on_signal)
 vel=n.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
 action=n.create_publisher(String,'/d1/sdk/d1_action',1)
 client=ActionClient(n,FollowPath if approved else NavigateToPose,'/follow_path' if approved else '/navigate_to_pose')
 sdk={}; loc={}; pose=None; raw=None; odom=None
 sdk_at=loc_at=pose_at=raw_at=odom_at=scan_at=-math.inf
 sdk_identity=None; loc_identity=None; walk_gate=WalkGate()
 execution_started=None; execution_deadline=None; next_countdown=None; runtime_expired=False
 state='WAIT'; deadline=time.monotonic()+60; goal=None; result=None; terminal=False; failed=''; stop_sequences=set(); halt_at=None
 scanhealth={}; health_at=-math.inf; next_wait_log=0.
 def zero():
  vel.publish(Twist())
 def fault(reason):
  nonlocal state,failed
  if state=='FAULT': return
  state='FAULT'; failed=reason; zero()
  if goal: goal.cancel_goal_async()
 def on_sdk(m):
  nonlocal sdk,sdk_at
  try:
   d=json.loads(m.data)
   if isinstance(d,dict): sdk=d; sdk_at=time.monotonic()
  except ValueError: pass
 def on_loc(m):
  nonlocal loc,loc_at
  try:
   d=json.loads(m.data)
   if isinstance(d,dict): loc=d; loc_at=time.monotonic()
  except ValueError: pass
 def on_health(m):
  nonlocal scanhealth,health_at
  try:
   d=json.loads(m.data)
   if isinstance(d,dict): scanhealth=d; health_at=time.monotonic()
  except ValueError: pass
 def stamp_age(h): return n.get_clock().now().nanoseconds*1e-9-h.stamp.sec-h.stamp.nanosec*1e-9
 def on_pose(m):
  nonlocal pose,pose_at
  if m.header.frame_id=='map' and -.05<stamp_age(m.header)<.4: pose=m; pose_at=time.monotonic()
 def on_raw(m):
  nonlocal raw,raw_at
  raw=m; raw_at=time.monotonic()
 def on_odom(m):
  nonlocal odom,odom_at
  if m.header.frame_id=='nav_odom' and m.child_frame_id=='nav_base' and -.05<stamp_age(m.header)<.4: odom=m; odom_at=time.monotonic()
 def on_scan(m):
  nonlocal scan_at
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if m.header.frame_id=='nav_base' and -.05<n.get_clock().now().nanoseconds*1e-9-t<.4 and len(m.ranges)==720:
   scan_at=time.monotonic()
 n.create_subscription(String,'/d1/sdk/d1_motion_status',on_sdk,1)
 n.create_subscription(String,'/d1/localization/status',on_loc,1)
 n.create_subscription(String,'/d1/nav/scan_status',on_health,1)
 n.create_subscription(PoseStamped,'/d1/localization/pose6d',on_pose,1)
 n.create_subscription(Twist,'/d1/nav/cmd_vel_raw',on_raw,1)
 n.create_subscription(Odometry,'/d1/localization/odom',on_odom,1)
 from rclpy.qos import qos_profile_sensor_data
 n.create_subscription(LaserScan,'/d1/nav/scan',on_scan,qos_profile_sensor_data)
 def accepted(future):
  nonlocal goal,result,state
  try:
   goal=future.result()
   if not goal.accepted: fault('Nav2 rejected goal'); return
   result=goal.get_result_async()
   if state=='FAULT': goal.cancel_goal_async()
   else:
    state='NAVIGATE'; print('[ACCEPTED] 执行模块已接收路线，开始受监督的速度输出。',flush=True)
  except Exception as e: fault('Nav2 goal request failed: '+str(e))
 def base_safe(now):
  return (now-sdk_at<.4 and all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh'))
   and all(sdk.get(k) is False for k in ('soft_stop','hardware_stop'))
   and isinstance(sdk.get('state_age_sec'),(int,float)) and 0<=sdk['state_age_sec']<2)
 def localization_safe(now):
  if loc.get('map_id')!=map_id: return False
  if not (now-loc_at<.4 and loc.get('ready') is True and now-pose_at<.4 and now-scan_at<.5 and now-odom_at<.4 and now-health_at<.5 and scanhealth.get('ready') is True): return False
  if pose is None: return False
  q=pose.pose.orientation
  tilt=2*math.acos(min(1.,math.sqrt(q.w*q.w+q.z*q.z)))
  return tilt<math.radians(10) and abs(pose.pose.position.z-target['z'])<.3
 def at_target():
  q=pose.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  angle=math.atan2(math.sin(yaw-target['yaw']),math.cos(yaw-target['yaw']))
  return math.hypot(pose.pose.position.x-target['x'],pose.pose.position.y-target['y'])<.13 and abs(angle)<.13
 print('Navigating to map goal: '+str(target),flush=True)
 print('[LIMIT] max-runtime=%.1fs; starts at goal submission, includes acceptance/navigation/stopping; countdown every 5s.' % args.max_runtime,flush=True)
 try:
  while rclpy.ok() and state not in ('FAULT','DONE'):
   rclpy.spin_once(n,timeout_sec=.03); now=time.monotonic()
   if interrupted: fault('[CANCELED] 操作者取消或服务关闭'); continue
   if args.owner_pid:
    try:os.kill(args.owner_pid,0)
    except ProcessLookupError:fault('[OWNER_LOST] 网页服务进程已退出');continue
   if n.count_publishers('/d1/sdk/supervised_cmd_vel')>1:fault('[MULTIPLE_CONTROLLERS] 检测到多个速度发布者');continue
   if execution_deadline is not None and now>=execution_deadline:
    runtime_expired=True
    fault('[MAX_RUNTIME_EXCEEDED] 最大运行时间 %.1fs 已到，已运行 %.2fs；取消任务并发送最后一次零速，随后停止所有速度发布，不自动恢复。' % (args.max_runtime,now-execution_started))
    continue
   if next_countdown is not None and now>=next_countdown:
    print('[COUNTDOWN] 执行剩余 %.1fs / 上限 %.1fs；状态 %s' % (max(0.,execution_deadline-now),args.max_runtime,state),flush=True)
    next_countdown=execution_started+(math.floor((now-execution_started)/5)+1)*5
   if now>deadline:
    position=None if pose is None else [pose.pose.position.x,pose.pose.position.y]
    fault('mission deadline exceeded in '+state+'; position='+str(position)+'; SDK command_zero='+str(sdk.get('command_zero')))
    continue
   if sdk_identity is not None and (sdk.get('bridge_session'),sdk.get('safety_epoch'))!=sdk_identity: fault('SDK session/safety changed'); continue
   if loc_identity is not None and (loc.get('session'),loc.get('epoch'))!=loc_identity: fault('localization session/epoch changed; start a new task'); continue
   if state not in ('WAIT',) and (not base_safe(now) or not localization_safe(now)):
    fault('navigation SDK/localization/scan check failed; SDK=%s localization=%s scan=%s ages(sdk/loc/pose/scan/odom/health)=%s' % (sdk.get('reason'),loc.get('reason'),scanhealth.get('reason'),[round(now-v,3) for v in (sdk_at,loc_at,pose_at,scan_at,odom_at,health_at)]));continue
   if state=='WAIT':
    zero()
    if now>=next_wait_log:
     blockers=[]
     if not base_safe(now):blockers.append('SDK连接/控制权/急停/状态新鲜度')
     if not localization_safe(now):blockers.append('定位/地图身份/扫描/里程计/倾斜或高度')
     if sdk.get('command_zero') is not True:blockers.append('SDK命令尚非零速')
     measured=[sdk.get(k) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
     if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.03 for v in measured):blockers.append('SDK实测速率未停稳，要求各轴绝对值<0.03，当前='+str(measured))
     if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False:blockers.append('等待Walk姿态确认')
     if not client.server_is_ready():blockers.append('执行动作服务未就绪')
     print('[WAIT] '+('；'.join(blockers) or '等待两个新鲜停稳/Walk状态确认')+'；准备阶段最多60s',flush=True);next_wait_log=now+5
    if not base_safe(now) or not localization_safe(now) or sdk.get('command_zero') is not True: stop_sequences.clear(); walk_gate.reset_confirmation(); continue
    stopped_values=[sdk.get(k,math.inf) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.03 for v in stopped_values): stop_sequences.clear(); walk_gate.reset_confirmation(); continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2: continue
    sdk_identity=(sdk.get('bridge_session'),sdk.get('safety_epoch')); loc_identity=(loc.get('session'),loc.get('epoch'))
    mode=walk_gate.update(sdk)
    if mode=='request':
     print('[WALK] Requesting StandUp once; holding zero until two confirmed Walk state reports.',flush=True)
     action.publish(String(data='standup')); continue
    if mode!='ready':continue
    if not client.server_is_ready(): continue
    if approved:
     bound=approved['plan']
     if (loc.get('map_id'),loc.get('session'),loc.get('epoch'))!=(bound['map_id'],bound['localization_session'],bound['localization_epoch']):fault('[STALE_PLAN] 规划后定位身份改变');continue
     q=pose.pose.orientation;current_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
     yaw_change=math.atan2(math.sin(current_yaw-bound['start']['yaw']),math.cos(current_yaw-bound['start']['yaw']))
     if math.hypot(pose.pose.position.x-bound['start']['x'],pose.pose.position.y-bound['start']['y'])>.15 or abs(yaw_change)>.25:fault('[STALE_PLAN] 准备期间起点或朝向变化，请重新规划');continue
     message=FollowPath.Goal();message.path=NavPath();message.path.header.frame_id='map';message.path.header.stamp=n.get_clock().now().to_msg()
     for v in approved['poses']:
      pnt=PoseStamped();pnt.header=message.path.header;pnt.pose.position.x=float(v['x']);pnt.pose.position.y=float(v['y']);pnt.pose.orientation.z=math.sin(v['yaw']/2);pnt.pose.orientation.w=math.cos(v['yaw']/2);message.path.poses.append(pnt)
     message.path.poses[-1].pose.orientation.z=math.sin(target['yaw']/2);message.path.poses[-1].pose.orientation.w=math.cos(target['yaw']/2)
     message.controller_id='FollowPath';message.goal_checker_id='goal_checker'
     print('[PATH] 提交已确认的预览路线，共 %d 个点；终点朝向 %.4f rad' % (len(message.path.poses),target['yaw']),flush=True)
    else:
     message=NavigateToPose.Goal(); message.pose=PoseStamped(); message.pose.header.frame_id='map'; message.pose.header.stamp=n.get_clock().now().to_msg()
     message.pose.pose.position.x=target['x']; message.pose.pose.position.y=target['y']
     message.pose.pose.orientation.z=math.sin(target['yaw']/2); message.pose.pose.orientation.w=math.cos(target['yaw']/2)
     message.behavior_tree=str(root/'config/navigate.xml')
    state='ACCEPT'
    execution_started=now; execution_deadline=now+args.max_runtime; next_countdown=now+5; deadline=execution_deadline
    print('[START] Walk 已稳定确认%s；最大运行时间 %.1fs，剩余 %.1fs；到时自动终止并停止速度发布。' % ('，未重复发送 standup' if walk_gate.request_before is None else '',args.max_runtime,args.max_runtime),flush=True)
    client.send_goal_async(message).add_done_callback(accepted)
   elif state in ('NAVIGATE','ACCEPT'):
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False: fault('Walk mode lost'); continue
    if result and result.done():
     terminal=True
     if result.result().status!=GoalStatus.STATUS_SUCCEEDED:
      response=result.result();detail=response.result
      fault('[CONTROLLER_FAILED] Nav2 status=%s error_code=%s message=%s；详见 work/navigation_nav2.log' % (response.status,getattr(detail,'error_code','unavailable'),getattr(detail,'error_msg','unavailable')));continue
     print('[STOPPING] 控制器报告到达，开始零速与实际停稳复核。',flush=True)
     state='STOPPING'; deadline=now+10; halt_at=now; stop_sequences.clear(); zero(); continue
    if state=='NAVIGATE' and raw is not None and now-raw_at<.15:
     values=(raw.linear.x,raw.linear.y,raw.linear.z,raw.angular.x,raw.angular.y,raw.angular.z)
     if not all(math.isfinite(v) for v in values) or not 0<=raw.linear.x<=MAX_LINEAR_SPEED or any(abs(v)>1e-6 for v in values[1:5]):
      fault('unsupported Nav2 velocity: linear=(%.4f, %.4f, %.4f) angular=(%.4f, %.4f, %.4f)' % values); continue
     raw.angular.z=max(-MAX_ANGULAR_SPEED,min(MAX_ANGULAR_SPEED,raw.angular.z))
     vel.publish(raw)
    else: zero()
   elif state=='STOPPING':
    zero()
    # Terminal result is required; cancel acknowledgement alone is not sufficient.
    if not terminal or now-halt_at<.7 or not at_target() or now-odom_at>.4: continue
    t=odom.twist.twist
    if not all(math.isfinite(v) and abs(v)<.03 for v in (t.linear.x,t.linear.y,t.angular.z)): stop_sequences.clear(); continue
    if not all(isinstance(sdk.get(k),(int,float)) and math.isfinite(sdk[k]) and abs(sdk[k])<.03 for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')) or sdk.get('command_zero') is not True: stop_sequences.clear(); continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2: continue
    state='DONE'
  print(state+': '+(failed or 'goal reached and stopped'),flush=True)
 except KeyboardInterrupt: fault('operator interrupted')
 except Exception as e:
  fault(type(e).__name__+': '+str(e));print('FAULT: '+failed,flush=True)
 finally:
  if goal and not terminal: goal.cancel_goal_async()
  shutdown=time.monotonic()+1
  while rclpy.ok() and time.monotonic()<shutdown:
   # On runtime expiry, fault() sent the final zero. Drain late action replies
   # (and cancel accepted goals) without publishing any further Twist.
   if not runtime_expired and (execution_deadline is None or time.monotonic()<execution_deadline):zero()
   rclpy.spin_once(n,timeout_sec=.03)
  n.destroy_node()
  if rclpy.ok(): rclpy.shutdown()
 if interrupted: return 130
 if state!='DONE': return 1
 return 0
if __name__=='__main__': sys.exit(main())
