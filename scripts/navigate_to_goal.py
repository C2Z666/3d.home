#!/usr/bin/env python3
"""Navigate to an explicit map-frame goal and finish after stopping."""
import argparse,json,math,signal,sys,time,os,fcntl
from pathlib import Path
import yaml,hashlib
from mission_runtime import RecoveryBudget, MissionTrace
from terminal_feedback import TerminalApproach, TerminalYaw
from navigation_control import HeadingAlignment, MotionGuard, rotation_clearance, quaternion_yaw, wrap, initial_path_heading, scan_input_state, POSITION_TOLERANCE, POSITION_RELEASE, replan_errors, settled_goal_action, SettledGoalReview, RECOVERABLE_INPUT_PHASES

MAX_LINEAR_SPEED = 0.30
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
 nav_cfg=yaml.safe_load((root/'config/nav2.yaml').read_text())
 endpoint_tolerance=float(nav_cfg['planner_server']['ros__parameters']['GridBased']['tolerance'])
 if not 0<=endpoint_tolerance<=POSITION_TOLERANCE:raise ValueError('planner endpoint tolerance must fit arrival tolerance')
 configured_goal=nav_cfg['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
 if abs(configured_goal-POSITION_TOLERANCE)>1e-6:raise ValueError('controller and supervisor position tolerances disagree')
 if 'RotateToGoal' in nav_cfg['controller_server']['ros__parameters']['FollowPath']['critics']:raise ValueError('Final yaw belongs to supervisor; disable DWB RotateToGoal')
 body=nav_cfg['local_costmap']['local_costmap']['ros__parameters']
 padding=body['footprint_padding']
 turn_radius=max(math.hypot(abs(x)+padding,abs(y)+padding) for x,y in json.loads(body['footprint']))
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--live',action='store_true',required=True)
 for key in ('x', 'y', 'yaw'):
  p.add_argument('--' + key, type=float, required=True)
 p.add_argument('--z', type=float, default=0., help='Expected base height in map coordinates, metres')
 p.add_argument('--scene', default='old')
 p.add_argument('--max-runtime',type=float,default=10.,help='Maximum seconds from goal submission through stopped confirmation (default: 10)')
 p.add_argument('--path-file',type=Path,help='Approved initial path; bounded endpoint reacquisition may replan to the same goal')
 p.add_argument('--owner-pid',type=int,help='Stop if the owning web process exits')
 p.add_argument('--lease-fd',type=int,help=argparse.SUPPRESS)
 p.add_argument('--floor',help='Floor identity of this flat navigation segment')
 args=p.parse_args()
 if not math.isfinite(args.max_runtime) or args.max_runtime<=0: p.error('--max-runtime must be finite and positive')
 target={key:getattr(args,key) for key in ('x','y','z','yaw')}
 if not all(math.isfinite(v) for v in target.values()): p.error('target coordinates must be finite')
 (root/'work').mkdir(exist_ok=True)
 if args.lease_fd is None:
  lease=(root/'work/navigation_mission.lock').open('a')
  try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except BlockingIOError:p.error('another navigation supervisor already owns this repository')
 else:
  lease=os.fdopen(os.dup(args.lease_fd),'a')
  if os.fstat(lease.fileno()).st_ino!=(root/'work/navigation_mission.lock').stat().st_ino:p.error('invalid inherited mission lease')
  try:fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except BlockingIOError:p.error('inherited descriptor does not own the mission lease')
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
 from floor_routes import load_registry,read_state,floor_id
 floor_registry=load_registry(root,scene);floor_state=read_state(root,floor_registry)
 segment_floor=floor_id(floor_registry,args.floor) if args.floor else floor_state['actual_floor']
 if not segment_floor or floor_state['active_nav_floor']!=segment_floor or floor_state['actual_floor']!=segment_floor:
  p.error('机器人实际楼层与导航地图不一致，禁止平地导航')
 if not cfg.get('maps_validated') or not cfg.get('scan_source_validated'):
  p.error('review the selected map and current scan, then run validate_site.py')
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import qos_profile_sensor_data
 from rclpy.action import ActionClient
 from geometry_msgs.msg import PoseStamped,Twist
 from nav_msgs.msg import Odometry,OccupancyGrid
 from sensor_msgs.msg import LaserScan
 from nav2_msgs.action import NavigateToPose,FollowPath,ComputePathToPose
 from nav_msgs.msg import Path as NavPath
 from std_msgs.msg import String
 from action_msgs.msg import GoalStatus
 from rclpy.signals import SignalHandlerOptions
 from rclpy.time import Time
 from tf2_ros import Buffer,TransformListener
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n=Node('d1_nav_supervisor')
 tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,n)
 interrupted=False
 def on_signal(*_):
  nonlocal interrupted
  interrupted=True
 signal.signal(signal.SIGINT,on_signal); signal.signal(signal.SIGTERM,on_signal)
 vel=n.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
 action=n.create_publisher(String,'/d1/sdk/d1_action',1)
 client=ActionClient(n,FollowPath if approved else NavigateToPose,'/follow_path' if approved else '/navigate_to_pose')
 planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
 sdk={}; loc={}; pose=None; raw=None; odom=None
 sdk_at=loc_at=pose_at=raw_at=odom_at=scan_at=-math.inf
 sdk_identity=None; loc_identity=None; walk_gate=WalkGate()
 yaw_priority_latched=False; yaw_position_limit=.35
 terminal_approach=None; approach_attempted=False; terminal_started=None; yaw_attempts=0; yaw_feedback=TerminalYaw(); control_at=-math.inf; next_feedback_log=0.
 execution_started=None; execution_deadline=None; next_countdown=None; runtime_expired=False
 state='WAIT'; deadline=time.monotonic()+60; goal=None; result=None; terminal=False; failed=''; stop_sequences=set(); halt_at=None
 scan_stamp=None;scanhealth={}; health_at=-math.inf; next_wait_log=0.
 costmap=None; costmap_at=-math.inf; alignment=None; motion_guard=MotionGuard(); settled_review=SettledGoalReview(); next_control_log=0.
 raw_controller=None; controller_at=-math.inf; command_missing_at=None
 pending_message=None; align_halt_at=None
 first_command_seen=False
 reacquire_count=0; planner_goal=None; planner_reply=None; planner_result=None
 route_reply=None; paused_state=None; stable_since=None; paused_sequence=None; paused_stamp=None; paused_at=None
 next_turn_check_log=0.
 budget=RecoveryBudget();pause_category='data';next_sample=0.;last_sent=(0.,0.);raw_lio=None;raw_lio_at=-math.inf
 task_id=args.path_file.name.removesuffix('.path.json') if args.path_file else 'cli-'+str(os.getpid())
 trace=MissionTrace(root/'work/navigation',task_id)
 trace.emit('mission',goal=target,scene=args.scene,map_id=map_id,max_runtime=args.max_runtime,source_hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ('scripts/navigate_to_goal.py','scripts/mission_runtime.py','config/calibration.yaml','config/nav2.yaml')},bridge_actual_semantics='normalized SDK Move request; transport counters do not prove physical execution')
 def transition(next_state,reason=''):
  nonlocal state,raw,raw_at,command_missing_at,first_command_seen,yaw_priority_latched,terminal_started
  old=state
  if next_state=='STOPPING' and old in ('WAIT','REPLAN','NAVIGATE','ACCEPT'):
   yaw_priority_latched=True
   if terminal_started is None:terminal_started=time.monotonic()
  if old==next_state:return
  if next_state in ('FAULT','INPUT_PAUSE','REACQUIRE_STOP','REPLAN','STOPPING','DONE','ACCEPT'):
   zero();raw=None;raw_at=-math.inf
  if next_state in ('ACCEPT','REACQUIRE_STOP'):
   motion_guard.reset();command_missing_at=None;first_command_seen=False
  if next_state in ('STOPPING','REACQUIRE_STOP','INPUT_PAUSE'):settled_review.reset();stop_sequences.clear()
  if next_state in ('FAULT','INPUT_PAUSE','REACQUIRE_STOP','STOPPING','DONE'):trace.route_status('history')
  if next_state in ('FAULT','INPUT_PAUSE') and goal and not terminal:goal.cancel_goal_async()
  if next_state=='ALIGN_YAW':trace.route_status('position_reached')
  state=next_state
  event=dict(previous=old,state=state,reason=reason,recovery=budget.snapshot(time.monotonic()))
  trace.emit('transition',**event);print('[NAV_EVENT] '+json.dumps(dict(task_id=task_id,**event),ensure_ascii=False),flush=True)
 def begin_alignment(*a,**kw):
  if kw.get('check_goal_position',True):
   kw['position_limit']=yaw_position_limit if yaw_priority_latched else POSITION_RELEASE
   kw['yaw_priority']=yaw_priority_latched
  motion_guard.reset();return HeadingAlignment(*a,**kw)
 def source_age(header):
  return n.get_clock().now().nanoseconds/1e9-header.stamp.sec-header.stamp.nanosec*1e-9
 def age(now,stamp):return round(now-stamp,4) if math.isfinite(stamp) else None
 def transform_snapshot():
  try:
   t=tf_buffer.lookup_transform('map','nav_base',Time());p=t.transform.translation
   return dict(stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9,source_age=source_age(t.header),position=[p.x,p.y,p.z],yaw=quaternion_yaw(t.transform.rotation))
  except Exception as e:return dict(error=str(e))
 def observe(now):
  nonlocal next_sample
  if now<next_sample:return
  next_sample=now+.2
  t=odom.twist.twist if odom else None
  trace.sample(state=state,recovery=budget.snapshot(now),goal=target,
   map_pose=current_pose() if pose else None,odom_pose=current_odom() if odom else None,
   raw_fastlio=raw_lio,correction=loc.get('correction'),localization_mode=loc.get('mode'),tracking_fault=loc.get('tracking_fault'),
   commands=dict(dwb=None if raw_controller is None else [raw_controller.linear.x,raw_controller.angular.z],smoothed=None if raw is None else [raw.linear.x,raw.angular.z],supervised=list(last_sent),bridge_actual=None if sdk.get('sdk_move_normalized_lr_fb_yaw') is None else dict(normalized_lr_fb_yaw=sdk['sdk_move_normalized_lr_fb_yaw'],sequence=sdk.get('move_sequence'),age_sec=sdk.get('move_age_sec'),write_ok=sdk.get('move_write_ok'),write_failed=sdk.get('move_write_failed'))),
   measured=None if t is None else [t.linear.x,t.linear.y,t.angular.z],
   sdk={k:sdk.get(k) for k in ('state_sequence','state_age_sec','reported_forward_speed','reported_lateral_speed','reported_yaw_rate','command_zero','motion','bridge_target','bridge_ramped','sdk_move_normalized_lr_fb_yaw','move_sequence','move_age_sec','cmd_age_sec','move_write_ok','move_write_failed','speed_level','limits_vx_wz_ax_aw')},
   receive_age={k:age(now,v) for k,v in dict(localization=loc_at,pose=pose_at,scan=scan_at,odom=odom_at,scan_health=health_at,sdk=sdk_at,dwb=controller_at,smoothed=raw_at,raw_fastlio=raw_lio_at).items()},
   source_age=dict(scan=None if scan_stamp is None else n.get_clock().now().nanoseconds/1e9-scan_stamp,raw_fastlio=None if raw_lio is None else n.get_clock().now().nanoseconds/1e9-raw_lio['stamp'],pose=source_age(pose.header) if pose else None,odom=source_age(odom.header) if odom else None,localization=None if loc.get('source_stamp') is None else n.get_clock().now().nanoseconds/1e9-loc['source_stamp']),
   scan_health=scanhealth,tf=transform_snapshot(),tf_reason=tf_reason(),diagnostic_error=trace.error)
 def on_raw_lio(m):
  nonlocal raw_lio,raw_lio_at
  q=m.pose.pose.orientation;p=m.pose.pose.position
  raw_lio=dict(frame=m.header.frame_id,child_frame=m.child_frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]);raw_lio_at=time.monotonic()
 n.create_subscription(Odometry,'/Odometry',on_raw_lio,qos_profile_sensor_data)

 def zero():
  nonlocal last_sent
  last_sent=(0.,0.);vel.publish(Twist())
 def fault(reason):
  nonlocal state,failed
  if state=='FAULT': return
  transition('FAULT',reason); failed=reason;trace.emit('fault',reason=reason); zero()
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
 def on_controller(m):
  nonlocal raw_controller,controller_at
  raw_controller=m;controller_at=time.monotonic()
 def on_costmap(m):
  nonlocal costmap,costmap_at
  if m.header.frame_id=='nav_odom' and -.05<stamp_age(m.header)<.5:
   costmap=m;costmap_at=time.monotonic()
 def on_odom(m):
  nonlocal odom,odom_at
  if m.header.frame_id=='nav_odom' and m.child_frame_id=='nav_base' and -.05<stamp_age(m.header)<.4: odom=m; odom_at=time.monotonic()
 def on_scan(m):
  nonlocal scan_at,scan_stamp
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if m.header.frame_id=='nav_base' and -.05<n.get_clock().now().nanoseconds*1e-9-t<.4 and len(m.ranges)==720:
   scan_at=time.monotonic();scan_stamp=t
 n.create_subscription(String,'/d1/sdk/d1_motion_status',on_sdk,1)
 n.create_subscription(String,'/d1/localization/status',on_loc,1)
 n.create_subscription(String,'/d1/nav/scan_status',on_health,1)
 n.create_subscription(PoseStamped,'/d1/localization/pose6d',on_pose,1)
 n.create_subscription(Twist,'/d1/nav/cmd_vel_smoothed',on_raw,1)
 n.create_subscription(Twist,'/d1/nav/cmd_vel_raw',on_controller,1)
 n.create_subscription(OccupancyGrid,'/local_costmap/costmap',on_costmap,1)
 n.create_subscription(Odometry,'/d1/localization/odom',on_odom,1)
 from rclpy.qos import qos_profile_sensor_data
 n.create_subscription(LaserScan,'/d1/nav/scan',on_scan,qos_profile_sensor_data)
 def accepted(future):
  nonlocal goal,result,state
  try:
   if future is not route_reply:
    stale_goal=future.result()
    if stale_goal.accepted:stale_goal.cancel_goal_async()
    return
   goal=future.result()
   if not goal.accepted: fault('Nav2 rejected goal'); return
   result=goal.get_result_async()
   if state in ('FAULT','INPUT_PAUSE'): goal.cancel_goal_async()
   else:
    transition('NAVIGATE');trace.route_status('executing'); print('[ACCEPTED] 执行模块已接收路线，开始受监督的速度输出。',flush=True)
  except Exception as e: fault('Nav2 goal request failed: '+str(e))
 def submit_route():
  nonlocal state,raw,raw_at,command_missing_at,first_command_seen,route_reply,goal,result,terminal
  # A result belongs to one action only. Never inspect a completed old action
  # while waiting for the next route's acceptance callback.
  goal=None;result=None;terminal=False
  transition('ACCEPT');raw=None;raw_at=-math.inf;command_missing_at=None;first_command_seen=False
  motion_guard.reset()
  # Initial alignment can take seconds. Never submit a path with that old TF time.
  stamp=n.get_clock().now().to_msg()
  if approved:
   pending_message.path.header.stamp=stamp
   for point in pending_message.path.poses:point.header.stamp=stamp
  else:pending_message.pose.header.stamp=stamp
  print('[TRACK_PATH] 提交%s路线给DWB。' % ('终点复核后重新规划的' if reacquire_count else '已确认的预览'),flush=True)
  if approved:trace.set_route([[p.pose.position.x,p.pose.position.y] for p in pending_message.path.poses],map_id,args.scene)
  route_reply=client.send_goal_async(pending_message);route_reply.add_done_callback(accepted)
 def pause_inputs(reason,now,category='data'):
  nonlocal state,paused_state,deadline,stable_since,paused_sequence,paused_stamp,paused_at,pause_category
  zero()
  if state=='INPUT_PAUSE':return
  pause_category=category;budget.begin_wait(now,category)
  paused_state=state;transition('INPUT_PAUSE',reason);stable_since=None
  paused_at=now;motion_guard.reset();settled_review.reset()
  paused_sequence=sdk.get('state_sequence');paused_stamp=loc.get('source_stamp')
  deadline=min(execution_deadline,now+30.)
  if goal and not terminal:goal.cancel_goal_async()
  print('[INPUT_PAUSE] %s；立即保持零速，取消旧路线；最多等待30秒，要求新数据稳定0.5秒并停稳后恢复，总时限不重置。' % reason,flush=True)
 def base_safe(now):
  return (now-sdk_at<.4 and all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh'))
   and all(sdk.get(k) is False for k in ('soft_stop','hardware_stop'))
   and isinstance(sdk.get('state_age_sec'),(int,float)) and 0<=sdk['state_age_sec']<2)
 def localization_safe(now):
  if loc.get('map_id')!=map_id: return False
  if not (now-loc_at<.4 and loc.get('ready') is True and now-pose_at<.4 and now-scan_at<.5 and now-odom_at<.4 and now-health_at<.5 and scan_input_state(scanhealth)[0]): return False
  if pose is None: return False
  q=pose.pose.orientation
  tilt=2*math.acos(min(1.,math.sqrt(q.w*q.w+q.z*q.z)))
  return tilt<math.radians(10) and abs(pose.pose.position.z-target['z'])<.3
 def at_target():
  q=pose.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  angle=math.atan2(math.sin(yaw-target['yaw']),math.cos(yaw-target['yaw']))
  return math.hypot(pose.pose.position.x-target['x'],pose.pose.position.y-target['y'])<=(yaw_position_limit if yaw_priority_latched else POSITION_RELEASE) and abs(angle)<.10
 def tf_reason():
  try:
   t=tf_buffer.lookup_transform('map','nav_base',Time())
   age=stamp_age(t.header)
   if not -.05<age<.4:return 'map -> nav_base TF年龄 %.3fs，要求小于0.4s' % age
   if pose is not None:
    p=t.transform.translation;q=t.transform.rotation
    if math.hypot(p.x-pose.pose.position.x,p.y-pose.pose.position.y)>.15 or abs(wrap(quaternion_yaw(q)-quaternion_yaw(pose.pose.orientation)))>.15:
     return 'Nav2 TF与网页融合定位不一致'
   return ''
  except Exception as e:return str(e)
 def current_pose():
  return [pose.pose.position.x,pose.pose.position.y,quaternion_yaw(pose.pose.orientation)]
 def current_odom():
  return [odom.pose.pose.position.x,odom.pose.pose.position.y,quaternion_yaw(odom.pose.pose.orientation)]
 def stopped():
  t=odom.twist.twist
  return (all(math.isfinite(v) and abs(v)<.03 for v in (t.linear.x,t.linear.y,t.angular.z))
    and sdk.get('command_zero') is True
    and all(isinstance(sdk.get(k),(int,float)) and math.isfinite(sdk[k]) and abs(sdk[k])<.03
            for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')))
 def reacquire(reason,now,category=None,charged=False):
  nonlocal state,deadline,halt_at,reacquire_count,raw,raw_at,terminal
  # Called after the path action ended, or during pre-path alignment before
  # any action has started. No
  # concurrent old path is left running; obstacle/source/SDK failures remain
  # hard stops. The same mission deadline and final map goal are retained.
  zero()
  if state=='ALIGN_PATH' and goal is None and (route_reply is None or route_reply.done()):
   terminal=True  # No path controller is running; safe to replan after settling.
  if not terminal:
   fault('[REACQUIRE_INVALID_STATE] 路径执行尚未结束');return
  if category is None:category='motion' if reason.startswith(('[ALIGN_ODOM_DRIFT]','[ZERO_TRANSLATION_DRIFT]','[ROUTE_SPIN]','[NO_MOTION_RESPONSE]')) else 'position'
  if category!='data' and not charged:
   error=budget.request(category)
   if error:fault(error+'；'+reason);return
  reacquire_count+=1;transition('REACQUIRE_STOP',reason);halt_at=now;deadline=min(execution_deadline,now+10)
  raw=None;raw_at=-math.inf;stop_sequences.clear()
  print('[REACQUIRE_STOP] %s；类别=%s 计数=%s；停稳后重规划，总时限不重置。' % (reason,category,json.dumps(budget.snapshot(now),ensure_ascii=False)),flush=True)
 def turn_clear(now):
  nonlocal next_turn_check_log
  if not costmap_fresh(now):
   return '局部代价地图超过0.5秒未更新，禁止终点转向'
  p=odom.pose.pose.position
  details={};reason=rotation_clearance(costmap,(p.x,p.y),turn_radius,details)
  if reason or now>=next_turn_check_log:
   print('[TURN_CHECK] '+json.dumps(dict(phase=state,**details),ensure_ascii=False),flush=True)
   next_turn_check_log=now+1.
  if reason:
   # Preserve the map that ACTUALLY blocked motion. A later live map can have
   # different dynamic points and cannot explain an earlier collision check.
   label=args.path_file.name.replace('.path.json','') if args.path_file else str(os.getpid())
   snapshot=root/'work/navigation'/(label+'.turn_clearance.json');snapshot.parent.mkdir(exist_ok=True)
   info=costmap.info;q=info.origin.orientation
   record=dict(reason=reason,phase=state,ros_now=n.get_clock().now().nanoseconds/1e9,
               odom_pose=current_odom(),map_pose=current_pose(),target=target,check=details,
               frame=costmap.header.frame_id,stamp=costmap.header.stamp.sec+costmap.header.stamp.nanosec*1e-9,
               resolution=info.resolution,width=info.width,height=info.height,
               origin=dict(x=info.origin.position.x,y=info.origin.position.y,q=[q.x,q.y,q.z,q.w]),data=list(costmap.data))
   try:snapshot.write_text(json.dumps(record,ensure_ascii=False));print('[TURN_CHECK_SNAPSHOT] '+str(snapshot),flush=True)
   except OSError as e:print('[TURN_CHECK_SNAPSHOT_FAILED] '+str(e),flush=True)
  return reason
 def costmap_fresh(now):
  return costmap is not None and now-costmap_at<.5 and -.05<stamp_age(costmap.header)<.5
 def output(vx,wz,now):
  nonlocal next_control_log,last_sent
  t=odom.twist.twist
  measured=(t.linear.x,t.linear.y,t.angular.z)
  reason=motion_guard.check(vx,wz,measured,now,current_odom()[:2],current_odom()[2] if state=='NAVIGATE' else None,zero_translation_limit=None if state=='ALIGN_YAW' and yaw_priority_latched else .12)
  if reason.startswith(('[NO_MOTION_RESPONSE]','[ZERO_TRANSLATION_DRIFT]','[ROUTE_SPIN]')) and state=='NAVIGATE':
   error=budget.request('motion')
   if error:fault(error+'；'+reason);return
   pause_inputs(reason,now,'motion');return
  if reason.startswith('[ZERO_TRANSLATION_DRIFT]') and state in ('ALIGN_PATH','ALIGN_YAW'):
   if state=='ALIGN_YAW':fault('[FINAL_ALIGNMENT_DRIFT] '+reason)
   else:reacquire(reason,now)
   return
  if reason:fault(reason);return
  msg=Twist();msg.linear.x=vx;msg.angular.z=wz;vel.publish(msg);last_sent=(vx,wz)
  if now>=next_control_log:
   pos=current_pose();distance=math.hypot(pos[0]-target['x'],pos[1]-target['y'])
   dwb=None if raw_controller is None else (round(raw_controller.linear.x,3),round(raw_controller.angular.z,3))
   heading=alignment.target[2] if state in ('ALIGN_PATH','ALIGN_YAW') else target['yaw']
   print('[CONTROL] phase=%s distance=%.3fm yaw_error=%.3frad DWB=%s sent=(%.3f,%.3f) measured=(%.3f,%.3f,%.3f)' % (state,distance,wrap(heading-pos[2]),dwb,vx,wz,*measured),flush=True)
   if alignment is not None and state in ('ALIGN_PATH','ALIGN_YAW'):
    print('[ALIGN_METRICS] '+json.dumps(alignment.diagnostics(pos,current_odom())),flush=True)
   next_control_log=now+1
 print('Navigating to map goal: '+str(target),flush=True)
 print('[CONTROLLER] DWB StandardTrajectoryGenerator -> OPEN_LOOP smoother -> supervisor -> SDK bridge; terminal_handoff=0.25m, position_envelope=0.35m, final_yaw_accept=0.10rad; source_feedback=10Hz observed, control_refresh<=20Hz, final_turn_cap=0.25rad/s; progress includes rotation.',flush=True)
 print('[LIMIT] max-runtime=%.1fs; starts at goal submission, includes acceptance/navigation/stopping; countdown every 5s.' % args.max_runtime,flush=True)
 try:
  while rclpy.ok() and state not in ('FAULT','DONE'):
   rclpy.spin_once(n,timeout_sec=.005); now=time.monotonic();observe(now)
   if now-control_at<.05:continue
   control_at=now
   if interrupted: fault('[CANCELED] 操作者取消或服务关闭'); continue
   if args.owner_pid:
    try:os.kill(args.owner_pid,0)
    except ProcessLookupError:fault('[OWNER_LOST] 网页服务进程已退出');continue
   if n.count_publishers('/d1/sdk/supervised_cmd_vel')>1:fault('[MULTIPLE_CONTROLLERS] 检测到多个速度发布者');continue
   current_floor=read_state(root,floor_registry)
   if current_floor['actual_floor']!=segment_floor or current_floor['active_nav_floor']!=segment_floor:
    fault('[FLOOR_CHANGED] 平地执行期间楼层或导航地图改变');continue
   if execution_deadline is not None and now>=execution_deadline:
    runtime_expired=True
    fault('[MAX_RUNTIME_EXCEEDED] 最大运行时间 %.1fs 已到，已运行 %.2fs；取消任务并发送最后一次零速，随后停止所有速度发布，不自动恢复。' % (args.max_runtime,now-execution_started))
    continue
   if next_countdown is not None and now>=next_countdown:
    print('[COUNTDOWN] 执行剩余 %.1fs / 上限 %.1fs；状态 %s' % (max(0.,execution_deadline-now),args.max_runtime,state),flush=True)
    next_countdown=execution_started+(math.floor((now-execution_started)/5)+1)*5
   if terminal_started is not None and now-terminal_started>45.:fault('[TERMINAL_TIMEOUT] 终点闭环超过45秒，未满足朝向和停稳结束条件');continue
   if state=='INPUT_PAUSE' and budget.wait_error(now):fault(budget.wait_error(now));continue
   if now>deadline:
    position=None if pose is None else [pose.pose.position.x,pose.pose.position.y]
    fault('[%s_TIMEOUT] 阶段等待超时；position=%s SDK_command_zero=%s' % (state,position,sdk.get('command_zero')))
    continue
   if sdk_identity is not None and (sdk.get('bridge_session'),sdk.get('safety_epoch'))!=sdk_identity: fault('SDK session/safety changed'); continue
   if loc_identity is not None and (loc.get('session'),loc.get('epoch'))!=loc_identity: fault('localization session/epoch changed; start a new task'); continue
   tf_block=tf_reason()
   if state!='WAIT':
    if not base_safe(now):fault('[SDK_NOT_READY] '+str(sdk.get('reason')));continue
    if loc.get('tracking_fault'):fault('[ODOMETRY_FAULT] '+loc['tracking_fault']);continue
    loc_ok=localization_safe(now)
    if not loc_ok or tf_block:
     # Recover only missing/stale streams, never an inconsistent frame, map,
     # height/tilt, explicit odometry fault, or an invalid sensor status.
     q=pose.pose.orientation if pose else None
     geometry_ok=q is not None and 2*math.acos(min(1.,math.sqrt(q.w*q.w+q.z*q.z)))<math.radians(10) and abs(pose.pose.position.z-target['z'])<.3
     scan_ok,scan_wait,scan_reason=scan_input_state(scanhealth)
     recoverable=(loc.get('map_id')==map_id and loc.get('mode') in ('FUSED','ODOM_ONLY','LOST')
                  and not loc.get('relocalizing') and scan_wait and geometry_ok
                  and '不一致' not in tf_block)
     why='TF=%s localization=%s scan_ready=%s scan_reason=%s scan_recoverable=%s geometry_ok=%s map_ok=%s ages(loc/pose/scan/odom/health)=%s' % (tf_block,loc.get('reason'),scan_ok,scan_reason,scan_wait,geometry_ok,loc.get('map_id')==map_id,[round(now-v,3) for v in (loc_at,pose_at,scan_at,odom_at,health_at)])
     if recoverable and state in RECOVERABLE_INPUT_PHASES:
      pause_inputs(why,now);stable_since=None;continue
     fault('[LOCALIZATION_NOT_READY] '+why);continue
    if state=='INPUT_PAUSE':
     zero()
     if not stopped() or sdk.get('state_sequence')==paused_sequence or loc.get('source_stamp')==paused_stamp or not costmap_fresh(now):stable_since=None;continue
     if stable_since is None:stable_since=now
     if now-stable_since<.5:continue
     if route_reply is not None and not route_reply.done():continue
     if goal and not terminal and (result is None or not result.done()):continue
     print('[INPUT_RECOVERED] 新数据连续稳定0.5秒、机器人已停稳，恢复前复核完成。',flush=True)
     budget.end_wait(now);deadline=execution_deadline
     if paused_state in ('NAVIGATE','ACCEPT'):
      terminal=True;goal=None;result=None;reacquire('零速等待恢复，丢弃旧路线和旧速度',now,category=pause_category,charged=pause_category=='motion')
     else:
      transition(paused_state)
      if state in ('ALIGN_PATH','ALIGN_YAW'):
       alignment.started+=now-paused_at;alignment.progress_at+=now-paused_at
      if state in ('STOPPING','REACQUIRE_STOP'):halt_at=now;deadline=min(execution_deadline,now+10);stop_sequences.clear()
      if state=='REPLAN':deadline=min(execution_deadline,now+10)
     continue
   if state=='WAIT':
    zero()
    if now>=next_wait_log:
     blockers=[]
     if not base_safe(now):blockers.append('SDK连接/控制权/急停/状态新鲜度')
     if not localization_safe(now):blockers.append('定位/地图身份/扫描/里程计/倾斜或高度')
     if tf_block:blockers.append('TF未就绪：'+tf_block)
     if not costmap_fresh(now):blockers.append('等待新鲜局部代价地图')
     if sdk.get('command_zero') is not True:blockers.append('SDK命令尚非零速')
     measured=[sdk.get(k) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
     if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.03 for v in measured):blockers.append('SDK实测速率未停稳，要求各轴绝对值<0.03，当前='+str(measured))
     if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False:blockers.append('等待Walk姿态确认')
     if not client.server_is_ready():blockers.append('执行动作服务未就绪')
     print('[WAIT] '+('；'.join(blockers) or '等待两个新鲜停稳/Walk状态确认')+'；准备阶段最多60s',flush=True);next_wait_log=now+5
    if not base_safe(now) or not localization_safe(now) or tf_block or not costmap_fresh(now) or sdk.get('command_zero') is not True: stop_sequences.clear(); walk_gate.reset_confirmation(); continue
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
     print('[PATH] 使用已确认的预览路线，共 %d 个点；终点朝向 %.4f rad' % (len(message.path.poses),target['yaw']),flush=True)
    else:
     message=NavigateToPose.Goal(); message.pose=PoseStamped(); message.pose.header.frame_id='map'; message.pose.header.stamp=n.get_clock().now().to_msg()
     message.pose.pose.position.x=target['x']; message.pose.pose.position.y=target['y']
     message.pose.pose.orientation.z=math.sin(target['yaw']/2); message.pose.pose.orientation.w=math.cos(target['yaw']/2)
     message.behavior_tree=str(root/'config/navigate.xml')
    pending_message=message
    execution_started=now; execution_deadline=now+args.max_runtime; next_countdown=now+5; deadline=execution_deadline
    print('[START] Walk 已稳定确认%s；最大运行时间 %.1fs，剩余 %.1fs；到时自动终止并停止速度发布。' % ('，未重复发送 standup' if walk_gate.request_before is None else '',args.max_runtime,args.max_runtime),flush=True)
    pos=current_pose();heading=initial_path_heading(approved['poses'],pos) if approved else pos[2]
    if math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=.25:
     alignment=None;terminal=True
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;zero()
     print('[YAW_PRIORITY] 已在25cm终点闭环范围，跳过路线起步转向；先停稳，再对齐最终朝向。',flush=True)
    elif abs(wrap(heading-pos[2]))>.35:
     alignment=begin_alignment(pos,[pos[0],pos[1],heading],now,current_odom(),check_goal_position=False);transition('ALIGN_PATH');align_halt_at=None
     print('[ALIGN_PATH] 先原地对准路线起步方向 %.3frad，速度上限0.30rad/s；计入本次最大运行时间。' % heading,flush=True)
    else:submit_route()
   elif state in ('NAVIGATE','ACCEPT'):
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False: fault('Walk mode lost'); continue
    pos=current_pose()
    if state=='NAVIGATE' and math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=.25:
     zero()
     if goal and not (result and result.done()):goal.cancel_goal_async()
     terminal=bool(result and result.done());alignment=None;raw=None;raw_at=-math.inf
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now
     print('[YAW_PRIORITY] 进入25cm终点闭环范围；停止平移并撤销DWB，等待动作结束和停稳后对齐朝向。',flush=True)
     continue
    if result and result.done():
     terminal=True
     if result.result().status!=GoalStatus.STATUS_SUCCEEDED:
      response=result.result();detail=response.result
      fault('[CONTROLLER_FAILED] Nav2 status=%s error_code=%s message=%s；详见 work/navigation_nav2.log' % (response.status,getattr(detail,'error_code','unavailable'),getattr(detail,'error_msg','unavailable')));continue
     pos=current_pose()
     if math.hypot(pos[0]-target['x'],pos[1]-target['y'])>POSITION_RELEASE:
      reacquire('[POSITION_NOT_REACHED] DWB报告到达，但融合定位终点距离未通过复核',now);continue
     alignment=None
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;zero()
     print('[PATH_BRAKE] 路径跟踪结束；先在零指令下确认停稳，再独立调整最终朝向。',flush=True)
     continue
    if state=='NAVIGATE' and raw is not None and now-raw_at<.15:
     values=(raw.linear.x,raw.linear.y,raw.linear.z,raw.angular.x,raw.angular.y,raw.angular.z)
     if not all(math.isfinite(v) for v in values) or not -1e-6<=raw.linear.x<=MAX_LINEAR_SPEED+1e-6 or any(abs(v)>1e-6 for v in values[1:5]):
      fault('unsupported Nav2 velocity: linear=(%.4f, %.4f, %.4f) angular=(%.4f, %.4f, %.4f)' % values); continue
     command_missing_at=None;first_command_seen=True
     output(max(0.,min(MAX_LINEAR_SPEED,raw.linear.x)),max(-MAX_ANGULAR_SPEED,min(MAX_ANGULAR_SPEED,raw.angular.z)),now)
    else:
     zero()
     if state=='NAVIGATE':
      if command_missing_at is None:
       command_missing_at=now
       if not first_command_seen:print('[WAIT_COMMAND] 等待首帧DWB平滑速度，最多3秒；保持零速，计入本次时限。',flush=True)
      allowance=1. if first_command_seen else 3.
      if now-command_missing_at>allowance:fault('[COMMAND_STREAM_LOST] DWB/速度平滑器持续%.1f秒没有新鲜输出；raw_age=%.2fs smoothed_age=%.2fs' % (allowance,now-controller_at,now-raw_at))
   elif state in ('ALIGN_PATH','ALIGN_YAW'):
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False: fault('Walk mode lost'); continue
    if state=='ALIGN_YAW' and at_target():
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;zero()
     print('[FINAL_POSE_ACCEPTABLE] 位置和朝向已在最终验收范围；直接停车复核，不追加转向。',flush=True)
     continue
    wz,reason=alignment.command(current_pose(),now,current_odom(),odom.twist.twist.angular.z)
    if reason.startswith('[ALIGN_ODOM_DRIFT]'):
     if state=='ALIGN_YAW':fault('[FINAL_ALIGNMENT_DRIFT] '+reason)
     else:reacquire(reason,now)
     continue
    if reason.startswith('[ALIGN_GOAL_SHIFT]') and state=='ALIGN_YAW':
     print('[GOAL_BOUNDARY_HOLD] '+reason+'；不立即消耗恢复次数，停稳后要求连续1秒新鲜数据复核。',flush=True)
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;stop_sequences.clear();settled_review.reset();zero();continue
    if reason:fault(reason);continue
    if state=='ALIGN_YAW':
     stamp=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
     wz,decision=yaw_feedback.command(wrap(target['yaw']-current_pose()[2]),odom.twist.twist.angular.z,stamp,now,source_age(pose.header))
     if now>=next_feedback_log:
      print('[CLOSED_LOOP_YAW] source_age=%.3fs decision=%s yaw_error=%.4f measured_wz=%.4f output_wz=%.4f' % (source_age(pose.header),decision,wrap(target['yaw']-current_pose()[2]),odom.twist.twist.angular.z,wz),flush=True);next_feedback_log=now+.5
     if decision=='WAIT_SAMPLE':zero();motion_guard.reset();continue
     if decision in ('INVALID_SAMPLE','STAMP_REGRESSION'):fault('[FEEDBACK_INVALID] '+decision);continue
    if wz==0.:
     if state=='ALIGN_PATH':
      zero()
      if align_halt_at is None:align_halt_at=now
      if now-align_halt_at>=.3 and abs(odom.twist.twist.angular.z)<.05:
       alignment=None;submit_route()
      continue
     print('[STOPPING] 已进入朝向容差或提前制动；保持零速，停稳后分别复核位置与朝向。',flush=True)
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;stop_sequences.clear();settled_review.reset();zero();continue
    reason=turn_clear(now)
    if reason:fault('[TURN_CLEARANCE] '+reason);continue
    align_halt_at=None
    output(0.,wz,now)
   elif state=='TERMINAL_APPROACH':
    zero_age=source_age(pose.header)
    stamp=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
    vx,decision=terminal_approach.command(current_pose(),[target['x'],target['y']],odom.twist.twist.linear.x,stamp,now,zero_age)
    wz=terminal_approach.wz
    if decision=='ADVANCE':
     if not costmap_fresh(now):zero();continue
     op=current_odom();blocked=''
     # Conservative swept padded-body disk, includes 0.15m/s upper speed and braking.
     for step in (0.,.05,.10,.15,.20):
      angle=wz*step/max(vx,.01)
      local_x=step if abs(wz)<1e-6 else vx/wz*math.sin(angle)
      local_y=0. if abs(wz)<1e-6 else vx/wz*(1-math.cos(angle))
      centre=(op[0]+local_x*math.cos(op[2])-local_y*math.sin(op[2]),op[1]+local_x*math.sin(op[2])+local_y*math.cos(op[2]))
      blocked=rotation_clearance(costmap,centre,turn_radius+.025)
      if blocked:break
     if blocked:vx=0.;wz=0.;decision='POSITION_BLOCKED:'+blocked
    if now>=next_feedback_log:
     print('[CLOSED_LOOP_POSITION] source_age=%.3fs decision=%s distance=%.3fm measured_vx=%.4f output_vx=%.3f output_wz=%.3f feedback=%s' % (zero_age,decision,math.hypot(current_pose()[0]-target['x'],current_pose()[1]-target['y']),odom.twist.twist.linear.x,vx,wz,json.dumps(terminal_approach.diagnostics)),flush=True);next_feedback_log=now+.5
    if decision=='WAIT_SAMPLE':zero();motion_guard.reset();continue
    if decision in ('INVALID_SAMPLE','STAMP_REGRESSION'):fault('[FEEDBACK_INVALID] '+decision);continue
    if decision=='ADVANCE':output(vx,wz,now)
    else:
     zero();transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now
     print('[POSITION_FINISHED] '+decision+'；位置微调结束，转入最终朝向，不重试位置。',flush=True)
   elif state=='STOPPING':
    zero()
    if not terminal and result is not None and result.done():
     terminal=True
     print('[PATH_RELEASED] 旧路径动作已结束；零速停稳复核后才允许最终转向。',flush=True)
    if not terminal or now-halt_at<.7:continue
    pos=current_pose();stamp=(pose.header.stamp.sec,pose.header.stamp.nanosec)
    stationary=stopped()
    if not stationary:stop_sequences.clear()
    action=settled_review.update(pos,[target['x'],target['y'],target['yaw']],now,stamp,stationary,sdk.get('state_sequence'),current_odom(),yaw_position_limit if yaw_priority_latched else POSITION_RELEASE)
    if action=='WAIT':continue
    if not approach_attempted:
     approach_attempted=True
     distance=math.hypot(pos[0]-target['x'],pos[1]-target['y'])
     bearing=wrap(math.atan2(target['y']-pos[1],target['x']-pos[0])-pos[2])
     yaw_reserve=max(8.,abs(wrap(target['yaw']-pos[2]))/.18+4.)
     approach_budget=min(12.,execution_deadline-now-yaw_reserve)
     if .13<distance<=.30 and abs(bearing)<=.6 and approach_budget>=2.:
      terminal_approach=TerminalApproach(now,pos,approach_budget);motion_guard.reset()
      transition('TERMINAL_APPROACH');deadline=execution_deadline
      print('[POSITION_ATTEMPT] 一次有界接近闭环，最多12秒；0.10至0.15m/s按响应调整，小幅转向修正；按朝向误差预留最终转向时间，不循环位置重试。',flush=True);continue
    print('[SETTLED_CHECK] stable=1.0s action=%s distance=%.3fm yaw_error=%.3frad' % (action,math.hypot(pos[0]-target['x'],pos[1]-target['y']),wrap(target['yaw']-pos[2])),flush=True)
    if action=='REPLAN' and yaw_priority_latched:
     fault('[FINAL_POSITION_ENVELOPE] 位置超过35cm；停止而不循环位置恢复。');continue
    if action=='REPLAN':
     reacquire('[STOPPING_POSITION_SHIFT] 停稳且连续1秒新鲜定位确认位置超过0.20m，重新接近目标',now);continue
    if action=='ALIGN_YAW':
     yaw_attempts+=1
     if yaw_attempts>4:fault('[YAW_SETTLE_LIMIT] 已进行4次朝向闭环与停稳复核，仍未达到0.10rad');continue
     yaw_feedback=TerminalYaw()
     if alignment is None:alignment=begin_alignment(pos,[target['x'],target['y'],target['yaw']],now,current_odom())
     print('[ALIGN_YAW] 制动停稳已确认，进入最终朝向调整。',flush=True)
     transition('ALIGN_YAW');deadline=execution_deadline;motion_guard.reset()
     print('[ALIGN_YAW_RESUME] 停稳连续复核位置合格，仅微调朝向，不消耗重新接近次数。',flush=True)
     continue
    # The same stable window already contains two distinct SDK reports.
    print('[FINAL_ERROR] position_error=%.3fm yaw_error=%.4frad; policy=yaw_priority position_limit=0.35m yaw_limit=0.10rad' % (math.hypot(pos[0]-target['x'],pos[1]-target['y']),wrap(target['yaw']-pos[2])),flush=True)
    if math.hypot(pos[0]-target['x'],pos[1]-target['y'])>POSITION_RELEASE:print('[POSITION_APPROXIMATE] 朝向完成，位置超过20cm；实际误差见FINAL_ERROR。',flush=True)
    transition('DONE')
   elif state=='REACQUIRE_STOP':
    zero()
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False:fault('Walk mode lost');continue
    if now-halt_at<.7 or not stopped() or not costmap_fresh(now):stop_sequences.clear();continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2:continue
    pos=current_pose()
    if math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=POSITION_TOLERANCE:
     alignment=begin_alignment(pos,[target['x'],target['y'],target['yaw']],now,current_odom())
     transition('ALIGN_YAW');deadline=execution_deadline;continue
    if not approved:
     # NavigateToPose owns planning in this optional CLI mode.
     result=None;goal=None;terminal=False;deadline=execution_deadline;submit_route();continue
    if not planner.server_is_ready():fault('[REPLAN_UNAVAILABLE] 路径规划服务不可用');continue
    request=ComputePathToPose.Goal();request.goal=PoseStamped();request.goal.header.frame_id='map'
    request.goal.header.stamp=n.get_clock().now().to_msg()
    request.goal.pose.position.x=target['x'];request.goal.pose.position.y=target['y']
    request.goal.pose.orientation.z=math.sin(target['yaw']/2);request.goal.pose.orientation.w=math.cos(target['yaw']/2)
    request.start=PoseStamped();request.start.header=request.goal.header
    request.start.pose.position.x=pos[0];request.start.pose.position.y=pos[1]
    request.start.pose.orientation.z=math.sin(pos[2]/2);request.start.pose.orientation.w=math.cos(pos[2]/2)
    request.use_start=True;request.planner_id='GridBased'
    planner_goal=None;planner_result=None;planner_reply=planner.send_goal_async(request)
    transition('REPLAN');deadline=min(execution_deadline,now+10)
    print('[REPLAN] 已停稳，使用当前融合位置、当前代价地图重新规划到原目标。',flush=True)
   elif state=='REPLAN':
    zero()
    if planner_reply.done() and planner_goal is None:
     planner_goal=planner_reply.result()
     if not planner_goal.accepted:fault('[REPLAN_REJECTED] 规划器拒绝请求');continue
     planner_result=planner_goal.get_result_async()
    if planner_result is None or not planner_result.done():continue
    response=planner_result.result();path=response.result.path
    if response.status!=GoalStatus.STATUS_SUCCEEDED or not path.poses or path.header.frame_id!='map':
     fault('[REPLAN_FAILED] 无可用路线，status=%s' % response.status);continue
    if any(p.header.frame_id not in ('','map') or not all(math.isfinite(v) for v in (p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w)) for p in path.poses):
     fault('[REPLAN_INVALID] 新路线坐标无效');continue
    endpoint=path.poses[-1].pose.position
    startpoint=path.poses[0].pose.position;pos=current_pose()
    replan_reason,errors=replan_errors((startpoint.x,startpoint.y),(endpoint.x,endpoint.y),pos,(target['x'],target['y']),endpoint_tolerance)
    print('[REPLAN_CHECK] '+json.dumps(errors),flush=True)
    if replan_reason:
     reacquire('['+replan_reason+'] '+json.dumps(errors),now);continue
    pending_message.path=path
    pending_message.path.poses[-1].pose.orientation=request.goal.pose.orientation
    result=None;goal=None;terminal=False;deadline=execution_deadline;alignment=None
    print('[REPLAN_READY] 新路线%d点，恢复DWB跟踪；仍受原任务剩余时间约束。' % len(path.poses),flush=True)
    points=[dict(x=p.pose.position.x,y=p.pose.position.y) for p in path.poses]
    heading=initial_path_heading(points,pos)
    if math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=.25:
     alignment=None;terminal=True
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now;zero()
     print('[YAW_PRIORITY] 已在25cm终点闭环范围，跳过路线起步转向；先停稳，再对齐最终朝向。',flush=True)
    elif abs(wrap(heading-pos[2]))>.35:
     alignment=begin_alignment(pos,[pos[0],pos[1],heading],now,current_odom(),check_goal_position=False)
     transition('ALIGN_PATH');align_halt_at=None
     print('[ALIGN_PATH] 重规划后对准新路线起步方向 %.3frad。' % heading,flush=True)
    else:submit_route()
  trace.emit('finished',state=state,reason=failed,recovery=budget.snapshot(time.monotonic()));trace.route_status('history')
  print(state+': '+(failed or 'goal reached and stopped'),flush=True)
 except KeyboardInterrupt: fault('operator interrupted')
 except Exception as e:
  fault(type(e).__name__+': '+str(e));print('FAULT: '+failed,flush=True)
 finally:
  if planner_goal and (planner_result is None or not planner_result.done()):planner_goal.cancel_goal_async()
  if goal and not terminal: goal.cancel_goal_async()
  shutdown=time.monotonic()+1
  while rclpy.ok() and time.monotonic()<shutdown:
   # On runtime expiry, fault() sent the final zero. Drain late action replies
   # (and cancel accepted goals) without publishing any further Twist.
   if not runtime_expired and (execution_deadline is None or time.monotonic()<execution_deadline):zero()
   rclpy.spin_once(n,timeout_sec=.03)
  trace.close();n.destroy_node()
  if rclpy.ok(): rclpy.shutdown()
 if interrupted: return 130
 if state!='DONE': return 1
 return 0
if __name__=='__main__': sys.exit(main())
