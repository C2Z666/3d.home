#!/usr/bin/env python3
"""Navigate to an explicit map-frame goal and finish after stopping."""
import argparse,json,math,signal,sys,time,os,fcntl
from pathlib import Path
import yaml,hashlib
import numpy as np
from geometry import imu_from_base,pose_matrix,navigation_pose
from route_validation import check_route,ReplanPolicy
from body_planner import plan_body,departure_heading,PlanningTimeout,turn_drift_margin
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from mission_runtime import RecoveryBudget, MissionTrace
from terminal_feedback import TerminalApproach, TerminalYaw, terminal_turn_position_limit
from turn_recovery import TurnWait
from navigation_control import HeadingAlignment, MotionGuard, swept_rotation_clearance, quaternion_yaw, wrap, initial_path_heading, scan_input_state, POSITION_TOLERANCE, POSITION_RELEASE, replan_errors, settled_goal_action, SettledGoalReview, RECOVERABLE_INPUT_PHASES

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
 replan_radius=float(yaml.safe_load((root/'config/replan.yaml').read_text())['replan_radius_m'])
 if not math.isfinite(replan_radius) or replan_radius<1.2:
  raise ValueError('replan_radius_m 必须为至少 1.2 m 的有限数值')
 nav_cfg=yaml.safe_load((root/'config/nav2.yaml').read_text())
 runtime_cfg=yaml.safe_load((root/'config/runtime.yaml').read_text());I_B=imu_from_base(root/'config/calibration.yaml');raw_geometry={}
 endpoint_tolerance=float(nav_cfg['planner_server']['ros__parameters']['GridBased']['tolerance'])
 if not 0<=endpoint_tolerance<=POSITION_TOLERANCE:raise ValueError('planner endpoint tolerance must fit arrival tolerance')
 configured_goal=nav_cfg['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
 if abs(configured_goal-POSITION_TOLERANCE)>1e-6:raise ValueError('controller and supervisor position tolerances disagree')
 if 'RotateToGoal' in nav_cfg['controller_server']['ros__parameters']['FollowPath']['critics']:raise ValueError('Final yaw belongs to supervisor; disable DWB RotateToGoal')
 body=nav_cfg['local_costmap']['local_costmap']['ros__parameters']
 padding=body['footprint_padding']
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--live',action='store_true',required=True)
 for key in ('x', 'y', 'yaw'):
  p.add_argument('--' + key, type=float, required=True)
 p.add_argument('--z', type=float, default=0., help='Start map Z for diagnostics only; flat arrival uses XY/yaw and stopped confirmation')
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
 # Every entry point, including post-stair segments without a saved preview,
 # uses the same certified FollowPath pipeline.
 auto_plan=approved is None
 if auto_plan:approved={'generated':True,'poses':[]}
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
 from nav2_msgs.action import FollowPath,ComputePathToPose
 from nav2_msgs.srv import GetCostmap
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
 client=ActionClient(n,FollowPath,'/follow_path')
 planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
 route_costmap=n.create_client(GetCostmap,'/global_costmap/get_costmap')
 body_executor=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context("spawn"));body_future=None;body_repaired=False;body_force=False
 route_future=None;route_requested=0.;route_next_check=0.;route_last_valid=None;route_after='prepare';route_details={}
 pending_replan_charge=False
 background_future=None;background_ready=None;background_retry_at=0.;route_grid=None
 replan_policy=ReplanPolicy();obstacle_at=None;obstacle_force_replan=False
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
 turn_wait=None; turn_replan_route=False
 budget=RecoveryBudget();pause_category='data';next_sample=0.;last_sent=(0.,0.);raw_lio=None;raw_lio_at=-math.inf
 task_id=args.path_file.name.removesuffix('.path.json') if args.path_file else 'cli-'+str(os.getpid())
 trace=MissionTrace(root/'work/navigation',task_id)
 trace.emit('mission',goal=target,scene=args.scene,map_id=map_id,max_runtime=args.max_runtime,source_hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ('scripts/navigate_to_goal.py','scripts/mission_runtime.py','config/calibration.yaml','config/nav2.yaml')},bridge_actual_semantics='normalized SDK Move request; transport counters do not prove physical execution')
 def transition(next_state,reason=''):
  nonlocal state,raw,raw_at,command_missing_at,first_command_seen,yaw_priority_latched,terminal_started
  old=state
  if alignment is not None and old in ('ALIGN_PATH','ALIGN_YAW') and next_state!=old:
   alignment.pause(time.monotonic())
  if alignment is not None and next_state in ('ALIGN_PATH','ALIGN_YAW'):
   alignment.resume(time.monotonic())
  if next_state=='STOPPING' and old in ('WAIT','REPLAN','NAVIGATE','ACCEPT'):
   yaw_priority_latched=True
   if terminal_started is None:terminal_started=time.monotonic()
  if old==next_state:return
  if next_state in ('FAULT','INPUT_PAUSE','REACQUIRE_STOP','REPLAN','STOPPING','DONE','ACCEPT','ROUTE_CHECK','OBSTACLE_WAIT'):
   zero();raw=None;raw_at=-math.inf
  if next_state in ('ACCEPT','REACQUIRE_STOP'):
   motion_guard.reset();command_missing_at=None;first_command_seen=False
  if next_state in ('STOPPING','REACQUIRE_STOP','INPUT_PAUSE'):settled_review.reset();stop_sequences.clear()
  if next_state in ('FAULT','INPUT_PAUSE','REACQUIRE_STOP','STOPPING','DONE','OBSTACLE_WAIT'):trace.route_status('history')
  if next_state in ('FAULT','INPUT_PAUSE','OBSTACLE_WAIT') and goal and not terminal:goal.cancel_goal_async()
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
   geometry=dict(raw_geometry,map_z=pose.pose.position.z if pose else None,start_map_z=target['z'],diagnostic_only=True),
   path_check=route_details,obstacle_replans=replan_policy.attempts,
   map_pose=current_pose() if pose else None,odom_pose=current_odom() if odom else None,
   raw_fastlio=raw_lio,correction=loc.get('correction'),localization_mode=loc.get('mode'),tracking_fault=loc.get('tracking_fault'),
   commands=dict(dwb=None if raw_controller is None else [raw_controller.linear.x,raw_controller.angular.z],smoothed=None if raw is None else [raw.linear.x,raw.angular.z],supervised=list(last_sent),bridge_actual=None if sdk.get('sdk_move_normalized_lr_fb_yaw') is None else dict(normalized_lr_fb_yaw=sdk['sdk_move_normalized_lr_fb_yaw'],sequence=sdk.get('move_sequence'),age_sec=sdk.get('move_age_sec'),write_ok=sdk.get('move_write_ok'),write_failed=sdk.get('move_write_failed'))),
   measured=None if t is None else [t.linear.x,t.linear.y,t.angular.z],
   sdk={k:sdk.get(k) for k in ('state_sequence','state_age_sec','reported_forward_speed','reported_lateral_speed','reported_yaw_rate','command_zero','motion','bridge_target','bridge_ramped','sdk_move_normalized_lr_fb_yaw','move_sequence','move_age_sec','cmd_age_sec','move_write_ok','move_write_failed','speed_level','limits_vx_wz_ax_aw')},
   receive_age={k:age(now,v) for k,v in dict(localization=loc_at,pose=pose_at,scan=scan_at,odom=odom_at,scan_health=health_at,sdk=sdk_at,dwb=controller_at,smoothed=raw_at,raw_fastlio=raw_lio_at).items()},
   source_age=dict(scan=None if scan_stamp is None else n.get_clock().now().nanoseconds/1e9-scan_stamp,raw_fastlio=None if raw_lio is None else n.get_clock().now().nanoseconds/1e9-raw_lio['stamp'],pose=source_age(pose.header) if pose else None,odom=source_age(odom.header) if odom else None,localization=None if loc.get('source_stamp') is None else n.get_clock().now().nanoseconds/1e9-loc['source_stamp']),
   scan_health=scanhealth,tf=transform_snapshot(),tf_reason=tf_reason(),diagnostic_error=trace.error)
 def on_raw_lio(m):
  nonlocal raw_lio,raw_lio_at,raw_geometry
  q=m.pose.pose.orientation;p=m.pose.pose.position
  raw_lio=dict(frame=m.header.frame_id,child_frame=m.child_frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w]);raw_lio_at=time.monotonic()
  if m.header.frame_id!=runtime_cfg['odom_frame'] or m.child_frame_id!=runtime_cfg['body_frame']:
   raw_geometry=dict(error='原始里程计坐标系不匹配');return
  try:
   N=navigation_pose(pose_matrix(m.pose.pose)@I_B)
   raw_geometry=dict(tilt_deg=math.degrees(math.acos(float(np.clip(N[2,2],-1.,1.)))),raw_z=float(N[2,3]))
  except (ValueError,TypeError) as e:raw_geometry=dict(error=str(e))
 n.create_subscription(Odometry,runtime_cfg['odom_topic'],on_raw_lio,qos_profile_sensor_data)

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
   if state in ('FAULT','INPUT_PAUSE','OBSTACLE_WAIT'): goal.cancel_goal_async()
   else:
    transition('NAVIGATE');trace.route_status('executing'); print('[ACCEPTED] 执行模块已接收路线，开始受监督的速度输出。',flush=True)
  except Exception as e: fault('Nav2 goal request failed: '+str(e))
 def dispatch_route():
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
  print('[TRACK_PATH] 提交%s路线给DWB。' % ('现场生成并通过机身检查的' if auto_plan else '终点复核后重新规划的' if reacquire_count else '已确认的预览'),flush=True)
  if approved:trace.set_route([[p.pose.position.x,p.pose.position.y] for p in pending_message.path.poses],map_id,args.scene)
  route_reply=client.send_goal_async(pending_message);route_reply.add_done_callback(accepted)
 def request_route_check(now):
  nonlocal route_future,route_requested
  if route_future is None:
   if not route_costmap.service_is_ready():raise RuntimeError('[PATH_CHECK_UNAVAILABLE] 路径代价地图服务不可用')
   route_future=route_costmap.call_async(GetCostmap.Request());route_requested=now
 def install_body_route(values,certificate):
  stamp=n.get_clock().now().to_msg();poses=[]
  for value in values:
   point=PoseStamped();point.header.frame_id='map';point.header.stamp=stamp
   point.pose.position.x=value['x'];point.pose.position.y=value['y']
   point.pose.orientation.z=math.sin(value['yaw']/2);point.pose.orientation.w=math.cos(value['yaw']/2)
   poses.append(point)
  pending_message.path.poses=poses;pending_message.path.header.stamp=stamp
  print('[REPLAN_READY] 已找到绕行路线%d点，等待最新地图复核。'%len(poses),flush=True)
  trace.emit('body_plan',**certificate)
 def poll_route_check(now):
  nonlocal route_future,route_last_valid,route_details,route_next_check,body_future,body_repaired,body_force,route_grid,pending_replan_charge
  if body_future is not None:
   if not body_future.done():return None
   future=body_future;body_future=None
   charged=pending_replan_charge;pending_replan_charge=False
   try:values,certificate=future.result()
   except ValueError as e:
    route_details=dict(reason='planning_timeout' if isinstance(e,PlanningTimeout) else 'body_planning_failed',message=str(e))
    if isinstance(e,PlanningTimeout):replan_policy.timeout(now,charged)
    route_last_valid=False;return False
   install_body_route(values,certificate)
   request_route_check(now);return None
  if route_future is None:return None
  if not route_future.done():
   if now-route_requested>3.:
    route_costmap.remove_pending_request(route_future);route_future=None
    raise RuntimeError('[PATH_CHECK_TIMEOUT] 代价地图检查超过3秒')
   return None
  m=route_future.result().map;route_future=None
  meta=m.metadata;q=meta.origin.orientation
  if m.header.frame_id!='map':raise RuntimeError('[PATH_CHECK_FRAME] 全局代价地图坐标系错误')
  grid=dict(width=meta.size_x,height=meta.size_y,resolution=meta.resolution,
            origin=[meta.origin.position.x,meta.origin.position.y,quaternion_yaw(q)],data=m.data)
  route_grid=grid
  points=[[p.pose.position.x,p.pose.position.y] for p in pending_message.path.poses]
  footprint=json.loads(body['footprint'])
  length=max(abs(p[0]) for p in footprint)+padding;width=max(abs(p[1]) for p in footprint)+padding
  ok,route_details=check_route(points,current_pose(),grid,length,width,goal_yaw=target['yaw'])
  if state=='ROUTE_CHECK' and not body_repaired and (body_force or not ok):
   body_repaired=True;body_force=False
   body_future=body_executor.submit(plan_body,grid,current_pose(),[target['x'],target['y'],target['yaw']],length,width)
   print('[BODY_REPLAN] 搜索机身可通行且远离墙角的路线，保持停车。',flush=True)
   return None
  route_last_valid=ok;route_next_check=now+1.;trace.emit('path_check',valid=ok,details=route_details)
  return ok
 def queue_route_check(after='prepare',force_plan=False):
  nonlocal route_after,route_future,route_last_valid,deadline,body_future,body_repaired,body_force,background_future
  if body_future is not None:body_future.cancel();body_future=None
  body_repaired=False;body_force=force_plan
  if not approved:
   dispatch_route();return
  if route_future is not None:route_costmap.remove_pending_request(route_future);route_future=None
  route_after=after;route_last_valid=None;transition('ROUTE_CHECK');deadline=min(execution_deadline,time.monotonic()+15.)
  if background_future is not None:
   body_future=background_future;background_future=None;body_repaired=True;body_force=False
  else:request_route_check(time.monotonic())
 def submit_route():queue_route_check('submit')
 def prepare_route():
  nonlocal alignment,terminal,deadline,halt_at,align_halt_at
  pos=current_pose();points=[dict(x=p.pose.position.x,y=p.pose.position.y) for p in pending_message.path.poses] if approved else []
  heading=departure_heading([[p['x'],p['y']] for p in points],pos) if approved else pos[2]
  deadline=execution_deadline
  if not turn_replan_route and math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=.25:
   alignment=None;terminal=True;transition('STOPPING');deadline=min(execution_deadline,time.monotonic()+10);halt_at=time.monotonic();zero()
  elif abs(wrap(heading-pos[2]))>.35:
   alignment=begin_alignment(pos,[pos[0],pos[1],heading],time.monotonic(),current_odom(),check_goal_position=False)
   transition('ALIGN_PATH');align_halt_at=None
   print('[ALIGN_PATH] 路线机身检查通过，先对准起步方向 %.3frad。'%heading,flush=True)
  else:dispatch_route()
 def obstacle_hold(reason,now,force=False):
  nonlocal obstacle_at,obstacle_force_replan,deadline,terminal,route_future,route_next_check,route_last_valid
  if state=='OBSTACLE_WAIT':return
  zero();obstacle_at=now;obstacle_force_replan=force;replan_policy.blocked(now);replan_policy.progress(now,0.,False)
  if route_future is not None:route_costmap.remove_pending_request(route_future);route_future=None
  route_last_valid=None;route_next_check=now;stop_sequences.clear()
  if goal is None and (route_reply is None or route_reply.done()):terminal=True
  transition('OBSTACLE_WAIT',reason);deadline=min(execution_deadline,now+20.)
  print('[OBSTACLE_WAIT] '+reason+'；撤销旧控制并停稳，等待3秒，仍阻挡则有界重规划。',flush=True)
 def pause_inputs(reason,now,category='data'):
  nonlocal state,paused_state,deadline,stable_since,paused_sequence,paused_stamp,paused_at,pause_category
  zero()
  if state=='INPUT_PAUSE':return
  pause_category=category;budget.begin_wait(now,category)
  paused_state=state;transition('INPUT_PAUSE',reason);stable_since=None
  paused_at=now;motion_guard.reset();settled_review.reset();replan_policy.progress(now,0.,False)
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
  return True
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
 def recover_terminal(reason,now):
  nonlocal alignment,approach_attempted,terminal_started,yaw_priority_latched,yaw_attempts,turn_replan_route
  reacquire(reason,now,category='position')
  if state=='REACQUIRE_STOP':
   alignment=None;approach_attempted=False;terminal_started=None
   yaw_priority_latched=False;yaw_attempts=0;turn_replan_route=True
 def turn_clear(now):
  nonlocal next_turn_check_log
  if not costmap_fresh(now):
   return '局部代价地图超过0.5秒未更新，禁止终点转向'
  delta=wrap(alignment.target[2]-current_pose()[2])
  footprint=json.loads(body['footprint'])
  half_length=max(abs(v[0]) for v in footprint)+padding
  half_width=max(abs(v[1]) for v in footprint)+padding
  coast=odom.twist.twist.angular.z*.3
  margin=turn_drift_margin(max(abs(delta),abs(coast)))
  details=dict(turn_drift_margin_m=margin)
  reason=swept_rotation_clearance(costmap,current_odom(),delta,half_length+margin,half_width+margin,details,coast_delta=coast)
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
   if replan_policy.stall_error(now):fault(replan_policy.stall_error(now));continue
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
     # Flat arrival is XY/yaw/stationary only. Height and tilt are diagnostics.
     scan_ok,scan_wait,scan_reason=scan_input_state(scanhealth)
     recoverable=(loc.get('map_id')==map_id and loc.get('mode') in ('FUSED','ODOM_ONLY','LOST')
                  and not loc.get('relocalizing') and scan_wait
                  and '不一致' not in tf_block)
     why='TF=%s localization=%s scan_ready=%s scan_reason=%s scan_recoverable=%s map_ok=%s ages(loc/pose/scan/odom/health)=%s' % (tf_block,loc.get('reason'),scan_ok,scan_reason,scan_wait,loc.get('map_id')==map_id,[round(now-v,3) for v in (loc_at,pose_at,scan_at,odom_at,health_at)])
     if recoverable and (state in RECOVERABLE_INPUT_PHASES or state in ('ROUTE_CHECK','OBSTACLE_WAIT')):
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
      if state=='TERMINAL_APPROACH' and terminal_approach is not None:
       terminal_approach.resume_after_pause(now-paused_at)
      if state in ('STOPPING','REACQUIRE_STOP'):halt_at=now;deadline=min(execution_deadline,now+10);stop_sequences.clear()
      if state in ('REPLAN','ROUTE_CHECK'):deadline=min(execution_deadline,now+10)
      if state=='OBSTACLE_WAIT':deadline=min(execution_deadline,now+20)
     continue
   if state=='WAIT':
    zero()
    if now>=next_wait_log:
     blockers=[]
     if not base_safe(now):blockers.append('SDK连接/控制权/急停/状态新鲜度')
     if not localization_safe(now):blockers.append('定位/地图身份/扫描/里程计')
     if tf_block:blockers.append('TF未就绪：'+tf_block)
     if not costmap_fresh(now):blockers.append('等待新鲜局部代价地图')
     if sdk.get('command_zero') is not True:blockers.append('SDK命令尚非零速')
     measured=[sdk.get(k) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
     if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.03 for v in measured):blockers.append('SDK实测速率未停稳，要求各轴绝对值<0.03，当前='+str(measured))
     if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False:blockers.append('等待Walk姿态确认')
     if not client.server_is_ready():blockers.append('执行动作服务未就绪')
     if approved and not route_costmap.service_is_ready():blockers.append('路径复核代价地图服务未就绪')
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
    if approved and not route_costmap.service_is_ready():continue
    if approved:
     if not auto_plan:
      bound=approved['plan']
      if (loc.get('map_id'),loc.get('session'),loc.get('epoch'))!=(bound['map_id'],bound['localization_session'],bound['localization_epoch']):fault('[STALE_PLAN] 规划后定位身份改变');continue
      q=pose.pose.orientation;current_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
      yaw_change=math.atan2(math.sin(current_yaw-bound['start']['yaw']),math.cos(current_yaw-bound['start']['yaw']))
      if math.hypot(pose.pose.position.x-bound['start']['x'],pose.pose.position.y-bound['start']['y'])>.15 or abs(yaw_change)>.25:fault('[STALE_PLAN] 准备期间起点或朝向变化，请重新规划');continue
     else:
      pos=current_pose()
      approved['poses']=[dict(x=pos[0],y=pos[1],yaw=pos[2]),dict(x=target['x'],y=target['y'],yaw=target['yaw'])]
     message=FollowPath.Goal();message.path=NavPath();message.path.header.frame_id='map';message.path.header.stamp=n.get_clock().now().to_msg()
     for v in approved['poses']:
      pnt=PoseStamped();pnt.header=message.path.header;pnt.pose.position.x=float(v['x']);pnt.pose.position.y=float(v['y']);pnt.pose.orientation.z=math.sin(v['yaw']/2);pnt.pose.orientation.w=math.cos(v['yaw']/2);message.path.poses.append(pnt)
     message.path.poses[-1].pose.orientation.z=math.sin(target['yaw']/2);message.path.poses[-1].pose.orientation.w=math.cos(target['yaw']/2)
     message.controller_id='FollowPath';message.goal_checker_id='goal_checker'
     if auto_plan:print('[PATH] 无预览文件，先按当前楼层地图生成并认证完整机身路线；保持零速。',flush=True)
     else:print('[PATH] 使用已确认的预览路线，共 %d 个点；终点朝向 %.4f rad' % (len(message.path.poses),target['yaw']),flush=True)
    pending_message=message
    execution_started=now; execution_deadline=now+args.max_runtime; next_countdown=now+5; deadline=execution_deadline
    print('[START] Walk 已稳定确认%s；最大运行时间 %.1fs，剩余 %.1fs；到时自动终止并停止速度发布。' % ('，未重复发送 standup' if walk_gate.request_before is None else '',args.max_runtime,args.max_runtime),flush=True)
    queue_route_check('prepare',force_plan=auto_plan)
   elif state=='ROUTE_CHECK':
    zero()
    valid=poll_route_check(now)
    if valid is None:continue
    if not valid:obstacle_hold('[PATH_FOOTPRINT_BLOCKED] '+json.dumps(route_details),now);continue
    deadline=execution_deadline
    if route_after=='prepare':prepare_route()
    else:dispatch_route()
   elif state=='OBSTACLE_WAIT':
    zero()
    if route_reply is not None and not route_reply.done():continue
    if goal is not None and result is not None:
     if not result.done():
      if now-obstacle_at>5.:fault('[CANCEL_NOT_CONFIRMED] 旧路径任务未确认结束')
      continue
     terminal=True
    if not stopped() or not costmap_fresh(now):stop_sequences.clear();continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2 or now-obstacle_at<.7:continue
    if background_ready is not None:
     values,certificate=background_ready;background_ready=None
     terminal=True;goal=None;result=None
     install_body_route(values,certificate)
     queue_route_check('prepare');continue
    if turn_wait is not None:
     if now<turn_wait.next_check:continue
     turn_wait.next_check=now+.25
     blocked=turn_clear(now)
     decision=turn_wait.update(now,bool(blocked),True)
     if decision=='WAIT':continue
     if decision=='RESUME':
      previous=turn_wait.phase
      alignment=begin_alignment(current_pose(),turn_wait.target,now,current_odom(),check_goal_position=previous=='ALIGN_YAW')
      turn_wait=None;obstacle_force_replan=False;replan_policy.since=None;motion_guard.reset()
      transition(previous,'[TURN_CLEARED] 障碍已消退，矩形转向检查连续通过，恢复转向')
      deadline=execution_deadline;align_halt_at=None;continue
     turn_wait=None;turn_replan_route=True
     yaw_priority_latched=False;terminal_started=None;approach_attempted=False
     print('[TURN_REPLAN] 转向持续受阻，重新搜索包含终点朝向的可执行路线；不会原地强转。',flush=True)
    if route_future is None and now>=route_next_check:request_route_check(now)
    checked=poll_route_check(now)
    if checked is None and route_future is not None:continue
    if route_last_valid is True and not obstacle_force_replan:
     replan_policy.since=None;terminal=True;goal=None;result=None
     pos=current_pose();points=pending_message.path.poses
     nearest=min(range(len(points)),key=lambda i:math.hypot(points[i].pose.position.x-pos[0],points[i].pose.position.y-pos[1]))
     pending_message.path.poses=points[nearest:]
     print('[OBSTACLE_CLEARED] 路线已重新通过机身检查，恢复前复核起步朝向。',flush=True)
     queue_route_check('prepare');continue
    if now-obstacle_at>=3. and replan_policy.ready(now):
     if not replan_policy.request(now):fault('[OBSTACLE_REPLAN_LIMIT] 本次受阻连续4次重规划仍未取得有效前进，已停车；此前已成功行走的阶段不累计');continue
     print('[OBSTACLE_REPLAN] 本次连续受阻第%d/4次重规划；恢复有效前进后清零。'%replan_policy.attempts,flush=True)
     pending_replan_charge=True
     terminal=True;reacquire('[DYNAMIC_REPLAN] '+json.dumps(route_details),now,category='data')
   elif state in ('NAVIGATE','ACCEPT'):
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False: fault('Walk mode lost'); continue
    pos=current_pose()
    if state=='NAVIGATE' and not turn_replan_route and math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=.25:
     zero()
     if goal and not (result and result.done()):goal.cancel_goal_async()
     terminal=bool(result and result.done());alignment=None;raw=None;raw_at=-math.inf
     transition('STOPPING');deadline=min(execution_deadline,now+10);halt_at=now
     print('[YAW_PRIORITY] 进入25cm终点闭环范围；停止平移并撤销DWB，等待动作结束和停稳后对齐朝向。',flush=True)
     continue
    if approved and state=='NAVIGATE' and not (result and result.done()):
     if route_future is None and now>=route_next_check:request_route_check(now)
     valid=poll_route_check(now)
     if valid is False:
      if route_details.get('clear_prefix_m',0.)>replan_radius:
       if replan_policy.since is None:replan_policy.since=now
      else:replan_policy.blocked(now)
      if route_details.get('clear_prefix_m',0.)<1.2:
       obstacle_hold('[PATH_FOOTPRINT_BLOCKED] '+json.dumps(route_details),now);continue
      observation_sec=5. if route_details.get('clear_prefix_m',0.)>replan_radius else 3.
      if background_future is None and background_ready is None and now>=background_retry_at and replan_policy.ready(now) and now-replan_policy.since>=observation_sec:
       footprint=json.loads(body['footprint'])
       background_future=body_executor.submit(plan_body,route_grid,current_pose(),[target['x'],target['y'],target['yaw']],
        max(abs(p[0]) for p in footprint)+padding,max(abs(p[1]) for p in footprint)+padding)
       background_retry_at=now+5.
       print('[DISTANT_OBSTACLE] 障碍持续存在，近段已通过机身复核，后台搜索备选路线。',flush=True)
     elif valid is True:
      replan_policy.cleared();background_ready=None
     if background_future is not None and background_future.done():
      try:background_ready=background_future.result()
      except ValueError as e:
       print('[BACKGROUND_PLAN_RETRY] '+str(e),flush=True);background_retry_at=now+5.
      background_future=None
      if route_last_valid is True:background_ready=None
     if valid is False and background_ready is not None and route_details.get('clear_prefix_m',0.)<=replan_radius:
       obstacle_hold('[BACKGROUND_ROUTE_READY] 已找到候选绕行路线，停车切换并按最新地图认证',now,force=True);continue
     if replan_policy.progress(now,math.hypot(pos[0]-target['x'],pos[1]-target['y']),valid=route_last_valid is True):
      print('[REPLAN_EPISODE_RESET] 连续恢复行走且向目标推进0.5m，清零本次受阻计数。',flush=True)
     if route_future is not None and now-route_requested>1.5:
      obstacle_hold('[PATH_CHECK_DELAY] 路径复核延迟，先停稳等待',now);continue
    if result and result.done():
     terminal=True
     if result.result().status!=GoalStatus.STATUS_SUCCEEDED:
      response=result.result();detail=response.result
      code=getattr(detail,'error_code',None)
      if approved and code in (104,105,106):
       obstacle_hold('[CONTROLLER_BLOCKED] error_code='+str(code),now,force=True);continue
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
    alignment.resume(now)
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
     if decision=='WAIT_SAMPLE':alignment.pause(now);zero();motion_guard.reset();continue
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
    if reason:
     turn_wait=TurnWait(state,alignment.target,now)
     obstacle_hold('[TURN_BLOCKED] '+reason,now,force=True)
     continue
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
     # Bounded 20cm stopping corridor; rectangle plus 2.5cm interpolation margin.
     for step in (0.,.05,.10,.15,.20):
      angle=wz*step/max(vx,.01)
      local_x=step if abs(wz)<1e-6 else vx/wz*math.sin(angle)
      local_y=0. if abs(wz)<1e-6 else vx/wz*(1-math.cos(angle))
      centre=(op[0]+local_x*math.cos(op[2])-local_y*math.sin(op[2]),op[1]+local_x*math.sin(op[2])+local_y*math.cos(op[2]))
      footprint=json.loads(body['footprint'])
      half_step_angle=wz*.025/max(vx,.01)
      blocked=swept_rotation_clearance(costmap,(*centre,op[2]+angle-half_step_angle),2*half_step_angle,
       max(abs(v[0]) for v in footprint)+padding+.025,max(abs(v[1]) for v in footprint)+padding+.025)
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
     recover_terminal('[FINAL_POSITION_ENVELOPE] 停稳复核超过35cm，进行有次数限制的重新接近',now);continue
    if action=='REPLAN':
     reacquire('[STOPPING_POSITION_SHIFT] 停稳且连续1秒新鲜定位确认位置超过0.20m，重新接近目标',now);continue
    if action=='ALIGN_YAW':
     distance=math.hypot(pos[0]-target['x'],pos[1]-target['y'])
     if distance>terminal_turn_position_limit(wrap(target['yaw']-pos[2])):
      recover_terminal('[TURN_POSITION_MARGIN] 转向前位置余量不足，先重规划接近目标',now);continue
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
    if not turn_replan_route and math.hypot(pos[0]-target['x'],pos[1]-target['y'])<=POSITION_TOLERANCE:
     alignment=begin_alignment(pos,[target['x'],target['y'],target['yaw']],now,current_odom())
     transition('ALIGN_YAW');deadline=execution_deadline;continue
    result=None;goal=None;terminal=False;deadline=execution_deadline;alignment=None
    print('[REPLAN] 已停稳，按当前地图搜索机身及转向均可通过的绕行路线。',flush=True)
    queue_route_check('prepare',force_plan=True)
  trace.emit('finished',state=state,reason=failed,recovery=budget.snapshot(time.monotonic()));trace.route_status('history')
  print(state+': '+(failed or 'goal reached and stopped'),flush=True)
 except KeyboardInterrupt: fault('operator interrupted')
 except Exception as e:
  fault(type(e).__name__+': '+str(e));print('FAULT: '+failed,flush=True)
 finally:
  if background_future is not None:background_future.cancel()
  if body_future is not None:body_future.cancel()
  body_executor.shutdown(wait=False,cancel_futures=True)
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
