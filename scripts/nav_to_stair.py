#!/usr/bin/env python3
"""Navigate to the measured lower entry, stop, select Stair, then wait for y."""
import argparse,json,math,signal,subprocess,sys,time
from pathlib import Path
import yaml


def main():
 root=Path(__file__).resolve().parents[1]; cfg=yaml.safe_load((root/'config/site.yaml').read_text())
 p=argparse.ArgumentParser(description=__doc__); p.add_argument('--live',action='store_true',required=True)
 p.add_argument('--duration',type=float)
 p.add_argument('--approach-only',action='store_true')
 args=p.parse_args(); entry=cfg.get('entry'); stair=cfg['stair']
 from site_identity import verify_entry
 map_id=verify_entry(root,cfg)
 if not cfg.get('maps_validated') or not cfg.get('scan_source_validated') or not entry:
  p.error('validate site.yaml and capture the actual lower entry before live navigation')
 if cfg.get('initial_floor')!='lower': p.error('this mission starts on lower')
 if not args.approach_only and (args.duration is None or not math.isfinite(args.duration) or not 0<args.duration<=180): p.error('--duration must be in (0,180] seconds')
 if entry.get('frame')!='map': p.error('entry must use map frame')
 if not all(isinstance(entry.get(k),(int,float)) and math.isfinite(entry[k]) for k in ('x','y','z','yaw')): p.error('entry must have finite measured x/y/z/yaw')
 import rclpy
 from rclpy.node import Node
 from rclpy.action import ActionClient
 from geometry_msgs.msg import PoseStamped,Twist
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import LaserScan
 from nav2_msgs.action import NavigateToPose
 from std_msgs.msg import Bool,String
 from action_msgs.msg import GoalStatus
 from rclpy.signals import SignalHandlerOptions
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n=Node('d1_nav_stair_supervisor')
 interrupted=False
 def on_signal(*_):
  nonlocal interrupted
  interrupted=True
 signal.signal(signal.SIGINT,on_signal); signal.signal(signal.SIGTERM,on_signal)
 vel=n.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
 stairvel=n.create_publisher(Twist,'/d1/sdk/stair_cmd_vel',1)
 permit=n.create_publisher(Bool,'/d1/sdk/stair_motion_permit',1)
 action=n.create_publisher(String,'/d1/sdk/d1_action',1)
 client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
 sdk={}; loc={}; pose=None; raw=None; odom=None
 sdk_at=loc_at=pose_at=raw_at=odom_at=scan_at=-math.inf; nonzero_at=-math.inf
 sdk_identity=None; loc_identity=None; mode_request=None
 state='WAIT'; deadline=time.monotonic()+60; goal=None; result=None; terminal=False; failed=''; stop_sequences=set(); halt_at=None
 scanhealth={}; health_at=-math.inf
 def zero():
  vel.publish(Twist()); stairvel.publish(Twist()); permit.publish(Bool(data=False))
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
  nonlocal raw,raw_at,nonzero_at
  raw=m; raw_at=time.monotonic()
  values=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
  if any(not math.isfinite(v) or abs(v)>1e-4 for v in values): nonzero_at=raw_at
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
   else: state='NAVIGATE'
  except Exception as e: fault('Nav2 goal request failed: '+str(e))
 def base_safe(now):
  return (now-sdk_at<.4 and all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh'))
   and all(sdk.get(k) is False for k in ('soft_stop','hardware_stop','flat_test_lock','nav_handoff_hold'))
   and isinstance(sdk.get('state_age_sec'),(int,float)) and 0<=sdk['state_age_sec']<2)
 def localization_safe(now):
  if loc.get('map_id')!=map_id: return False
  if not (now-loc_at<.4 and loc.get('ready') is True and now-pose_at<.4 and now-scan_at<.5 and now-odom_at<.4 and now-health_at<.5 and scanhealth.get('ready') is True): return False
  if pose is None: return False
  q=pose.pose.orientation
  tilt=2*math.acos(min(1.,math.sqrt(q.w*q.w+q.z*q.z)))
  return tilt<math.radians(10) and abs(pose.pose.position.z-entry['z'])<.3
 def at_entry():
  q=pose.pose.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  angle=math.atan2(math.sin(yaw-entry['yaw']),math.cos(yaw-entry['yaw']))
  return math.hypot(pose.pose.position.x-entry['x'],pose.pose.position.y-entry['y'])<.13 and abs(angle)<.13
 print('准备导航至实测入口；到点、停稳、Stair 确认后等待 y。',flush=True)
 try:
  while rclpy.ok() and state not in ('FAULT','HANDOFF'):
   rclpy.spin_once(n,timeout_sec=.03); now=time.monotonic()
   if interrupted: fault('operator interrupted'); continue
   if now>deadline: fault('mission deadline exceeded'); continue
   if state!='STAIR':
    if sdk_identity is not None and (sdk.get('bridge_session'),sdk.get('safety_epoch'))!=sdk_identity: fault('SDK session/safety changed'); continue
    if loc_identity is not None and (loc.get('session'),loc.get('epoch'))!=loc_identity: fault('localization session/epoch changed; start a new task'); continue
    if state not in ('WAIT',) and (not base_safe(now) or not localization_safe(now)): fault('navigation SDK/localization/scan check failed'); continue
   if state=='WAIT':
    zero()
    if not base_safe(now) or not localization_safe(now) or sdk.get('command_zero') is not True: stop_sequences.clear(); continue
    stopped_values=[sdk.get(k,math.inf) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<.03 for v in stopped_values): stop_sequences.clear(); continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2: continue
    sdk_identity=(sdk.get('bridge_session'),sdk.get('safety_epoch')); loc_identity=(loc.get('session'),loc.get('epoch'))
    if mode_request is None:
     mode_request=sdk.get('pose_request_id',-1); action.publish(String(data='standup')); continue
    if (sdk.get('pose_request_id',-1)<=mode_request or sdk.get('pose_confirmed_id')!=sdk.get('pose_request_id')
      or sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False): continue
    if not client.server_is_ready(): continue
    message=NavigateToPose.Goal(); message.pose=PoseStamped(); message.pose.header.frame_id='map'; message.pose.header.stamp=n.get_clock().now().to_msg()
    message.pose.pose.position.x=entry['x']; message.pose.pose.position.y=entry['y']
    message.pose.pose.orientation.z=math.sin(entry['yaw']/2); message.pose.pose.orientation.w=math.cos(entry['yaw']/2)
    message.behavior_tree=str(root/'config/navigate.xml'); state='ACCEPT'; deadline=now+180
    client.send_goal_async(message).add_done_callback(accepted)
   elif state in ('NAVIGATE','ACCEPT'):
    if sdk.get('motion')!='Walk' or sdk.get('motion_pending') is not False: fault('Walk mode lost'); continue
    if result and result.done():
     terminal=True
     if result.result().status!=GoalStatus.STATUS_SUCCEEDED: fault('Nav2 did not succeed'); continue
     state='STOPPING'; deadline=now+10; halt_at=now; stop_sequences.clear(); zero(); continue
    if state=='NAVIGATE' and raw is not None and now-raw_at<.15:
     values=(raw.linear.x,raw.linear.y,raw.linear.z,raw.angular.x,raw.angular.y,raw.angular.z)
     if not all(math.isfinite(v) for v in values) or not 0<=raw.linear.x<=.15 or abs(raw.angular.z)>.25 or any(abs(v)>1e-6 for v in values[1:5]):
      fault('unsupported Nav2 velocity'); continue
     vel.publish(raw)
    else: zero()
   elif state=='STOPPING':
    zero()
    # Terminal result is required; cancel acknowledgement alone is not sufficient.
    if not terminal or now-halt_at<.7 or not at_entry() or now-odom_at>.4: continue
    t=odom.twist.twist
    if not all(math.isfinite(v) and abs(v)<.03 for v in (t.linear.x,t.linear.y,t.angular.z)): stop_sequences.clear(); continue
    if not all(isinstance(sdk.get(k),(int,float)) and math.isfinite(sdk[k]) and abs(sdk[k])<.03 for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')) or sdk.get('command_zero') is not True: stop_sequences.clear(); continue
    stop_sequences.add(sdk.get('state_sequence'))
    if len(stop_sequences)<2: continue
    state='HANDOFF'
  print(state+': '+(failed or 'entry reached and stopped'),flush=True)
 except KeyboardInterrupt: fault('operator interrupted')
 except Exception as e: fault(type(e).__name__+': '+str(e))
 finally:
  if goal and not terminal: goal.cancel_goal_async()
  shutdown=time.monotonic()+1
  while rclpy.ok() and time.monotonic()<shutdown:
   zero(); rclpy.spin_once(n,timeout_sec=.03)
  n.destroy_node()
  if rclpy.ok(): rclpy.shutdown()
 if interrupted: return 130
 if state!='HANDOFF': return 1
 if args.approach_only:
  print('DONE: 已停在预备点，本次不切换 Stair。',flush=True); return 0
 print('HANDOFF: 停止 Nav2 速度转发，调用实测 stair_start.py；确认后输入 y 和速度，例如 y 0.5。',flush=True)
 signal.signal(signal.SIGINT,signal.default_int_handler); signal.signal(signal.SIGTERM,signal.SIG_DFL)
 child=subprocess.Popen([sys.executable,str(root/'scripts/stair_start.py'),'--duration',str(args.duration)])
 try: return child.wait()
 except KeyboardInterrupt:
  if child.poll() is None: child.send_signal(signal.SIGINT)
  try: child.wait(timeout=5)
  except subprocess.TimeoutExpired: child.terminate(); child.wait(timeout=5)
  return 130
if __name__=='__main__': sys.exit(main())
