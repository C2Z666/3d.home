"""Real Nav2 planner/controller/BT + synthetic lower platform and SDK state, no hardware sockets."""
import os,sys,time,json,math,shutil,subprocess,signal
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];W=Path(sys.argv[1]).resolve();R=W/'fixture';R.mkdir(parents=True,exist_ok=True)
for folder in ('scripts','config','maps'):
 shutil.copytree(ROOT/folder,R/folder,dirs_exist_ok=True)
site=yaml.safe_load((R/'config/site.yaml').read_text());site.update(maps_validated=True,scan_source_validated=True)
(R/'config/site.yaml').write_text(yaml.safe_dump(site))
# A synthetic known-free floor with occupied border. Never substitute this map into the deliverable maps.
image=np.full((240,240),254,np.uint8);image[[0,-1],:]=0;image[:,[0,-1]]=0
(R/'maps/lower.pgm').write_bytes(b'P5\n240 240\n255\n'+image.tobytes())
(R/'maps/lower.yaml').write_text(yaml.safe_dump(dict(image='lower.pgm',resolution=.05,origin=[-6.,-6.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
# Use the real visual bank through a link, while binding the synthetic occupancy map.
if not (R/'assets').exists():
 (R/'assets').symlink_to(ROOT/'assets',target_is_directory=True)
sys.path.insert(0,str(R/'scripts'))
from site_identity import hashes
identity=dict(map_id='synthetic_navigation',sha256=hashes(R))
(R/'config/site_identity.json').write_text(json.dumps(identity))
registry=json.loads((R/'config/scenes.json').read_text(encoding='utf-8'))
registry['scenes']['old']['map_id']=identity['map_id']
(R/'config/scenes.json').write_text(json.dumps(registry))
os.environ.update(ROS_DOMAIN_ID='186',ROS_LOCALHOST_ONLY='1',PYTHONDONTWRITEBYTECODE='1')
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from lifecycle_msgs.srv import GetState
rclpy.init();n=rclpy.create_node('phase3_nav2_simulator');tf=TransformBroadcaster(n)
map_tf=StaticTransformBroadcaster(n)
map_transform=TransformStamped()
map_transform.header.stamp=n.get_clock().now().to_msg()
map_transform.header.frame_id='map'
map_transform.child_frame_id='nav_odom'
map_transform.transform.rotation.w=1.
map_tf.sendTransform(map_transform)
pubs={k:n.create_publisher(t,topic,q) for k,t,topic,q in [
 ('sdk',String,'/d1/sdk/d1_motion_status',1),('loc',String,'/d1/localization/status',1),
 ('health',String,'/d1/nav/scan_status',1),('pose',PoseStamped,'/d1/localization/pose6d',10),
 ('odom',Odometry,'/d1/localization/odom',10),('scan',LaserScan,'/d1/nav/scan',qos_profile_sensor_data)]}
x=y=yaw=0.;command=Twist();cmd_at=-math.inf;raw_count=0;forwarded=[];all_forwarded=[];actions=[];ready=True
motion='Walk';seq=0;request=0;confirmed=0;sequence_at=0;last=time.monotonic();current_case='startup'
def cmd(m):
 global command,cmd_at
 command=m;cmd_at=time.monotonic();all_forwarded.append((current_case,cmd_at,m.linear.x,m.angular.z))
 if abs(m.linear.x)>1e-5 or abs(m.angular.z)>1e-5:forwarded.append((current_case,time.monotonic(),m.linear.x,m.angular.z))
def action(m):
 global motion,request,confirmed
 actions.append((current_case,m.data));request+=1;confirmed=request;motion='Walk'
def raw(m):
 global raw_count
 raw_count+=1
n.create_subscription(Twist,'/d1/sdk/supervised_cmd_vel',cmd,1)
n.create_subscription(String,'/d1/sdk/d1_action',action,10)
n.create_subscription(Twist,'/d1/nav/cmd_vel_raw',raw,1)
def step():
 global x,y,yaw,last,seq,sequence_at
 now=time.monotonic();dt=min(.05,now-last);last=now
 vx=command.linear.x if motion=='Walk' and now-cmd_at<.3 else 0.;wz=command.angular.z if motion=='Walk' and now-cmd_at<.3 else 0.
 x+=vx*math.cos(yaw)*dt;y+=vx*math.sin(yaw)*dt;yaw+=wz*dt
 stamp=n.get_clock().now().to_msg()
 pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.position.y=y
 pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);pubs['pose'].publish(pose)
 od=Odometry();od.header.stamp=stamp;od.header.frame_id='nav_odom';od.child_frame_id='nav_base';od.pose.pose=pose.pose;od.twist.twist.linear.x=vx;od.twist.twist.angular.z=wz;pubs['odom'].publish(od)
 t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='nav_odom';t.child_frame_id='nav_base';t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=pose.pose.orientation
 tf.sendTransform(t)
 # Publish scan acquisition time behind TF delivery, as with a buffered real sensor.
 scan=LaserScan();scan.header.stamp=(n.get_clock().now()-Duration(seconds=.1)).to_msg();scan.header.frame_id='nav_base';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.angle_max=math.pi-scan.angle_increment;scan.range_min=.3;scan.range_max=8.;scan.ranges=[math.inf]*720;pubs['scan'].publish(scan)
 pubs['loc'].publish(String(data=json.dumps(dict(map_id='synthetic_navigation',session='synthetic_localizer',epoch=0,ready=ready,tracking_valid=True))))
 pubs['health'].publish(String(data=json.dumps(dict(ready=True))))
 if now-sequence_at>.25:seq+=1;sequence_at=now
 pubs['sdk'].publish(String(data=json.dumps(dict(sdk_connected=True,control_held=True,state_fresh=True,state_age_sec=.01,
 soft_stop=False,hardware_stop=False,motion_pending=False,
 command_zero=abs(vx)+abs(wz)<1e-5,reported_forward_speed=vx,reported_lateral_speed=0.,reported_yaw_rate=wz,
 state_sequence=seq,bridge_session='synthetic_sdk',safety_epoch=0,motion=motion,pose_request_id=request,pose_confirmed_id=confirmed))))
 rclpy.spin_once(n,timeout_sec=.005);time.sleep(.015)
console=(W/'nav2.log').open('w');launch=subprocess.Popen(['ros2','launch',str(R/'scripts/nav2_bringup.launch.py')],stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
process=None;reports={}
try:
 clients={name:n.create_client(GetState,'/'+name+'/get_state') for name in ('map_server','planner_server','controller_server','bt_navigator')};active=set();pending={}
 end=time.monotonic()+35
 while time.monotonic()<end and len(active)<4:
  if launch.poll() is not None:raise AssertionError('launch exited: '+(W/'nav2.log').read_text()[-6000:])
  step()
  for name,client in clients.items():
   if name in active:continue
   if name in pending:
    if pending[name].done():
     if pending[name].result().current_state.id==3:active.add(name)
     del pending[name]
   elif client.service_is_ready():pending[name]=client.call_async(GetState.Request())
 if len(active)<4:raise AssertionError('Nav2 not active '+repr(active)+' '+(W/'nav2.log').read_text()[-8000:])
 reports['nav2_lifecycle_active']=sorted(active)
 for case in ('goal_reached','localization_loss','runtime_limit'):
  current_case=case;x=y=yaw=0.;motion='Walk';ready=True;command=Twist();request=confirmed=0
  for _ in range(30):step()
  args=['--live','--x',('1.5' if case=='runtime_limit' else '.55'),'--y','0','--yaw','0']
  logfile=(W/(case+'.log')).open('w');process=subprocess.Popen([sys.executable,str(R/'scripts/navigate_to_goal.py')]+args,stdin=subprocess.PIPE,stdout=logfile,stderr=subprocess.STDOUT,text=True)
  start=time.monotonic();loss_at=None;timeout_seen=None
  while time.monotonic()-start<40 and process.poll() is None:
   step()
   if case=='runtime_limit' and timeout_seen is None and 'MAX_RUNTIME_EXCEEDED' in (W/(case+'.log')).read_text():timeout_seen=time.monotonic()
   if case=='localization_loss' and x>.05 and loss_at is None:ready=False;loss_at=time.monotonic()
  if process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=5);raise AssertionError(case+' timed out')
  for _ in range(20):step()
  logfile.close();log=(W/(case+'.log')).read_text();casecmd=[v for v in forwarded if v[0]==case]
  assert not [v for k,v in actions if k==case], 'Already Walk must not request StandUp'
  if case=='runtime_limit':
   assert process.returncode!=0 and timeout_seen is not None,log
   assert '[START]' in log and '[COUNTDOWN]' in log and 'MAX_RUNTIME_EXCEEDED' in log,log
   assert casecmd,'Timeout case never executed a velocity'
   assert not [v for v in all_forwarded if v[0]==case and v[1]>timeout_seen+.15], 'Twist continued after runtime expiration'
  elif case=='localization_loss':
   assert process.returncode!=0 and loss_at is not None,log
   assert not any(v[1]>loss_at+.5 for v in casecmd),casecmd
   assert all(value=='standup' for key,value in actions if key==case),actions
  else:
   assert process.returncode==0,(log,x,y,raw_count,len(casecmd),actions)
   assert abs(x-.55)<.13 and abs(y)<.13,(x,y,log)
   assert bool(casecmd),'Nav2 never forwarded a moving command'
   assert 'DONE: goal reached and stopped' in log,log
  reports[case]=dict(result='PASS',exit_code=process.returncode,final_pose=[x,y,yaw],forwarded_commands=len(casecmd))
  process=None
 reports.update(raw_nav2_commands=raw_count,hardware_connected=False,synthetic_floor_only=True)
 (W/'report.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
finally:
 if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
 os.killpg(launch.pid,signal.SIGINT)
 try:launch.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
 console.close();n.destroy_node();rclpy.try_shutdown()
