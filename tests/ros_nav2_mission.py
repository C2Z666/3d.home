"""Real Nav2 planner/controller/BT + synthetic lower platform and SDK state, no hardware sockets."""
import os,sys,time,json,math,shutil,subprocess,signal,copy
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];W=Path(sys.argv[1]).resolve();R=W/'fixture';R.mkdir(parents=True,exist_ok=True)
for folder in ('scripts','config','maps'):
 shutil.copytree(ROOT/folder,R/folder,dirs_exist_ok=True)
site=yaml.safe_load((R/'config/site.yaml').read_text());site.update(maps_validated=True,scan_source_validated=True)
(R/'config/site.yaml').write_text(yaml.safe_dump(site))
if os.environ.get('D1_VOXEL_TEST')=='1':
 cfg=yaml.safe_load((R/'config/nav2.yaml').read_text())
 for key in ('global_costmap','local_costmap'):
  layer=cfg[key][key]['ros__parameters']['obstacle_layer'];layer['plugin']='nav2_costmap_2d::VoxelLayer';layer['obstacles']['observation_persistence']=0.0
 (R/'config/nav2.yaml').write_text(yaml.safe_dump(cfg))
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
os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from scipy.spatial.transform import Rotation
from geometry import imu_from_base,LIO_TO_NAV
I_B=imu_from_base(R/'config/calibration.yaml');basis=np.eye(4);basis[:3,:3]=LIO_TO_NAV
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap,GetCostmap
rclpy.init();n=rclpy.create_node('phase3_nav2_simulator');tf=TransformBroadcaster(n)
map_tf=TransformBroadcaster(n)
map_transform=TransformStamped()
map_transform.header.stamp=n.get_clock().now().to_msg()
map_transform.header.frame_id='map'
map_transform.child_frame_id='nav_odom'
map_transform.transform.rotation.w=1.
map_tf.sendTransform(map_transform)
pubs={k:n.create_publisher(t,topic,q) for k,t,topic,q in [
 ('sdk',String,'/d1/sdk/d1_motion_status',1),('loc',String,'/d1/localization/status',1),
 ('health',String,'/d1/nav/scan_status',1),('pose',PoseStamped,'/d1/localization/pose6d',10),
 ('rawlio',Odometry,'/Odometry',qos_profile_sensor_data),('marks',PointCloud2,'/d1/nav/obstacle_points',qos_profile_sensor_data),('clears',PointCloud2,'/d1/nav/clearing_points',qos_profile_sensor_data),
 ('odom',Odometry,'/d1/localization/odom',10),('scan',LaserScan,'/d1/nav/scan',qos_profile_sensor_data)]}
x=y=yaw=0.;case_start=time.monotonic();actual_vx=actual_wz=0.;trajectory=[];command=Twist();cmd_at=-math.inf;raw_count=0;forwarded=[];all_forwarded=[];actions=[];ready=True
map_dx=0.;shift_enabled=False;drift_enabled=False
tf_available=True;obstacle=False;motion='Walk';seq=0;request=0;confirmed=0;sequence_at=0;last=time.monotonic();current_case='startup';voxel_stage='mark';temporal_stage='present';last_sensor_pub=-math.inf
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
 global x,y,yaw,last,seq,sequence_at,actual_vx,actual_wz,map_dx,last_sensor_pub
 now=time.monotonic();dt=min(.05,now-last);last=now
 vx=command.linear.x if motion=='Walk' and now-cmd_at<.3 else 0.;wz=command.angular.z if motion=='Walk' and now-cmd_at<.3 else 0.
 # Model the actual bridge's command ramp and an ASSUMED plant deadband.
 # The 0.08 threshold is a regression fixture, not a measured D1 specification.
 actual_vx+=max(-.5*dt,min(.5*dt,vx-actual_vx))
 actual_wz+=max(-1.*dt,min(1.*dt,wz-actual_wz))
 vx=actual_vx;wz=0. if abs(actual_wz)<.08 else actual_wz
 if os.environ.get('D1_TEST_WEAK_PLANT')=='1' and current_case=='approved_path':vx=max(0.,vx-.07)*.6
 if current_case=='slow_turn':wz*=.5
 if current_case=='turn_stall':wz=0.
 if current_case=='motion_retry' and not (W/'motion_retry.log').exists():vx=0.
 elif current_case=='motion_retry' and '[INPUT_PAUSE]' not in (W/'motion_retry.log').read_text():vx=0.
 if drift_enabled and abs(wz)>.1:vx=.035
 if current_case=='unexpected_reverse' and abs(wz)>.1:vx=-.10
 x+=vx*math.cos(yaw)*dt;y+=vx*math.sin(yaw)*dt;yaw+=wz*dt
 if shift_enabled:map_dx=max(-.18,map_dx-.03*dt)
 trajectory.append((current_case,x,y,yaw))
 stamp=n.get_clock().now().to_msg()
 map_transform.header.stamp=stamp;map_transform.transform.translation.x=map_dx
 if tf_available:map_tf.sendTransform(map_transform)
 pose=PoseStamped();pose.header.stamp=stamp;pose.header.frame_id='map';pose.pose.position.x=x+map_dx;pose.pose.position.y=y
 pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
 if current_case=='map_z_drift':pose.pose.position.z=-.6
 pubs['pose'].publish(pose)
 body=np.eye(4);body[:3,:3]=Rotation.from_euler('xyz',[math.radians(12) if current_case=='flat_tilt_diagnostic' and x>.05 else 0.,0.,yaw]).as_matrix();body[:3,3]=[x,y,0.]
 raw=np.linalg.inv(basis)@body@np.linalg.inv(I_B);q=Rotation.from_matrix(raw[:3,:3]).as_quat()
 lio=Odometry();lio.header.stamp=stamp;lio.header.frame_id='d1_lio_init';lio.child_frame_id='d1_imu'
 lio.pose.pose.position.x,lio.pose.pose.position.y,lio.pose.pose.position.z=map(float,raw[:3,3]);lio.pose.pose.orientation.x,lio.pose.pose.orientation.y,lio.pose.pose.orientation.z,lio.pose.pose.orientation.w=map(float,q);pubs['rawlio'].publish(lio)
 od=Odometry();od.header.stamp=stamp;od.header.frame_id='nav_odom';od.child_frame_id='nav_base';od.pose.pose=copy.deepcopy(pose.pose);od.pose.pose.position.x=x;od.twist.twist.linear.x=vx;od.twist.twist.angular.z=wz;pubs['odom'].publish(od)
 t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='nav_odom';t.child_frame_id='nav_base';t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=pose.pose.orientation
 if tf_available:tf.sendTransform(t)
 # Match the real scanner rate; do not flood a ten-second history at 60Hz.
 if now-last_sensor_pub>=.1:
  last_sensor_pub=now
  # Publish scan acquisition time behind TF delivery, as with a buffered real sensor.
  scan=LaserScan();scan.header.stamp=(n.get_clock().now()-Duration(seconds=.1)).to_msg();scan.header.frame_id='nav_base';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.angle_max=math.pi-scan.angle_increment;scan.range_min=.3;scan.range_max=8.;scan.ranges=[(.75 if current_case=='turn_inflation' else .5) if obstacle else math.inf]*720;pubs['scan'].publish(scan)
  origin=TransformStamped();origin.header.stamp=stamp;origin.header.frame_id='nav_base';origin.child_frame_id='d1_nav_lidar_origin';origin.transform.rotation.w=1.;tf.sendTransform(origin)
  def cloud(points):
   m=PointCloud2();m.header=copy.deepcopy(scan.header);m.height=1;m.width=len(points);m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))];m.point_step=12;m.row_step=12*len(points);m.is_dense=True;m.data=np.asarray(points,dtype='<f4').reshape(-1,3).tobytes();return m
  points=[]
  if obstacle:
   distance=.75 if current_case=='turn_inflation' else .5
   points=[[distance*math.cos(a),distance*math.sin(a),0.] for a in np.linspace(-math.pi,math.pi,720)]
  # During the dynamic test, a temporary object blocks the forward route.
  if current_case=='dynamic_clear' and 1.<time.monotonic()-case_start<7.:
   dx=1.2-x;dy=-y;points=[[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,0.]]
  if current_case not in ('voxel_evidence','temporal_evidence'):pubs['marks'].publish(cloud(points))
  rays=[] if points else [[3.5*math.cos(a),3.5*math.sin(a),0.] for a in np.linspace(-math.pi,math.pi,720)]
  if current_case=='voxel_evidence':
   points=[[1.2,0.,0.]] if voxel_stage=='mark' else []
   pubs['marks'].publish(cloud(points))
   rays=[] if voxel_stage=='mark' else [[3.5,0.,-1.]] if voxel_stage=='floor_only' else [[.7,0.,0.]] if voxel_stage=='occluded' else [[3.5,0.,0.]]
  if current_case=='temporal_evidence':
   if temporal_stage!='dropout':pubs['marks'].publish(cloud([[1.2,0.,.5]] if temporal_stage=='present' else []))
   rays=[]
  pubs['clears'].publish(cloud(rays))
 pubs['loc'].publish(String(data=json.dumps(dict(map_id='synthetic_navigation',session='synthetic_localizer',epoch=0,ready=ready,tracking_valid=True,tracking_fault='',mode='FUSED',source_stamp=stamp.sec+stamp.nanosec*1e-9))))
 pubs['health'].publish(String(data=json.dumps(dict(ready=True))))
 if now-sequence_at>.25:seq+=1;sequence_at=now
 pubs['sdk'].publish(String(data=json.dumps(dict(sdk_connected=True,control_held=True,state_fresh=True,state_age_sec=.01,
 soft_stop=False,hardware_stop=False,motion_pending=False,
 command_zero=abs(vx)+abs(wz)<1e-5,reported_forward_speed=vx,reported_lateral_speed=0.,reported_yaw_rate=wz,
 state_sequence=seq,bridge_session='synthetic_sdk',safety_epoch=0,motion=motion,pose_request_id=request,pose_confirmed_id=confirmed))))
 # Drain TF and control callbacks; one spin per publish step accumulates stale TF.
 for _ in range(8):rclpy.spin_once(n,timeout_sec=0.)
 time.sleep(.015)
console=(W/'nav2.log').open('w');launch=subprocess.Popen(['ros2','launch',str(R/'scripts/nav2_bringup.launch.py')],stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
process=None;reports={}
try:
 clients={name:n.create_client(GetState,'/'+name+'/get_state') for name in ('map_server','planner_server','controller_server','velocity_smoother','bt_navigator')};active=set();pending={};pending_at={}
 end=time.monotonic()+35
 while time.monotonic()<end and len(active)<5:
  if launch.poll() is not None:raise AssertionError('launch exited: '+(W/'nav2.log').read_text()[-6000:])
  step()
  for name,client in clients.items():
   if name in active:continue
   if name in pending:
    if pending[name].done():
     if pending[name].result().current_state.id==3:active.add(name)
     del pending[name]
    elif time.monotonic()-pending_at[name]>2.:
     client.remove_pending_request(pending[name]);del pending[name]
   elif client.service_is_ready():
    pending[name]=client.call_async(GetState.Request());pending_at[name]=time.monotonic()
 if len(active)<5:raise AssertionError('Nav2 not active '+repr(active)+' '+(W/'nav2.log').read_text()[-8000:])
 reports['nav2_lifecycle_active']=sorted(active)
 if os.environ.get('D1_TEMPORAL_TEST')=='1':
  current_case='temporal_evidence';cost=n.create_client(GetCostmap,'/global_costmap/get_costmap');assert cost.wait_for_service(timeout_sec=3)
  evidence={}
  def check_temporal(label,expected,delay):
   until=time.monotonic()+delay
   while time.monotonic()<until:step()
   f=cost.call_async(GetCostmap.Request());until=time.monotonic()+3
   while not f.done() and time.monotonic()<until:step()
   assert f.done();g=f.result().map;d=np.array(g.data).reshape(g.metadata.size_y,g.metadata.size_x)
   col=int((1.2-g.metadata.origin.position.x)/g.metadata.resolution);row=int((0.-g.metadata.origin.position.y)/g.metadata.resolution)
   value=int(d[row-1:row+2,col-1:col+2].max());inflation=int(d[row,col+3]);evidence[label]=dict(obstacle=value,inflation=inflation)
   assert value==expected,(label,value,expected)
   assert (inflation>0)==(expected==254),(label,'inflation',inflation)
   assert int(d[0,0])==254,'Static wall was erased'
   assert int(d[row,col+7])==0,'Dynamic inflation extends past 0.25m'
   assert int(d[120,7])>0,'Static wall lost 0.4m inflation'
  temporal_stage='present';check_temporal('present_for_12s',254,12.)
  temporal_stage='absent';check_temporal('absent_for_0_5s',254,.5)
  check_temporal('absent_for_3s',0,2.5)
  temporal_stage='present';check_temporal('reappeared',254,1.)
  temporal_stage='dropout';check_temporal('dropout_12s_retains',254,12.)
  temporal_stage='absent';check_temporal('recovered_fresh_empty',0,1.5)
  reports['temporal_evidence']=evidence;n.destroy_client(cost);current_case='startup'
  (W/'report.json').write_text(json.dumps(reports,indent=2));print('TEMPORAL_PASS',json.dumps(evidence),flush=True)
 if os.environ.get('D1_VOXEL_TEST')=='1':
  current_case='voxel_evidence';cost=n.create_client(GetCostmap,'/global_costmap/get_costmap');assert cost.wait_for_service(timeout_sec=3)
  results={}
  for voxel_stage in ('mark','floor_only','occluded','clear'):
   end=time.monotonic()+2.5
   while time.monotonic()<end:step()
   f=cost.call_async(GetCostmap.Request());end=time.monotonic()+3
   while not f.done() and time.monotonic()<end:step()
   assert f.done();g=f.result().map;d=np.array(g.data).reshape(g.metadata.size_y,g.metadata.size_x)
   col=int((1.2-g.metadata.origin.position.x)/g.metadata.resolution);row=int((0.-g.metadata.origin.position.y)/g.metadata.resolution)
   value=int(d[row-1:row+2,col-1:col+2].max());results[voxel_stage]=value
   assert value==(0 if voxel_stage=='clear' else 254),(voxel_stage,value)
   assert int(d[0,0])==254, 'Static wall must remain'
  reports['voxel_evidence']=results;n.destroy_client(cost);current_case='startup'
 for case in ('goal_reached','localization_loss','runtime_limit','approved_path','approved_turn','behind_path','curved_path','slow_turn','turn_stall','unexpected_reverse','operator_cancel','tf_loss','turn_obstacle','visual_shift_turn','terminal_shift','tf_transient','tf_transient_turn','turn_inflation','turn_drift','turn_drift_path','motion_retry','data_retries','map_z_drift','flat_tilt_diagnostic','dynamic_clear'):
  if os.environ.get('D1_TEST_CASES') and case not in os.environ['D1_TEST_CASES'].split(','):continue
  current_case=case;case_start=time.monotonic()+1000;tf_available=True;obstacle=False;x=y=yaw=0.;actual_vx=actual_wz=0.;motion='Walk';ready=True;command=Twist();request=confirmed=0;map_dx=0.;shift_enabled=False;drift_enabled=False
  # The live layer now retains observation clouds for 10s. Clearing grid
  # cells alone cannot remove a previous case's still-valid observations.
  if any(isinstance(v,dict) and v.get('result')=='PASS' for v in reports.values()):
   until=time.monotonic()+10.5
   while time.monotonic()<until:step()
  for _ in range(30):step()
  # Independent synthetic cases must not inherit marked obstacles from the
  # previous case (infinite laser ranges may not clear obstacle-layer cells).
  # These services exist only in this isolated test ROS domain.
  for name in ('local','global'):
   c=n.create_client(ClearEntireCostmap,'/'+name+'_costmap/clear_entirely_'+name+'_costmap')
   assert c.wait_for_service(timeout_sec=3)
   f=c.call_async(ClearEntireCostmap.Request());until=time.monotonic()+3
   while not f.done() and time.monotonic()<until:step()
   assert f.done();n.destroy_client(c)
  for _ in range(30):step()
  drift_enabled=case in ('turn_drift','turn_drift_path')
  target_x=2.0 if case=='dynamic_clear' else 3.0 if case=='data_retries' else 3. if case=='runtime_limit' else .02 if case in ('approved_turn','slow_turn','turn_stall','unexpected_reverse','turn_obstacle','tf_transient_turn','turn_inflation','turn_drift') else -.5 if case in ('behind_path','visual_shift_turn','turn_drift_path') else .65 if case=='curved_path' else .55
  target_y=.04 if os.environ.get('D1_TEST_WEAK_PLANT')=='1' and case=='approved_path' else .1 if case=='behind_path' else .6 if case=='curved_path' else 0.
  target_yaw=2.8 if case=='tf_transient_turn' else 2. if case=='terminal_shift' else math.pi if case in ('slow_turn','visual_shift_turn','turn_drift','turn_drift_path') else 1. if case in ('approved_turn','turn_stall','unexpected_reverse','behind_path','turn_obstacle','turn_inflation') else math.pi/2 if case=='curved_path' else 0.
  runtime=40. if case in ('map_z_drift','dynamic_clear','flat_tilt_diagnostic') else 40. if os.environ.get('D1_TEST_WEAK_PLANT')=='1' and case=='approved_path' else 60. if case=='data_retries' else 60. if case in ('visual_shift_turn','terminal_shift','tf_transient_turn','turn_drift','turn_drift_path') else 30. if case in ('behind_path','curved_path','slow_turn','tf_transient','motion_retry') else 20. if case in ('approved_path','approved_turn') else 10.
  args=['--live','--x',str(target_x),'--y',str(target_y),'--yaw',str(target_yaw),'--max-runtime',str(runtime)]
  route=R/'work/approved_test.path.json';route.parent.mkdir(exist_ok=True)
  if case=='curved_path':
   vertices=[(0.,0.),(.4,0.),(.65,.25),(.65,.6)]
  else:vertices=[(0.,0.),(target_x,target_y)]
  points=[]
  for a,b in zip(vertices,vertices[1:]):
   for t in np.linspace(0.,1.,max(2,int(math.hypot(b[0]-a[0],b[1]-a[1])/.025))):
    points.append(dict(x=float(a[0]+t*(b[0]-a[0])),y=float(a[1]+t*(b[1]-a[1])),yaw=math.atan2(b[1]-a[1],b[0]-a[0])))
  route.write_text(json.dumps(dict(plan=dict(map_id='synthetic_navigation',localization_session='synthetic_localizer',localization_epoch=0,start=dict(x=0.,y=0.,yaw=0.)),poses=points)))
  # Post-stair navigation has no saved preview: exercise that actual entry point.
  if case!='goal_reached':args+=['--path-file',str(route)]
  logfile=(W/(case+'.log')).open('w');process=subprocess.Popen([sys.executable,str(R/'scripts/navigate_to_goal.py')]+args,stdin=subprocess.PIPE,stdout=logfile,stderr=subprocess.STDOUT,text=True)
  start=time.monotonic();case_start=start;loss_at=None;timeout_seen=None;cancel_at=None
  while time.monotonic()-start<runtime+10 and process.poll() is None:
   step()
   if case=='visual_shift_turn' and '[ALIGN_PATH]' in (W/(case+'.log')).read_text():shift_enabled=True
   if case=='terminal_shift' and '[ALIGN_YAW]' in (W/(case+'.log')).read_text():shift_enabled=True
   if case in ('turn_drift','turn_drift_path') and '[REACQUIRE_STOP]' in (W/(case+'.log')).read_text():drift_enabled=False
   if case=='runtime_limit' and timeout_seen is None and 'MAX_RUNTIME_EXCEEDED' in (W/(case+'.log')).read_text():timeout_seen=time.monotonic()
   if case=='data_retries':
    elapsed=time.monotonic()-start
    tf_available=not any(t<elapsed<t+.8 for t in (8.,12.,16.,20.))
   if case=='tf_loss' and x>.05:tf_available=False
   if case in ('tf_transient','tf_transient_turn'):
    trigger=abs(yaw)>.3 if case=='tf_transient_turn' else x>.05
    if trigger and loss_at is None:loss_at=time.monotonic()
    if loss_at is not None:tf_available=time.monotonic()-loss_at>(13. if case=='tf_transient_turn' else .8)
   if case in ('turn_obstacle','turn_inflation') and '[ALIGN_YAW]' in (W/(case+'.log')).read_text():obstacle=True
   if case=='operator_cancel' and x>.05 and cancel_at is None:cancel_at=time.monotonic();process.send_signal(signal.SIGINT)
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
  elif case in ('turn_stall','unexpected_reverse'):
   marker='NO_TURN_RESPONSE' if case=='turn_stall' else 'UNEXPECTED_REVERSE'
   assert process.returncode!=0 and marker in log,log
   assert time.monotonic()-start<9.,log
  elif case in ('tf_loss','turn_obstacle'):
   marker='INPUT_PAUSE' if case=='tf_loss' else 'TURN_BLOCKED'
   assert process.returncode!=0 and marker in log,log
  elif case=='operator_cancel':
   assert process.returncode==130 and cancel_at is not None,log
   assert not any(v[1]>cancel_at+.3 for v in casecmd),casecmd
  else:
   assert process.returncode==0,(log,x,y,raw_count,len(casecmd),actions)
   assert math.hypot(x+map_dx-target_x,y-target_y)<(.35 if case in ('terminal_shift','goal_reached') else .20) and abs(math.atan2(math.sin(yaw-target_yaw),math.cos(yaw-target_yaw)))<.13,(x,y,yaw,log)
   assert bool(casecmd),'Nav2 never forwarded a moving command'
   assert 'DONE: goal reached and stopped' in log,log
   if case=='approved_turn':assert '[ALIGN_PATH]' not in log,log
   if case=='goal_reached':
    assert '[BODY_REPLAN]' in log and '[REPLAN_READY]' in log,log
    assert log.index('[REPLAN_READY]')<log.index('[TRACK_PATH]'),log
   if case=='approved_path':assert '[PATH_RELEASED]' in log,log
   if case=='dynamic_clear':assert '[OBSTACLE_WAIT]' in log and ('[REPLAN_READY]' in log or '[OBSTACLE_CLEARED]' in log),log
   if case=='visual_shift_turn':assert abs(map_dx)>.12 and '[ALIGN_METRICS]' in log,log
   if case=='data_retries':
    assert log.count('[INPUT_RECOVERED]')>=4,log
    assert '[POSITION_RECOVERY_LIMIT]' not in log and '[MOTION_RECOVERY_LIMIT]' not in log,log
   if case=='motion_retry':assert '[NO_MOTION_RESPONSE]' in log and '[INPUT_RECOVERED]' in log and '[REPLAN_READY]' in log,log
   if case=='terminal_shift':assert '[REACQUIRE_STOP]' not in log and '[POSITION_APPROXIMATE]' in log,log
   if case in ('turn_drift','turn_drift_path'):assert '[ALIGN_ODOM_DRIFT]' in log and '[REACQUIRE_STOP]' in log and '[REPLAN_READY]' in log,log
   if case=='turn_inflation':
    checks=[json.loads(line.split('[TURN_CHECK] ',1)[1]) for line in log.splitlines() if '[TURN_CHECK] ' in line]
    assert checks and max(v['ignored_inflated_cells'] for v in checks)>0,log
   if case=='tf_transient_turn':
    assert '[INPUT_PAUSE]' in log and '[INPUT_RECOVERED]' in log and 'ALIGN_NO_PROGRESS' not in log,log
    assert not any(loss_at+.55<v[1]<loss_at+13. for v in casecmd),'Motion while TF stale'
   if case=='tf_transient':
    assert '[INPUT_PAUSE]' in log and '[INPUT_RECOVERED]' in log and '[REPLAN_READY]' in log,log
    assert not any(loss_at+.55<v[1]<loss_at+.8 for v in casecmd),'Motion while TF stale'
  if case=='curved_path':
   def segment_distance(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)
   maximum=max(min(segment_distance(x,y,a,b) for a,b in zip(vertices,vertices[1:])) for k,x,y,w in trajectory if k==case)
   assert maximum<.25,maximum
   reports['curved_max_cross_track_m']=maximum
  if case=='slow_turn':assert time.monotonic()-start>15.,log
  assert all(-1e-6<=v[2]<=.300001 and abs(v[3])<=.400001 for v in casecmd),casecmd
  reports[case]=dict(result='PASS',exit_code=process.returncode,final_pose=[x,y,yaw],forwarded_commands=len(casecmd))
  (W/'report.json').write_text(json.dumps(reports,indent=2))
  process=None
 reports.update(raw_nav2_commands=raw_count,hardware_connected=False,synthetic_floor_only=True,assumed_angular_deadband=.08,bridge_acceleration=[.5,1.])
 (W/'report.json').write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2))
finally:
 if process is not None and process.poll() is None:process.terminate();process.wait(timeout=5)
 os.killpg(launch.pid,signal.SIGINT)
 try:launch.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
 console.close();n.destroy_node();rclpy.try_shutdown()
