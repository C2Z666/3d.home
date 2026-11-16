"""ROS localization lifecycle test with a generated metric PnP bank, not field accuracy."""
from pathlib import Path
import os,sys,time,json,subprocess,hashlib,shutil,array
import numpy as np,cv2,yaml
SOURCE=Path(__file__).resolve().parents[1]; WORK=Path(sys.argv[1]).resolve(); WORK.mkdir(parents=True,exist_ok=True)
ROOT=WORK/'isolated_project'
ROOT.mkdir(exist_ok=True)
for folder in ('scripts','maps','config'):
 shutil.copytree(SOURCE/folder,ROOT/folder,dirs_exist_ok=True)
(ROOT/'assets/output/demo').mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'scripts'))
from geometry import calibration,imu_from_base,transform
from scipy.spatial.transform import Rotation
c=yaml.safe_load((ROOT/'config/calibration.yaml').read_text()); c['D']=[[0.],[0.],[0.],[0.],[0.]]
calib=ROOT/'config/calibration.yaml'; calib.write_text(yaml.safe_dump(c))
K,_,_,B_C=calibration(calib)
rng=np.random.default_rng(27); im=np.full((540,960,3),235,np.uint8)
for i in range(600):
 x,y=rng.integers([20,20],[940,520]); color=tuple(int(v) for v in rng.integers(0,210,3))
 cv2.circle(im,(int(x),int(y)),int(rng.integers(2,9)),color,-1)
features=cv2.SIFT_create(nfeatures=2200,contrastThreshold=.025); kp,desc=features.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),None)
uv=np.float32([k.pt for k in kp]); rays=np.c_[uv,np.ones(len(uv))]@np.linalg.inv(K).T
world=transform(B_C,rays*4)
visual=ROOT/'assets/output/demo/visual_map.npz'; np.savez_compressed(visual,stamps=np.array([1.]),image_indices=np.array([0]),pose_0=np.eye(4),uv_0=uv,desc_0=desc,world_0=world,image_0=cv2.imencode('.jpg',im)[1],odom_times=np.array([0.,1.]),odom_poses=np.array([np.eye(4),np.eye(4)]),calibration_sha256=hashlib.sha256(calib.read_bytes()).hexdigest())
from site_identity import hashes
(ROOT/'config/site_identity.json').write_text(json.dumps(dict(map_id='synthetic_test_only',sha256=hashes(ROOT))))
os.environ.update(ROS_DOMAIN_ID='180',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); node=rclpy.create_node('d1_synthetic_localization_test')
odpub=node.create_publisher(Odometry,'/Odometry',qos_profile_sensor_data); impub=node.create_publisher(Image,'/front_camera/image',qos_profile_sensor_data)
latest={}; poses=[]
def onstatus(m):
 global latest
 latest=json.loads(m.data)
node.create_subscription(String,'/d1/localization/status',onstatus,1)
node.create_subscription(PoseStamped,'/d1/localization/pose6d',lambda m:poses.append(m),10)
I_B=imu_from_base(calib); O=np.linalg.inv(I_B); q=Rotation.from_matrix(O[:3,:3]).as_quat()
raw=cv2.resize(im,(1920,1080)); raw_data=array.array('B',raw.tobytes()); last_image=0
console=(WORK/'node.log').open('w'); process=subprocess.Popen(['/usr/bin/python3',str(ROOT/'scripts/localization_live.py'),'--visual-map',str(visual),'--calibration',str(calib),'--odom-frame','synthetic_odom','--body-frame','synthetic_imu','--max-visual-age','3.0'],stdout=console,stderr=subprocess.STDOUT)
def step(images=True,rewind=False):
 global last_image
 stamp=node.get_clock().now().to_msg()
 if rewind: stamp.sec-=1
 od=Odometry(); od.header.stamp=stamp; od.header.frame_id='synthetic_odom'; od.child_frame_id='synthetic_imu'
 od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=map(float,O[:3,3])
 od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=map(float,q); odpub.publish(od)
 if images and time.monotonic()-last_image>.22:
  m=Image(); m.header.stamp=stamp; m.header.frame_id='synthetic_camera'; m.width=1920; m.height=1080; m.encoding='bgr8'; m.step=1920*3; m.data=raw_data; impub.publish(m); last_image=time.monotonic()
 rclpy.spin_once(node,timeout_sec=.03)
def wait(pred,seconds,images=True):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if process.poll() is not None: raise AssertionError('localization node exited: '+(WORK/'node.log').read_text())
  if pred(): return
  step(images)
 raise AssertionError('condition timed out; status='+repr(latest))
def service():
 client=node.create_client(Trigger,'/d1/localization/relocalize')
 assert client.wait_for_service(timeout_sec=3)
 f=client.call_async(Trigger.Request());wait(lambda:f.done(),5,images=False)
 response=f.result();node.destroy_client(client);return response
try:
 wait(lambda:latest.get('ready') is True and len(poses)>3,25)
 pose=poses[-1].pose; err=float(np.linalg.norm([pose.position.x,pose.position.y,pose.position.z]))
 assert err<.05,('synthetic known-pose error',err)
 epoch=latest['epoch']
 wait(lambda:latest.get('ready') is False and latest.get('recovery_required') is True,6,images=False)
 assert latest['tracking_valid'] and latest['epoch']>epoch
 end=time.monotonic()+2
 while time.monotonic()<end:step(True)
 assert not latest['ready'],'recovery must not silently re-enable old mission'
 blocked=node.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
 end=time.monotonic()+1
 while time.monotonic()<end:step(False)
 response=service();assert not response.success and 'command publisher' in response.message,response.message
 node.destroy_publisher(blocked)
 end=time.monotonic()+1
 while time.monotonic()<end:step(False)
 response=service();assert response.success,response.message
 wait(lambda:latest.get('ready') is True and not latest.get('relocalizing'),20)
 cli_log=(WORK/'relocalize_cli.log').open('w')
 cli=subprocess.Popen([sys.executable,str(ROOT/'scripts/relocalize.py'),'--confirm-stopped'],stdout=cli_log,stderr=subprocess.STDOUT)
 try:
  wait(lambda:cli.poll() is not None,25)
  assert cli.returncode==0,(WORK/'relocalize_cli.log').read_text()
 finally:
  if cli.poll() is None:cli.terminate();cli.wait(timeout=5)
  cli_log.close()
 step(False,True)
 wait(lambda:latest.get('tracking_valid') is False,3,images=False)
 report=dict(synthetic_pnp_and_online_correction='PASS',known_pose_position_error_m=err,visual_timeout_revokes_ready='PASS',recovery_requires_explicit_request='PASS',active_command_publisher_blocks_recovery='PASS',stationary_three_frame_recovery='PASS',relocalize_cli_epoch_confirmation='PASS',odometry_reset_revokes_ready='PASS',field_accuracy_verified=False)
 (WORK/'report.json').write_text(json.dumps(report,indent=2)); print(json.dumps(report),flush=True)
finally:
 process.terminate(); process.wait(timeout=8); console.close(); node.destroy_node(); rclpy.shutdown()
