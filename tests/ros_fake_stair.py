"""Exercise real ROS processes with an ABI fake SDK. No robot socket is opened."""
from pathlib import Path
import os,subprocess,time,json,sys
ROOT=Path(__file__).resolve().parents[1]
WORK=Path(sys.argv[1]).resolve(); WORK.mkdir(parents=True,exist_ok=True)
INSTALL=WORK.parent/'install'
pkg=ROOT/'src/d1_host_control'; fake=WORK/'libfake_robot_sdk.so'
subprocess.run(['g++','-std=c++17','-shared','-fPIC','-pthread','-I'+str(pkg/'third_party/robot_sdk/include'),str(pkg/'test/fake_sdk.cpp'),'-o',str(fake)],check=True)
os.environ.update(ROS_DOMAIN_ID='179',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist
rclpy.init(); node=rclpy.create_node('d1_extracted_bridge_offline_test'); latest={}
def receive(m):
 global latest
 latest=json.loads(m.data)
node.create_subscription(String,'/d1/sdk/d1_motion_status',receive,1)
action=node.create_publisher(String,'/d1/sdk/d1_action',1)
cmd=node.create_publisher(Twist,'/d1/sdk/supervised_cmd_vel',1)
staircmd=node.create_publisher(Twist,'/d1/sdk/stair_cmd_vel',1)
permit=node.create_publisher(Bool,'/d1/sdk/stair_motion_permit',1)

def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
def wait(pred,message,seconds=8):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if pred(): return
  spin(.04)
 raise AssertionError(message+' latest='+repr(latest))
def run_case(name,mode):
 global latest
 latest={}; log=WORK/(name+'.sdk.log'); commands=WORK/(name+'.commands'); commands.write_text('report_1hz\n')
 log.write_text(''); console=(WORK/(name+'.bridge.log')).open('w'); clientlog=(WORK/(name+'.client.log')).open('w')
 env=dict(os.environ,LD_PRELOAD=str(fake),D1_TEST_SDK_LOG=str(log),D1_TEST_SDK_COMMANDS=str(commands))
 bridge=subprocess.Popen([str(INSTALL/'d1_host_control/lib/d1_host_control/d1_bridge'),'--ros-args','-r','__ns:=/d1/sdk','-r','cmd_vel:=supervised_cmd_vel','-p','robot_ip:=127.0.0.2','-p','stair_max_lin_vel:=0.1','-p','max_lin_vel:=0.15'],env=env,stdout=console,stderr=subprocess.STDOUT)
 client=None
 def lines(): return log.read_text().splitlines()
 def nonzero(offset=0): return [x for x in lines()[offset:] if x.startswith('Move ') and any(abs(float(v))>1e-6 for v in x.split()[1:])]
 try:
  wait(lambda:latest.get('control_held') is True,'no control')
  assert 'StandUp' not in lines() and not any(x.startswith('SoftEmergencyStop') for x in lines()),'automatic posture/stop reset occurred'
  with commands.open('a') as stream: stream.write('walk\n')
  wait(lambda:latest.get('motion')=='Walk','no fake Walk')
  client=subprocess.Popen(['/usr/bin/python3',str(ROOT/'scripts/stair_start.py'),'--speed','0.05','--duration','1.2'],stdin=subprocess.PIPE,stdout=clientlog,stderr=subprocess.STDOUT,env=os.environ.copy(),text=True)
  wait(lambda:'输入 y' in (WORK/(name+'.client.log')).read_text(),'no Stair confirmation prompt',15)
  assert latest.get('motion')=='Stair' and not nonzero(),'movement occurred before y'
  # Ordinary Nav2 velocity must not move in Stair even after a valid SDK state.
  for _ in range(5):
   msg=Twist(); msg.linear.x=.1; cmd.publish(msg); spin(.04)
  assert not nonzero(),'ordinary velocity moved Stair'
  if mode=='eof': client.stdin.close()
  else: client.stdin.write('y\n'); client.stdin.flush()
  if mode=='loss':
   wait(lambda:bool(nonzero()),'never moved after y')
   with commands.open('a') as stream: stream.write('lost\n')
  wait(lambda:client.poll() is not None,'client did not exit',8)
  if mode=='eof': assert client.returncode!=0 and not nonzero(),'EOF moved robot'
  elif mode=='loss': assert client.returncode!=0,'control loss did not fault'
  else: assert client.returncode==0 and nonzero(),'y did not start bounded movement'
  offset=len(lines()); spin(.5); assert not nonzero(offset),'continued nonzero movement after segment exit'
  return dict(case=name,result='PASS',nonzero_writes=len(nonzero()))
 finally:
  if client and client.poll() is None: client.terminate(); client.wait(timeout=5)
  bridge.terminate(); bridge.wait(timeout=6); console.close(); clientlog.close(); spin(.3)
results=[]
try:
 for name,mode in [('y_and_bounded_stop','normal'),('eof_cancels','eof'),('control_loss_stops','loss')]:
  results.append(run_case(name,mode)); print(results[-1],flush=True)
 (WORK/'report.json').write_text(json.dumps(results,indent=2))
finally: node.destroy_node(); rclpy.shutdown()
