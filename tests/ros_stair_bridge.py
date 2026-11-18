"""Isolated ROS + socket-free SDK double. Never connects to a robot."""
import json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'work/stair_bridge_test';out.mkdir(parents=True,exist_ok=True)
fake=out/'libfake_robot_sdk.so';pkg=ROOT/'src/d1_host_control'
subprocess.run(['g++','-std=c++17','-shared','-fPIC','-pthread','-I'+str(pkg/'third_party/robot_sdk/include'),str(pkg/'test/fake_sdk.cpp'),'-o',str(fake)],check=True)
os.environ.update(ROS_DOMAIN_ID='178',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Bool
rclpy.init();n=rclpy.create_node('isolated_stair_bridge_check');status={}
def receive(m):status.update(json.loads(m.data))
n.create_subscription(String,'/stair_test/d1_motion_status',receive,1)
flat=n.create_publisher(Twist,'/stair_test/cmd_vel',1)
vel=n.create_publisher(Twist,'/stair_test/stair_cmd_vel',1)
permit=n.create_publisher(Bool,'/stair_test/stair_motion_permit',1)
action=n.create_publisher(String,'/stair_test/d1_action',1)
commands=out/'events';commands.write_text('');sdklog=out/'sdk.log'
env=dict(os.environ,LD_PRELOAD=str(fake),D1_TEST_SDK_LOG=str(sdklog),D1_TEST_SDK_COMMANDS=str(commands))
log=(out/'bridge.log').open('w')
proc=subprocess.Popen([str(ROOT/'install/d1_host_control/lib/d1_host_control/d1_bridge'),'--ros-args','-r','__ns:=/stair_test','-p','robot_ip:=127.0.0.1','-p','max_lin_vel:=0.3','-p','stair_max_lin_vel:=0.5'],env=env,stdout=log,stderr=subprocess.STDOUT)
def spin(sec):
    end=time.monotonic()+sec
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
def wait(predicate):
    end=time.monotonic()+6
    while time.monotonic()<end:
        spin(.05)
        if predicate():return
    raise AssertionError(status)
def send(seconds,lease=True,stair=.4,ordinary=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        permit.publish(Bool(data=lease));m=Twist();m.linear.x=stair;vel.publish(m)
        if ordinary is not None:m=Twist();m.linear.x=ordinary;flat.publish(m)
        spin(.04)
try:
    wait(lambda:status.get('control_held'))
    assert 'offline-test' in (out/'bridge.log').read_text(), 'SDK double not active'
    assert 'Stair' not in sdklog.read_text(), 'Startup must not switch mode'
    send(.3);assert status['command_zero']
    action.publish(String(data='stair'));wait(lambda:status.get('motion')=='Stair' and not status.get('motion_pending'))
    send(.4,lease=False);assert status['command_zero'],'No movement without permit'
    send(.8,ordinary=.9);assert .35<status['bridge_ramped'][0]<=.5, status
    spin(.6);assert status['command_zero'],'Lease/command timeout must stop'
    send(.4);permit.publish(Bool(data=False));spin(.15);assert status['command_zero']
    with commands.open('a') as f:f.write('walk\n')
    wait(lambda:status.get('motion')=='Walk')
    send(.4,ordinary=.8);assert status['command_zero'] and status['stair_fault'],'Mode loss must not fall through to Walk'
    print('PASS: Stair confirmation, independent cap, flat command exclusion, permit timeout, revoke and mode-loss latch')
finally:
    permit.publish(Bool(data=False));vel.publish(Twist());spin(.2)
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    log.close();n.destroy_node();rclpy.shutdown()
