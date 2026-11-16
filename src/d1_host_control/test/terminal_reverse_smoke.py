"""Opt-in SDK reverse lease regression using LD_PRELOAD; no robot connection."""
from pathlib import Path
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

ws=Path(__file__).resolve().parents[3]
pkg=ws/'src/d1_host_control'
output=ws/'output/terminal-reverse-smoke';output.mkdir(parents=True,exist_ok=True)
fake=output/'libfake_robot_sdk.so'
subprocess.run(['/usr/bin/g++','-std=c++17','-shared','-fPIC','-pthread',
                '-I'+str(pkg/'third_party/robot_sdk/include'),str(pkg/'test/fake_sdk.cpp'),
                '-o',str(fake)],check=True)
os.environ['ROS_DOMAIN_ID']='149';os.environ['ROS_LOCALHOST_ONLY']='1'
rclpy.init();node=rclpy.create_node('terminal_reverse_offline_check')
cmd=node.create_publisher(Twist,'cmd_vel',1)
permit=node.create_publisher(Bool,'terminal_reverse_permit',1)
readiness=[]
node.create_subscription(Bool,'d1_motion_ready',lambda msg:readiness.append(msg.data),1)
bridge=None

def spin(seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)

def wait_for(check,message):
    until=time.monotonic()+5.
    while time.monotonic()<until:
        if check():return
        spin(.04)
    raise AssertionError(message)

def command(seconds,vx,lease=None,vy=0.):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        if lease is not None:permit.publish(Bool(data=lease))
        value=Twist();value.linear.x=vx;value.linear.y=vy;cmd.publish(value)
        spin(.04)

try:
    with tempfile.TemporaryDirectory(prefix='d1_reverse_') as folder:
        log=Path(folder)/'sdk.log';events=Path(folder)/'commands';events.touch()
        env=dict(os.environ,LD_PRELOAD=str(fake),D1_TEST_SDK_LOG=str(log),D1_TEST_SDK_COMMANDS=str(events))
        with (output/'bridge.log').open('w') as console:
            bridge=subprocess.Popen([str(ws/'install/d1_host_control/lib/d1_host_control/d1_bridge'),
                '--ros-args','-p','robot_ip:=127.0.0.2','-p','allow_terminal_reverse:=true'],
                env=env,stdout=console,stderr=subprocess.STDOUT)
            def lines():return log.read_text().splitlines() if log.exists() else []
            def moves(offset=0):
                return [tuple(map(float,line.split()[1:])) for line in lines()[offset:] if line.startswith('Move ')]
            wait_for(lambda:'StandUp' in lines(),'No StandUp')
            with events.open('a') as stream:stream.write('walk\n')
            wait_for(lambda:readiness and readiness[-1],'No confirmed Walk')
            wait_for(lambda:cmd.get_subscription_count()>0 and permit.get_subscription_count()>0,'No subscriptions')

            offset=len(lines());command(.5,-.3)
            assert moves(offset) and all(fb>=0 for _,fb,_ in moves(offset)), 'Reverse without lease'
            offset=len(lines());command(.7,-.3,True)
            assert any(fb<-.05 for _,fb,_ in moves(offset)), 'Authorized reverse did not move'
            assert all(fb>=-.060001 and lr==0 for lr,fb,_ in moves(offset)), 'Reverse speed cap failed'

            command(.35,-.3)  # velocity keeps arriving, only authorization expires
            offset=len(lines());command(.2,-.3)
            assert moves(offset) and all(abs(fb)<1e-6 for _,fb,_ in moves(offset)), 'Expired lease allowed motion'
            command(.4,-.06,True)
            command(.1,-.06,False)
            assert all(abs(v)<1e-6 for v in moves()[-1]), 'Explicit lease revoke did not stop'

            offset=len(lines());command(.4,-.06,True,vy=.2)
            assert moves(offset) and all(all(abs(v)<1e-6 for v in move) for move in moves(offset)), 'Lateral escaped gate'
            command(.4,-.06,True)
            spin(.4)
            assert all(abs(v)<1e-6 for v in moves()[-1]), 'Connection silence failed to stop'
            command(.4,-.06,True)
            count=lines().count('SoftEmergencyStop 1');offset=len(lines())
            bridge.send_signal(signal.SIGINT);bridge.wait(timeout=5)
            assert bridge.returncode==0
            assert moves(offset) and all(abs(v)<1e-6 for v in moves(offset)[-1]), 'Exit omitted zero'
            assert lines().count('SoftEmergencyStop 1')==count, 'Exit enabled soft emergency stop'
finally:
    if bridge is not None and bridge.poll() is None:
        bridge.send_signal(signal.SIGINT)
        try:bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
    node.destroy_node();rclpy.shutdown()
print('PASS: opt-in reverse lease, speed cap, expiry, revoke, lateral rejection and zero exit. Fake SDK only.')
