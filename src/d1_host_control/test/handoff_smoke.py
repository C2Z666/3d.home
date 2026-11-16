"""Exercise the real v3 bridge against fake SDK on loopback, never a real robot."""
from pathlib import Path
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


ws = Path(__file__).resolve().parents[3]
pkg = ws/'src/d1_host_control'
output = ws/'output/handoff-smoke'
output.mkdir(parents=True, exist_ok=True)
fake = output/'libfake_robot_sdk.so'
subprocess.run(['/usr/bin/g++', '-std=c++17', '-shared', '-fPIC', '-pthread',
                '-I'+str(pkg/'third_party/robot_sdk/include'),
                str(pkg/'test/fake_sdk.cpp'), '-o', str(fake)], check=True)
os.environ.update(ROS_DOMAIN_ID='149', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
rclpy.init(args=[])
node = rclpy.create_node('d1_handoff_offline_check')
cmd = node.create_publisher(Twist, 'cmd_vel', 1)
stair_cmd = node.create_publisher(Twist, 'stair_cmd_vel', 1)
permit = node.create_publisher(Bool, 'stair_motion_permit', 1)
hold = node.create_publisher(Bool, 'nav_handoff_hold', 1)
action = node.create_publisher(String, 'd1_action', 1)
statuses = []
node.create_subscription(String, 'd1_motion_status', lambda m:statuses.append(json.loads(m.data)), 1)
bridge = None


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def wait_for(predicate, message, seconds=6.):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if predicate():
            return
        spin(.04)
    raise AssertionError(message+'; latest='+repr(statuses[-1] if statuses else None))


try:
    with tempfile.TemporaryDirectory(prefix='d1_handoff_fake_') as folder:
        log, commands = Path(folder)/'sdk.log', Path(folder)/'commands'
        commands.touch()
        def lines():
            return log.read_text().splitlines() if log.exists() else []
        def event(value):
            with commands.open('a') as stream:
                stream.write(value+'\n')
        def nonzero_moves(offset):
            return [line for line in lines()[offset:] if line.startswith('Move ') and
                    any(abs(float(v)) > 1e-6 for v in line.split()[1:])]
        env = dict(os.environ, LD_PRELOAD=str(fake), D1_TEST_SDK_LOG=str(log),
                   D1_TEST_SDK_COMMANDS=str(commands))
        binary = str(ws/'install/d1_host_control/lib/d1_host_control/d1_bridge')
        with (output/'bridge.log').open('w') as console:
            bridge = subprocess.Popen([binary, '--ros-args', '-p', 'robot_ip:=127.0.0.2',
                '-p', 'flat_test_lock:=true', '-p', 'stair_max_lin_vel:=0.0',
                '-p', 'trace_control:=true'], env=env, stdout=console, stderr=subprocess.STDOUT)
            wait_for(lambda:'StandUp' in lines(), 'Fake SDK did not receive startup')
            wait_for(lambda:all(p.get_subscription_count() for p in (cmd, stair_cmd, permit, hold, action)),
                     'ROS subscriptions unavailable')
            event('walk')
            wait_for(lambda:statuses and statuses[-1]['ready'], 'No fresh Walk')
            assert statuses[-1].get('handoff_protocol') == 1 and statuses[-1].get('flat_test_lock') is True
            # Before handoff ordinary flat navigation must still work.
            offset = len(lines())
            for _ in range(15):
                message = Twist(); message.linear.x = .1
                cmd.publish(message); spin(.04)
            assert nonzero_moves(offset), 'Flat navigation no longer works before handoff'
            cmd.publish(Twist()); spin(.1)
            stair_count = lines().count('Stair')
            action.publish(String(data='stair')); spin(.3)
            assert lines().count('Stair') == stair_count, 'Stair escaped before hold'
            print('PASS: flat Walk commands work; flat test rejects Stair before hold', flush=True)
            hold.publish(Bool(data=True))
            wait_for(lambda:statuses[-1].get('nav_handoff_hold') is True and
                     statuses[-1].get('command_zero') is True and
                     statuses[-1]['bridge_motion_permitted'] is False, 'No hold acknowledgement')
            offset = len(lines())
            def attack(seconds):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    message = Twist(); message.linear.x = .1
                    cmd.publish(message)
                    stair_cmd.publish(message)
                    permit.publish(Bool(data=True))
                    hold.publish(Bool(data=False))  # A stale false cannot unlock.
                    spin(.04)
            attack(.6)
            assert statuses[-1]['nav_handoff_hold'] and not statuses[-1]['bridge_motion_permitted']
            assert not nonzero_moves(offset), 'Velocity escaped latched hold'
            print('PASS: queued nav/stair commands and false hold cannot unlock movement', flush=True)
            baseline = statuses[-1]
            action.publish(String(data='stair'))
            wait_for(lambda:statuses[-1]['motion'] == 'Stair' and
                     not statuses[-1]['motion_pending'] and
                     statuses[-1]['state_sequence'] > baseline['state_sequence'] and
                     statuses[-1]['pose_request_id'] == baseline['pose_request_id']+1 and
                     statuses[-1]['pose_confirmed_id'] == statuses[-1]['pose_request_id'],
                     'Stair request not confirmed with a fresh SDK report')
            attack(.6)
            assert not statuses[-1]['stair_motion_permitted'] and not nonzero_moves(offset)
            print('PASS: Stair is confirmed but stair velocity stays disabled', flush=True)
            count = lines().count('StandUp')
            baseline = statuses[-1]
            action.publish(String(data='standup'))
            wait_for(lambda:lines().count('StandUp') > count, 'No return StandUp request')
            wait_for(lambda:statuses[-1]['motion'] == 'StandUp' and statuses[-1]['motion_pending'],
                     'Intermediate StandUp lost')
            event('walk')
            wait_for(lambda:statuses[-1]['motion'] == 'Walk' and
                     not statuses[-1]['motion_pending'] and
                     statuses[-1]['state_sequence'] > baseline['state_sequence'] and
                     statuses[-1]['pose_confirmed_id'] == statuses[-1]['pose_request_id'],
                     'No confirmed return Walk')
            attack(.6)
            assert statuses[-1]['nav_handoff_hold'] and not statuses[-1]['ready']
            assert statuses[-1]['reason'] == 'navigation_handoff_hold'
            assert not nonzero_moves(offset), 'Old velocity resumed after returning to Walk'
            print('PASS: return Walk stays held; old commands cannot resume motion', flush=True)
            count = lines().count('StandUp')
            epoch = statuses[-1]['safety_epoch']
            event('lost')
            wait_for(lambda:not statuses[-1]['control_held'] and statuses[-1]['safety_epoch'] > epoch,
                     'Control loss not reflected')
            event('available')
            wait_for(lambda:statuses[-1]['control_held'], 'Fake control not reacquired')
            attack(.4)
            assert lines().count('StandUp') == count, 'Hold reacquisition automatically changed posture'
            assert statuses[-1]['nav_handoff_hold'] and not nonzero_moves(offset)
            print('PASS: control reacquisition during hold does not restart motion or request StandUp', flush=True)
            bridge.send_signal(signal.SIGINT)
            bridge.wait(timeout=5)
            assert bridge.returncode == 0
finally:
    if bridge is not None and bridge.poll() is None:
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill(); bridge.wait()
    node.destroy_node()
    rclpy.shutdown()
print('Handoff bridge checks passed. Fake SDK only; no real robot connection.', flush=True)
