"""Verify 1Hz RobotState and genuine timeout using fake SDK; never connects to a robot."""
from pathlib import Path
import os
import json
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String

ws = Path(__file__).resolve().parents[3]
pkg = ws/'src/d1_host_control'
output = ws/'output/state-rate-smoke'
output.mkdir(parents=True, exist_ok=True)
fake = output/'libfake_robot_sdk.so'
subprocess.run(['/usr/bin/g++', '-std=c++17', '-shared', '-fPIC', '-pthread',
                '-I'+str(pkg/'third_party/robot_sdk/include'),
                str(pkg/'test/fake_sdk.cpp'), '-o', str(fake)], check=True)
os.environ['ROS_DOMAIN_ID'] = '148'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy.init()
node = rclpy.create_node('d1_state_rate_offline_check')
cmd = node.create_publisher(Twist, 'cmd_vel', 1)
stair_cmd = node.create_publisher(Twist, 'stair_cmd_vel', 1)
stair_permit = node.create_publisher(Bool, 'stair_motion_permit', 1)
posture = node.create_publisher(Int32, 'd1_posture', 1)
readiness = []
statuses = []
node.create_subscription(Bool, 'd1_motion_ready', lambda msg: readiness.append(msg.data), 1)
node.create_subscription(String, 'd1_motion_status', lambda msg: statuses.append(json.loads(msg.data)), 1)
bridge = None

def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)

def wait_for(predicate, message, seconds=5.):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if predicate():
            return
        spin(.04)
    raise AssertionError(message + '; latest SDK status=' + repr(statuses[-1] if statuses else None))

def drive(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        message = Twist()
        message.linear.x = .1
        cmd.publish(message)
        spin(.04)

def stair_drive(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        stair_permit.publish(Bool(data=True))
        nav_message = Twist()
        nav_message.linear.x = .2
        cmd.publish(nav_message)
        message = Twist()
        message.linear.x = .08
        stair_cmd.publish(message)
        spin(.04)

def confirm_return_to_walk(posture, statuses, lines, event, wait_for):
    """Fake SDK: command receipt -> intermediate StandUp -> injected Walk.

    Keep the deliberate intermediate state. StandUp or an old Walk sample
    must never be treated as fresh stable motion permission.
    """
    assert statuses, 'Missing SDK status before return-to-Walk request'
    request_sequence = statuses[-1]['state_sequence']
    stand_count = lines().count('StandUp')
    posture.publish(Int32(data=1))
    wait_for(lambda: lines().count('StandUp') > stand_count,
             'Return-to-Walk request did not reach fake SDK StandUp')
    wait_for(lambda: statuses and statuses[-1].get('motion') == 'StandUp' and
             statuses[-1]['state_sequence'] > request_sequence and
             statuses[-1]['state_fresh'] and statuses[-1]['motion_pending'] and
             not statuses[-1]['ready'] and not statuses[-1]['bridge_motion_permitted'],
             'Fake StandUp intermediate state did not keep motion blocked')
    stand_sequence = statuses[-1]['state_sequence']
    # StandUp() intentionally reports STAND_UP only. The harness must supply
    # the later stable report, just as it does during initial startup.
    event('walk')
    wait_for(lambda: statuses and statuses[-1].get('motion') == 'Walk' and
             statuses[-1]['state_sequence'] > stand_sequence and
             statuses[-1]['state_fresh'] and statuses[-1]['control_held'] and
             statuses[-1]['ready'] and not statuses[-1]['motion_pending'] and
             statuses[-1]['bridge_motion_permitted'],
             'Bridge did not return to a fresh confirmed Walk state')


try:
    with tempfile.TemporaryDirectory(prefix='d1_state_rate_') as folder:
        log = Path(folder)/'sdk.log'
        commands = Path(folder)/'commands'
        commands.touch()
        env = dict(os.environ, LD_PRELOAD=str(fake), D1_TEST_SDK_LOG=str(log),
                   D1_TEST_SDK_COMMANDS=str(commands))
        with (output/'bridge.log').open('w') as console:
            bridge = subprocess.Popen([str(ws/'install/d1_host_control/lib/d1_host_control/d1_bridge'),
                                       '--ros-args', '-p', 'robot_ip:=127.0.0.2',
                                       '-p', 'watchdog_timeout:=0.3', '-p',
                                       'stair_max_lin_vel:=0.08', '-p', 'trace_control:=true'],
                                      env=env, stdout=console, stderr=subprocess.STDOUT)
            def lines():
                return log.read_text().splitlines() if log.exists() else []
            def event(value):
                with commands.open('a') as stream:
                    stream.write(value+'\n')
            def moves(start=0):
                return [tuple(map(float, line.split()[1:])) for line in lines()[start:]
                        if line.startswith('Move ')]
            wait_for(lambda: 'StandUp' in lines(), 'SDK bridge did not start')
            wait_for(lambda: cmd.get_subscription_count() >= 1, 'ROS velocity subscriptions unavailable')
            event('walk')
            wait_for(lambda: readiness and readiness[-1], 'Confirmed Walk not ready')
            wait_for(lambda: statuses and statuses[-1].get('motion') == 'Walk',
                     'Structured status omitted the actual Walk mode')
            walk_status = statuses[-1]
            assert walk_status['state_sequence'] > 0 and walk_status['state_fresh']
            assert walk_status['control_held'] and walk_status['bridge_motion_permitted']
            assert not walk_status['soft_stop'] and not walk_status['hardware_stop']
            assert not walk_status['motion_pending']
            posture.publish(Int32(data=3))
            wait_for(lambda: any(status.get('motion') == 'Stair' and
                                 status['state_sequence'] > walk_status['state_sequence']
                                 for status in statuses),
                     'Fresh SDK Stair state was not published')
            stair_status = statuses[-1]
            assert not stair_status['ready'], 'Walk-only readiness must remain false in Stair'
            assert not stair_status['bridge_motion_permitted'], \
                'Stair mode must remain blocked until the mission lease arrives'
            assert not stair_status['stair_motion_permitted'], \
                'Stair motion must remain locked until the mission lease arrives'
            offset = len(lines())
            drive(.5)
            assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
                'Ordinary Nav2 /cmd_vel escaped into Stair mode'
            stair_drive(.6)
            assert any(fb > 0.01 for _, fb, _ in moves(offset)), \
                'Fresh stair lease and stair_cmd_vel did not reach the fake SDK'
            wait_for(lambda: statuses and statuses[-1]['stair_motion_permitted'],
                     'Fresh stair lease was not reported')
            spin(.45)
            assert not statuses[-1]['stair_motion_permitted'] and \
                not statuses[-1]['bridge_motion_permitted'], \
                'Stair permit did not expire after heartbeat loss'
            assert all(abs(value) < 1e-6 for value in moves()[-1]), \
                'Stair permit heartbeat timeout did not stop SDK motion'
            stair_drive(.4)
            stair_permit.publish(Bool(data=False))
            spin(.15)
            assert all(abs(value) < 1e-6 for value in moves()[-1]), \
                'Revoking the stair lease did not stop SDK motion'
            assert not statuses[-1]['bridge_motion_permitted'] and \
                not statuses[-1]['stair_motion_permitted']
            confirm_return_to_walk(posture, statuses, lines, event, wait_for)
            print('PASS: Stair exit remains blocked until a new confirmed Walk report', flush=True)
            event('report_1hz')
            wait_for(lambda: 'ReportMode1Hz' in lines(), 'Fake SDK did not switch to 1Hz')
            spin(.15)
            readiness.clear()
            offset = len(lines())
            drive(3.2)
            assert readiness and all(readiness), 'Normal 1Hz RobotState incorrectly became unready'
            assert any(fb > .05 for _, fb, _ in moves(offset)), '1Hz RobotState blocked valid velocity'
            print('PASS: 1Hz RobotState stays ready over three normal report periods', flush=True)
            assert statuses and statuses[-1]['version'] == 'd1-navigation-v6.5'
            state_sequences = {status['state_sequence'] for status in statuses}
            assert len(state_sequences) >= 3, 'Fresh SDK state sequences not published'
            assert statuses[-1]['reason'] == 'ready', 'Structured reason not ready'
            event('report_1700ms')
            wait_for(lambda: 'ReportMode1700ms' in lines(), 'Slow-report mode unavailable')
            readiness.clear()
            drive(3.6)
            assert readiness and all(readiness), '1.7s state gaps incorrectly blocked motion'
            print('PASS: two-second state limit permits 1.7s report intervals', flush=True)

            # ROS velocity watchdog must remain independent of slow RobotState reports.
            spin(.45)
            assert moves() and all(abs(value) < 1e-6 for value in moves()[-1]), \
                '0.3s velocity watchdog no longer stops'
            print('PASS: 0.3s velocity watchdog still stops with healthy 1Hz RobotState', flush=True)

            event('reports_off')
            wait_for(lambda: 'ReportsOff' in lines(), 'Fake SDK did not stop reports')
            drive(2.4)
            assert readiness and not readiness[-1], 'Missing RobotState did not revoke readiness'
            assert statuses and not statuses[-1]['state_fresh'] and \
                not statuses[-1]['bridge_motion_permitted'], \
                'Missing RobotState did not revoke the structured motion permit'
            assert moves() and all(abs(value) < 1e-6 for value in moves()[-1]), \
                'Movement continued after genuine RobotState timeout'
            print('PASS: actual RobotState interruption revokes readiness and stops movement', flush=True)

            event('reports_on')
            wait_for(lambda: readiness and readiness[-1], 'Resumed reports not observed')
            drive(.2)
            safety_epoch = statuses[-1]['safety_epoch']
            event('lost')
            wait_for(lambda: readiness and not readiness[-1], 'Control loss did not revoke readiness')
            wait_for(lambda: statuses and not statuses[-1]['control_held'] and
                     not statuses[-1]['bridge_motion_permitted'],
                     'Control loss did not revoke the structured motion permit')
            offset = len(lines())
            drive(.3)
            assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
                'Movement was sent after control loss'
            assert statuses[-1]['safety_epoch'] > safety_epoch, 'Control-loss history missing'
            print('PASS: explicit SDK control loss blocks motion and advances safety epoch', flush=True)
            bridge.send_signal(signal.SIGINT)
            bridge.wait(timeout=5)
            assert bridge.returncode == 0, 'Bridge shutdown failed'
        diagnostics = (output/'bridge.log').read_text()
        assert 'd1-navigation-v6.5' in diagnostics, 'Old SDK binary still installed; rebuild required'
        assert 'reason=RobotState_timeout' in diagnostics, 'State timeout reason missing'
        assert 'reason=control_not_held' in diagnostics, 'Control loss reason missing'
finally:
    if bridge is not None and bridge.poll() is None:
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait()
    node.destroy_node()
    rclpy.shutdown()
print('State-rate regression checks passed. No real robot SDK connection was made.', flush=True)
