"""Exercise installed nodes, ROS topics, and SDK commands without robot/network access."""
from pathlib import Path
import os
import json
import pty
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, String

ws = Path(__file__).resolve().parents[3]
pkg = ws / 'src/d1_host_control'
output = ws / 'output/control-smoke'
output.mkdir(parents=True, exist_ok=True)
fake = output / 'libfake_robot_sdk.so'
subprocess.run(['/usr/bin/g++', '-std=c++17', '-shared', '-fPIC', '-pthread',
                '-I' + str(pkg / 'third_party/robot_sdk/include'),
                str(pkg / 'test/fake_sdk.cpp'), '-o', str(fake)], check=True)
os.environ['ROS_DOMAIN_ID'] = '147'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
rclpy.init()
node = rclpy.create_node('d1_control_offline_check')
cmd = node.create_publisher(Twist, 'cmd_vel', 10)
action = node.create_publisher(String, 'd1_action', 10)
posture = node.create_publisher(Int32, 'd1_posture', 10)
speeds, poses = [], []
node.create_subscription(Twist, 'cmd_vel', lambda msg: speeds.append(msg.linear.x), 10)
node.create_subscription(Int32, 'd1_posture', lambda msg: poses.append(msg.data), 10)
readiness = []
statuses = []
node.create_subscription(String, 'd1_motion_status', lambda msg: statuses.append(json.loads(msg.data)), 1)
node.create_subscription(Bool, 'd1_motion_ready', lambda msg: readiness.append(msg.data), 1)

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

def drive(seconds, vx=0.6, wz=0.0, vy=0.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        cmd.publish(msg)
        spin(0.08)

def wait_for(predicate, label, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return
        spin(0.05)
    raise AssertionError(label)

def stop(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

bridge = teleop = None
try:
    with tempfile.TemporaryDirectory(prefix='d1_control_') as tmp:
        log = Path(tmp) / 'sdk.log'
        commands = Path(tmp) / 'sdk.commands'
        commands.touch()
        env = dict(os.environ, LD_PRELOAD=str(fake), D1_TEST_SDK_LOG=str(log),
                   D1_TEST_SDK_COMMANDS=str(commands))
        bridge_output = (output / 'bridge.log').open('w')
        bridge = subprocess.Popen([str(ws / 'install/d1_host_control/lib/d1_host_control/d1_bridge'),
                                   '--ros-args', '-p', 'robot_ip:=127.0.0.2',
                                   '-p', 'trace_control:=true'],
                                  env=env, stdout=bridge_output, stderr=subprocess.STDOUT)

        def lines():
            return log.read_text().splitlines() if log.exists() else []

        def moves(start=0):
            return [tuple(map(float, line.split()[1:])) for line in lines()[start:]
                    if line.startswith('Move ')]

        def event(value):
            with commands.open('a') as file:
                file.write(value + '\n')

        wait_for(lambda: posture.get_subscription_count() > 0, 'Posture subscriber unavailable')
        wait_for(lambda: statuses and statuses[-1]['control_held'] and
                 not statuses[-1]['soft_stop'] and not statuses[-1]['hardware_stop'],
                 'SDK control and recovered stop state unavailable')
        posture.publish(Int32(data=1))
        wait_for(lambda: 'StandUp' in lines(), 'Startup did not request StandUp')
        wait_for(lambda: cmd.get_subscription_count() >= 2, 'Bridge ROS subscriptions unavailable')
        drive(0.4)
        assert not any(abs(fb) > 1e-6 for _, fb, _ in moves()), 'Movement before posture confirmation'
        print('PASS: startup/control acquisition; no movement during StandUp', flush=True)

        assert readiness and not readiness[-1], 'SDK marked ready before confirmed Walk'
        event('walk')
        drive(1.25, vx=5.0, wz=5.0)
        assert readiness[-1], 'SDK did not report confirmed Walk readiness'
        observed = moves()
        assert observed and max(fb for _, fb, _ in observed) > 0.7
        assert all(abs(fb) <= 0.80001 and abs(yaw) <= 0.80001 for _, fb, yaw in observed)
        spin(1.8)
        assert all(abs(value) < 1e-6 for value in moves()[-1]), 'Watchdog failed to stop'
        print('PASS: velocity clamping/ramp and watchdog stopping', flush=True)

        posture.publish(Int32(data=2))
        wait_for(lambda: 'BalanceStandUp' in lines(), 'Posture 2 did not call BalanceStandUp')
        spin(0.25)
        offset = len(lines())
        drive(0.35)
        assert not any(abs(fb) > 1e-6 for _, fb, _ in moves(offset)), 'Movement in balance stand'
        action.publish(String(data='soft_stop_on'))
        spin(0.3)
        offset = len(lines())
        drive(0.3)
        assert not any(abs(fb) > 1e-6 for _, fb, _ in moves(offset)), 'Movement during soft stop'
        action.publish(String(data='soft_stop_off'))
        spin(0.25)
        event('lost')
        spin(0.25)
        offset = len(lines())
        drive(0.4)
        assert not any(abs(fb) > 1e-6 for _, fb, _ in moves(offset)), 'Movement after control loss'
        event('available')
        wait_for(lambda: lines().count('TakeControl') == 2, 'Control was not reacquired')
        spin(0.2)
        event('walk')
        spin(0.25)
        print('PASS: emergency stop and control loss/reacquisition', flush=True)

        for pose_name, sdk_name in (
                ('gait', 'Gait'), ('slim', 'Slim'), ('crawl_walk', 'CrawlWalk'),
                ('dsb', 'DSB'), ('sand', 'Sand')):
            count = lines().count(sdk_name)
            action.publish(String(data=pose_name))
            wait_for(lambda: lines().count(sdk_name) == count + 1,
                     f'{sdk_name} was not requested')
            spin(0.25)
            if pose_name == 'gait':
                action.publish(String(data='gait'))
                spin(0.2)
                assert lines().count('Gait') == count + 1, 'Repeated key retriggered pending Gait'
            offset = len(lines())
            drive(0.35, vx=0.3)
            assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
                f'Movement before {sdk_name} state confirmation'
            event(pose_name)
            spin(0.25)
            offset = len(lines())
            spin(0.2)
            observed = moves(offset)
            assert observed and all(abs(v) < 1e-6 for move in observed for v in move), \
                f'{sdk_name} did not remain neutral after confirmation'
            if pose_name == 'gait':
                action.publish(String(data='gait'))
                spin(0.2)
                assert lines().count('Gait') == count + 1, 'Repeated key retriggered confirmed Gait'
            offset = len(lines())
            drive(0.6, vx=0.3)
            assert any(fb > 0.29 for _, fb, _ in moves(offset)), \
                f'No forward control in {sdk_name}'
            offset = len(lines())
            drive(0.85, vx=-0.3)
            assert any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)), \
                f'Reverse request did not stop in {sdk_name}'
            offset = len(lines())
            drive(0.4, vx=0.0, wz=0.5)
            assert any(yaw > 0.32 for _, _, yaw in moves(offset)), \
                f'No turn control in {sdk_name}'
            offset = len(lines())
            cmd.publish(Twist())
            wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                     f'Zero velocity did not stop {sdk_name}', seconds=0.3)
            after_stop = len(lines())
            spin(0.3)
            observed = moves(after_stop)
            assert observed and all(abs(v) < 1e-6 for move in observed for v in move), \
                f'{sdk_name} did not hold stopped'
            offset = len(lines())
            drive(0.65, vx=0.0, vy=0.3)
            assert any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)), f'Left translation request did not stop in {sdk_name}'
            offset = len(lines())
            drive(0.85, vx=0.0, vy=-0.3)
            assert any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)), f'Right translation request did not stop in {sdk_name}'
            cmd.publish(Twist())
            spin(0.2)
            action.publish(String(data='soft_stop_on'))
            spin(0.25)
            offset = len(lines())
            drive(0.3)
            assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
                f'Movement during soft stop in {sdk_name}'
            action.publish(String(data='soft_stop_off'))
            spin(0.25)
            drive(0.5, vx=0.3)
            spin(1.2)
            assert all(abs(value) < 1e-6 for value in moves()[-1]), \
                f'Watchdog failed to stop {sdk_name}'
            if pose_name == 'gait':
                # A confirmed gait request must never silently move in Walk.
                event('walk')
                spin(0.25)
                offset = len(lines())
                drive(0.45, vx=0.3)
                assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
                    'Gait unexpectedly fell through to Walk movement'
                event('gait')
                spin(0.25)
                offset = len(lines())
                drive(0.35, vx=0.3)
                assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
                    'Gait fault resumed a latched velocity without an explicit retry'
                action.publish(String(data='gait'))
                wait_for(lambda: lines().count('Gait') == count + 2, 'Gait retry was not sent')
                event('gait')
                spin(0.25)
                offset = len(lines())
                drive(0.6, vx=0.3)
                assert any(fb > 0.29 for _, fb, _ in moves(offset)), 'Explicit Gait retry did not recover'
                cmd.publish(Twist())
                spin(0.2)
                take_count, stand_count = lines().count('TakeControl'), lines().count('StandUp')
                event('lost')
                spin(0.25)
                event('available')
                wait_for(lambda: lines().count('TakeControl') == take_count + 1,
                         'Gait control was not reacquired')
                spin(0.25)
                assert lines().count('StandUp') == stand_count, \
                    'Gait control reacquisition silently selected Walk'
                offset = len(lines())
                drive(0.35, vx=0.3)
                assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
                    'Gait movement resumed automatically after control reacquisition'
                action.publish(String(data='gait'))
                wait_for(lambda: lines().count('Gait') == count + 3,
                         'Manual Gait retry after reacquisition failed')
                event('gait')
                spin(0.25)
                offset = len(lines())
                drive(0.6, vx=0.3)
                assert any(fb > 0.29 for _, fb, _ in moves(offset)), \
                    'Gait did not recover after explicit selection'
                cmd.publish(Twist())
                spin(0.2)
            print(f'PASS: {sdk_name} confirmation, forward/turn/stop and reverse/lateral rejection, emergency and watchdog',
                  flush=True)

        action.publish(String(data='climb'))
        wait_for(lambda: 'Climb' in lines(), 'Climb was not sent')
        spin(0.25)
        offset = len(lines())
        drive(0.35)
        assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
            'Movement before Climb state confirmation'
        event('climb')
        spin(0.25)
        offset = len(lines())
        spin(0.25)
        observed = moves(offset)
        assert observed and all(abs(value) < 1e-6 for move in observed for value in move), \
            'Climb must receive neutral Move frames with no fresh motion command'
        offset = len(lines())
        drive(0.6, vx=0.3)
        assert any(fb > 0.29 for _, fb, _ in moves(offset)), 'No forward control in Climb'
        offset = len(lines())
        drive(0.85, vx=-0.3)
        assert any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)), 'Reverse request did not stop in Climb'
        # A zero input must bypass the ramp, then remain neutral at 50 Hz.
        offset = len(lines())
        cmd.publish(Twist())
        wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                 'Climb did not receive an explicit stop', seconds=0.3)
        after_stop = len(lines())
        spin(0.35)
        observed = moves(after_stop)
        assert observed and all(abs(value) < 1e-6 for move in observed for value in move), \
            'Climb stop was not held'
        drive(0.5, vx=0.3)
        spin(1.2)
        assert all(abs(value) < 1e-6 for value in moves()[-1]), 'Climb watchdog failed to stop'
        action.publish(String(data='soft_stop_on'))
        spin(0.25)
        offset = len(lines())
        drive(0.3)
        assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
            'Movement in Climb during emergency stop'
        action.publish(String(data='soft_stop_off'))
        spin(0.25)
        print('PASS: Climb confirmation, neutral frames, forward/stop and reverse rejection, watchdog and emergency stop',
              flush=True)

        offset = len(lines())
        action.publish(String(data='liedown'))
        spin(0.35)
        assert 'StandUp' in lines()[offset:] and 'Crawl' not in lines()[offset:]
        event('walk')
        wait_for(lambda: 'LieDown' in lines()[offset:], 'LieDown chain failed')
        sequence = [line for line in lines()[offset:] if line in ('StandUp', 'Crawl', 'LieDown')]
        assert sequence == ['StandUp', 'Crawl', 'LieDown'], sequence
        print('PASS: chained lying-down waits for actual posture reports', flush=True)
        master, slave = pty.openpty()
        teleop_output = (output / 'teleop.log').open('w')
        teleop = subprocess.Popen([str(ws / 'install/d1_host_control/lib/d1_host_control/teleop_keyboard')],
                                  stdin=slave, stdout=teleop_output, stderr=subprocess.STDOUT)
        os.close(slave)
        spin(1.2)
        speeds.clear()
        os.write(master, b'w')
        wait_for(lambda: any(value > 0.1 for value in speeds), 'Keyboard w did not publish velocity')
        os.write(master, b'8')
        wait_for(lambda: 2 in poses, 'Keyboard 8 did not publish /d1_posture=2')
        speeds.clear()
        spin(0.25)
        assert speeds and all(abs(value) < 1e-6 for value in speeds), 'Posture key did not stop velocity'

        # Exercise the real keyboard -> ROS -> bridge -> SDK path in Climb.
        count = lines().count('Climb')
        os.write(master, b'v')
        wait_for(lambda: lines().count('Climb') == count + 1, 'Keyboard v did not request Climb')
        spin(0.25)
        offset = len(lines())
        os.write(master, b'w')
        spin(0.35)
        assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
            'Keyboard motion escaped Climb transition gate'
        os.write(master, b' ')
        spin(0.25)
        event('climb')
        spin(0.25)
        offset = len(lines())
        os.write(master, b'w')
        wait_for(lambda: any(fb > 0.29 for _, fb, _ in moves(offset)),
                 'Keyboard w did not reach SDK in Climb')
        offset = len(lines())
        os.write(master, b's')
        wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                 'Keyboard s did not stop in Climb')
        offset = len(lines())
        os.write(master, b' ')
        wait_for(lambda: any(all(abs(value) < 1e-6 for value in move) for move in moves(offset)),
                 'Keyboard space did not stop SDK in Climb')
        after_stop = len(lines())
        spin(0.4)
        observed = moves(after_stop)
        assert observed and all(abs(value) < 1e-6 for move in observed for value in move), \
            'Keyboard space did not hold Climb stopped'
        os.write(master, b'wswsws ')
        spin(0.25)
        after_burst = len(lines())
        spin(0.35)
        observed = moves(after_burst)
        assert observed and all(abs(value) < 1e-6 for move in observed for value in move), \
            'Older keyboard velocity resurrected after a rapid stop'
        os.write(master, b'w')
        wait_for(lambda: any(fb > 0.29 for _, fb, _ in moves(after_stop)),
                 'Cannot restart Climb after keyboard stop')
        count = lines().count('StandUp')
        os.write(master, b'7')
        wait_for(lambda: lines().count('StandUp') == count + 1, 'Keyboard 7 did not exit Climb')
        offset = len(lines())
        spin(0.3)
        assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
            'Motion continued after leaving Climb'
        print('PASS: keyboard v/w/s/space in Climb and leaving Climb while moving', flush=True)

        event('walk')
        spin(0.25)
        for key, pose_name, sdk_name in (
                (b'b', 'gait', 'Gait'), (b'n', 'slim', 'Slim'),
                (b'c', 'crawl_walk', 'CrawlWalk'), (b'm', 'dsb', 'DSB'),
                (b';', 'sand', 'Sand')):
            count = lines().count(sdk_name)
            os.write(master, key)
            wait_for(lambda: lines().count(sdk_name) == count + 1,
                     f'Keyboard {key!r} did not request {sdk_name}')
            spin(0.25)
            offset = len(lines())
            os.write(master, b'w')
            spin(0.35)
            assert not any(abs(value) > 1e-6 for move in moves(offset) for value in move), \
                f'Keyboard motion escaped {sdk_name} transition gate'
            os.write(master, b' ')
            spin(0.2)
            event(pose_name)
            spin(0.25)
            offset = len(lines())
            os.write(master, b'w')
            wait_for(lambda: any(fb > 0.29 for _, fb, _ in moves(offset)),
                     f'Keyboard w did not reach SDK in {sdk_name}')
            offset = len(lines())
            os.write(master, b's')
            wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                     f'Keyboard s did not stop in {sdk_name}')
            offset = len(lines())
            os.write(master, b'j')
            wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                     f'Disabled keyboard j did not stop in {sdk_name}')
            offset = len(lines())
            os.write(master, b'k')
            wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                     f'Disabled keyboard k did not stop in {sdk_name}')
            offset = len(lines())
            os.write(master, b'a')
            wait_for(lambda: any(abs(lr) < 1e-6 and abs(fb) < 1e-6 and yaw > 0.32
                                 for lr, fb, yaw in moves(offset)),
                     f'Lateral velocity remained after keyboard a in {sdk_name}')
            offset = len(lines())
            os.write(master, b'd')
            wait_for(lambda: any(abs(lr) < 1e-6 and abs(fb) < 1e-6 and yaw < -0.32
                                 for lr, fb, yaw in moves(offset)),
                     f'Keyboard d did not produce pure right turn in {sdk_name}')
            offset = len(lines())
            os.write(master, b'w')
            wait_for(lambda: any(abs(lr) < 1e-6 and fb > 0.29 and abs(yaw) < 1e-6
                                 for lr, fb, yaw in moves(offset)),
                     f'Old movement axes remained after keyboard w in {sdk_name}')
            offset = len(lines())
            os.write(master, b' ')
            wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                     f'Keyboard space did not stop SDK in {sdk_name}')
            after_stop = len(lines())
            spin(0.3)
            observed = moves(after_stop)
            assert observed and all(abs(v) < 1e-6 for move in observed for v in move), \
                f'Keyboard stop was not held in {sdk_name}'
            if pose_name in ('crawl_walk', 'dsb'):
                offset = len(lines())
                os.write(master, b'w')
                wait_for(lambda: any(fb > 0.29 for _, fb, _ in moves(offset)),
                         f'Cannot restart {sdk_name} before testing exit')
                count_stand = lines().count('StandUp')
                os.write(master, b'7')
                wait_for(lambda: lines().count('StandUp') == count_stand + 1,
                         f'Keyboard 7 did not exit {sdk_name}')
                offset = len(lines())
                spin(0.3)
                assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
                    f'Movement continued while leaving {sdk_name}'
                event('walk')
                spin(0.25)
            print(f'PASS: keyboard {key!r}/w/s/j/k/a/d/space in {sdk_name}', flush=True)

        # Leaving Sand must stop the held keyboard velocity and await Walk.
        offset = len(lines())
        os.write(master, b'w')
        wait_for(lambda: any(fb > 0.29 for _, fb, _ in moves(offset)),
                 'Keyboard w did not restart Sand')
        count = lines().count('StandUp')
        os.write(master, b'7')
        wait_for(lambda: lines().count('StandUp') == count + 1,
                 'Keyboard 7 did not exit Sand')
        offset = len(lines())
        spin(0.3)
        assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
            'Motion continued while leaving Sand'
        event('walk')
        spin(0.25)
        offset = len(lines())
        spin(0.25)
        assert all(abs(v) < 1e-6 for move in moves(offset) for v in move), \
            'Old Sand keyboard velocity resumed after Walk confirmation'
        print('PASS: exiting Sand stops held velocity and awaits Walk', flush=True)

        offset = len(lines())
        os.write(master, b'j')
        wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                 'Disabled keyboard j did not stop in ordinary Walk')
        offset = len(lines())
        os.write(master, b'k')
        wait_for(lambda: any(all(abs(v) < 1e-6 for v in move) for move in moves(offset)),
                 'Disabled keyboard k did not stop in ordinary Walk')
        os.write(master, b' ')
        spin(0.25)
        assert all(abs(v) < 1e-6 for v in moves()[-1]), 'Walk lateral stop failed'
        print('PASS: ordinary Walk rejects keyboard j/k lateral movement and stops', flush=True)

        # Sand uses the same confirmed lying-down chain as other moving poses.
        count = lines().count('Sand')
        action.publish(String(data='sand'))
        wait_for(lambda: lines().count('Sand') == count + 1, 'Sand re-entry was not sent')
        event('sand')
        spin(0.25)
        offset = len(lines())
        action.publish(String(data='liedown'))
        wait_for(lambda: 'StandUp' in lines()[offset:], 'Sand lying-down did not request StandUp')
        spin(0.25)
        assert 'Crawl' not in lines()[offset:], 'Sand lying-down skipped StandUp confirmation'
        event('walk')
        wait_for(lambda: 'LieDown' in lines()[offset:], 'Sand lying-down chain failed')
        sequence = [line for line in lines()[offset:] if line in ('StandUp', 'Crawl', 'LieDown')]
        assert sequence == ['StandUp', 'Crawl', 'LieDown'], sequence
        print('PASS: Sand lying-down waits for confirmed StandUp/Crawl/LieDown', flush=True)

        for key, pose_name, sdk_name in ((b'c', 'crawl_walk', 'CrawlWalk'), (b'm', 'dsb', 'DSB')):
            count = lines().count(sdk_name)
            os.write(master, key)
            wait_for(lambda: lines().count(sdk_name) == count + 1,
                     f'{sdk_name} not selected for lying-down test')
            event(pose_name)
            spin(0.25)
            offset = len(lines())
            os.write(master, b'f')
            wait_for(lambda: 'StandUp' in lines()[offset:],
                     f'{sdk_name} lying-down skipped the StandUp step')
            spin(0.25)
            assert 'Crawl' not in lines()[offset:], f'{sdk_name} did not wait for StandUp'
            event('walk')
            wait_for(lambda: 'LieDown' in lines()[offset:], f'{sdk_name} lying-down chain failed')
            sequence = [line for line in lines()[offset:] if line in ('StandUp', 'Crawl', 'LieDown')]
            assert sequence == ['StandUp', 'Crawl', 'LieDown'], sequence
        print('PASS: CrawlWalk and DSB lying-down retain the static Crawl confirmation chain', flush=True)

        count_crawl = lines().count('Crawl')
        os.write(master, b'C')
        wait_for(lambda: lines().count('Crawl') == count_crawl + 1,
                 'Shift+C did not select static Crawl')
        spin(0.25)
        offset = len(lines())
        os.write(master, b'j')
        spin(0.4)
        os.write(master, b'w')
        spin(0.4)
        assert not any(abs(v) > 1e-6 for move in moves(offset) for v in move), \
            'Static Crawl unexpectedly permits nonzero Move'
        os.write(master, b' ')
        spin(0.15)
        print('PASS: Shift+C preserves static Crawl and blocks movement', flush=True)

        stop(teleop)
        assert teleop.returncode == 0, f'Teleop shutdown exit={teleop.returncode}'
        os.close(master)
        teleop_output.close()
        print('PASS: keyboard PTY input/topics and clean shutdown', flush=True)
        # Shutdown from a moving Walk must send zero without latching soft stop.
        event('walk')
        spin(0.25)
        offset = len(lines())
        drive(0.65, vx=0.3)
        assert any(fb > 0.29 for _, fb, _ in moves(offset)), 'Pre-shutdown drive unavailable'
        soft_stops = lines().count('SoftEmergencyStop 1')
        offset = len(lines())
        stop(bridge)
        assert lines().count('SoftEmergencyStop 1') == soft_stops, 'Shutdown latched soft stop'
        assert moves(offset) and all(abs(v) < 1e-6 for v in moves(offset)[-1]), 'Shutdown omitted zero Move'
        assert all(lr == 0. and fb >= 0. for lr, fb, _ in moves()), 'Forbidden motion reached SDK'
        assert 'ReverseHeadTail' not in lines(), 'Head-tail reversal escaped navigation policy'
        assert bridge.returncode == 0, f'Bridge shutdown exit={bridge.returncode}'
        bridge_output.close()
        diagnostics = (output / 'bridge.log').read_text()
        assert 'd1-navigation-v6.5' in diagnostics and 'CTRL motion=' in diagnostics, \
            'Control diagnostics missing'
        assert 'SDK acknowledged Gait receipt' in diagnostics, 'Gait ACK was not observed'
        assert 'Requested Gait lost: actual=Walk' in diagnostics, 'Gait loss was not diagnosed'
        assert 'Pose confirmed: Sand' in diagnostics, 'Sand state confirmation missing'
        assert 'Pose confirmed: CrawlWalk' in diagnostics, 'CrawlWalk state confirmation missing'
        assert 'Pose confirmed: DSB' in diagnostics, 'DSB state confirmation missing'
        assert 'Move write #' not in diagnostics, 'Offline SDK reported an asynchronous write failure'
finally:
    stop(bridge)
    stop(teleop)
    node.destroy_node()
    rclpy.shutdown()
print('All offline control checks passed. No robot connection was made.', flush=True)
