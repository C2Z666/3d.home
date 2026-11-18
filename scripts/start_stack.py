#!/usr/bin/env python3
"""Idempotent service startup; never publishes movement or posture requests."""
import argparse, fcntl, json, os, subprocess, sys, time, urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT / 'work'

def processes():
    result = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            if p.joinpath('stat').read_text().split()[2] == 'Z':
                continue
            argv = p.joinpath('cmdline').read_bytes().decode().split('\0')
            result.append((int(p.name), argv))
        except (OSError, UnicodeError, IndexError):
            pass
    return result

def ensure(name, marker, args, conflict=None):
    existing = [(pid, argv) for pid, argv in processes() if marker in argv]
    if len(existing) > 1:
        raise RuntimeError(f'{name} 有多个实例，请检查：{existing}')
    if existing:
        print(f'[REUSE] {name}: PID {existing[0][0]}', flush=True)
        return existing[0][0]
    if conflict:
        others = [(pid, argv) for pid, argv in processes() if any(Path(a).name == conflict for a in argv[:2])]
        if others:
            raise RuntimeError(f'{name} 存在其他实例，未自动停止或覆盖：{others}')
    logfile = WORK / (name + '.log')
    with logfile.open('ab', buffering=0) as log:
        log.write(('\n[START_STACK] ' + time.strftime('%F %T') + '\n').encode())
        proc = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.DEVNULL,
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (WORK / (name + '.pid')).write_text(str(proc.pid) + '\n')
    time.sleep(.7)
    if proc.poll() is not None:
        raise RuntimeError(f'{name} 启动退出，详见 {logfile}')
    print(f'[STARTED] {name}: PID {proc.pid}; {logfile}', flush=True)
    return proc.pid

def read_web(endpoint):
    with urllib.request.urlopen('http://127.0.0.1:8767/' + endpoint, timeout=2) as response:
        return json.load(response)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--status', action='store_true', help='仅检查，不启动服务')
    parser.add_argument('--wait', type=float, default=60, help='等待数据就绪的秒数，默认60')
    parser.add_argument('--robot-ip', default='192.168.168.168')
    args = parser.parse_args()
    if not 0 < args.wait <= 300:
        parser.error('--wait must be in (0, 300]')
    WORK.mkdir(exist_ok=True)
    lock = (WORK / 'start_stack.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2, Imu, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    rclpy.init()
    node = rclpy.create_node('d1_stack_start_check')
    received = {}; values = {}; clients = {}; pending = {}; life = {}
    preview_client = ActionClient(node, ComputePathToPose, '/d1/preview/compute_path_to_pose')
    topics = {
        'camera': ('/front_camera/image', Image), 'lidar': ('/front_lidar', PointCloud2),
        'imu': ('/front_lidar/imu', Imu), 'fastlio': ('/Odometry', Odometry),
        'localization': ('/d1/localization/status', String), 'scan': ('/d1/nav/scan', LaserScan),
        'scan_health': ('/d1/nav/scan_status', String), 'sdk': ('/d1/sdk/d1_motion_status', String)}
    def callback(key, message):
        received[key] = time.monotonic()
        if hasattr(message, 'data') and isinstance(message.data, str):
            try:
                values[key] = json.loads(message.data)
            except ValueError:
                pass
    for key, (topic, typ) in topics.items():
        node.create_subscription(typ, topic, lambda m, k=key: callback(k, m), qos_profile_sensor_data)
    for name in ('map_server', 'planner_server', 'controller_server', 'velocity_smoother', 'bt_navigator', 'd1/preview/map_server', 'd1/preview/planner_server'):
        clients[name] = node.create_client(GetState, '/' + name + '/get_state')
    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    try:
        spin_for(2)
        if not args.status:
            # A locked mission or another velocity source must not be disturbed.
            mission = (WORK / 'navigation_mission.lock').open('a')
            fcntl.flock(mission, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if node.get_publishers_info_by_topic('/d1/sdk/supervised_cmd_vel'):
                raise RuntimeError('存在速度发布者，请先结束当前控制任务，再启动服务')
            for key in ('camera', 'lidar', 'imu'):
                if time.monotonic() - received.get(key, -1e9) > 2:
                    raise RuntimeError('底层传感器未输出：' + topics[key][0] + '；请检查机器狗原厂传感器服务')
            for key in ('fastlio', 'localization', 'scan', 'sdk'):
                if len(node.get_publishers_info_by_topic(topics[key][0])) > 1:
                    raise RuntimeError('重复发布者：' + topics[key][0])
            script = lambda name: ['bash', str(ROOT / 'scripts' / name)]
            ensure('web_observation', str(ROOT / 'scripts/web_view.py'), script('run_web.sh') + ['--host', '0.0.0.0', '--port', '8767', '--scene', 'old'], 'web_view.py')
            bridge = str(ROOT / 'install/d1_host_control/lib/d1_host_control/d1_bridge')
            ensure('navigation_bridge', bridge, script('run_nav_bridge.sh') + [args.robot_ip], 'd1_bridge')
            ensure('fastlio', str(ROOT / 'config/fastlio_live.yaml'), script('run_fastlio.sh'), 'fastlio_mapping')
            ensure('localization', str(ROOT / 'scripts/localization_live.py'), script('run_localization.sh'), 'localization_live.py')
            ensure('cloud_scan', str(ROOT / 'scripts/cloud_scan.py'), script('run_cloud_scan.sh'), 'cloud_scan.py')
            ensure('planner_preview', str(ROOT / 'scripts/planner_preview.launch.py'), script('run_planner_preview.sh') + ['scene:=old'])
            ensure('navigation_nav2', str(ROOT / 'scripts/nav2_bringup.launch.py'), script('run_nav2.sh') + ['floor:=lower'], 'controller_server')
            mission.close()
        end = time.monotonic() + args.wait; next_print = 0
        while time.monotonic() < end:
            spin_for(.2)
            for name, client in clients.items():
                future = pending.get(name)
                if future is not None and future.done():
                    try:
                        life[name] = (future.result().current_state.id, time.monotonic())
                    except Exception:
                        life[name] = (0, time.monotonic())
                    pending.pop(name)
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetState.Request())
            now = time.monotonic()
            missing = [key for key in topics if now - received.get(key, -1e9) > 1.5]
            inactive = [name for name in clients if life.get(name, (0, 0))[0] != 3 or now - life.get(name, (0, 0))[1] > 3]
            web = {}; web_error = ''
            if now >= next_print:
                try:
                    web = read_web('state?scene=old')
                except Exception as e:
                    web_error = str(e)
                costmap_ok = False; costmap_error = ''
                try:
                    grid = read_web('preview-costmap?scene=old&floor=lower')
                    costmap_ok = grid.get('ok') is True and grid.get('age_sec', 99) < 2
                except Exception as e:
                    costmap_error = str(e)
                preview_ready = preview_client.server_is_ready()
                loc = values.get('localization', {}); sdk = values.get('sdk', {})
                sdk_ok = (all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh','command_zero'))
                          and all(sdk.get(k) is False for k in ('soft_stop','hardware_stop','motion_pending'))
                          and isinstance(sdk.get('state_age_sec'), (int, float)) and 0 <= sdk['state_age_sec'] < 2)
                duplicates = [topics[key][0] for key in ('fastlio','localization','scan','sdk') if len(node.get_publishers_info_by_topic(topics[key][0])) > 1]
                sensor_duplicates = [topics[key][0] for key in ('camera','lidar','imu') if len(node.get_publishers_info_by_topic(topics[key][0])) > 1]
                snapshot = dict(missing=missing, inactive_nav2=inactive, sdk_ready=sdk_ok, duplicate_topics=duplicates, sensor_duplicate_topics=sensor_duplicates,
                    localization=loc.get('reason'), mode=loc.get('mode'),
                    visual={k:loc.get('visual',{}).get(k) for k in ('reason','matches','inliers','accepted')},
                    scan=values.get('scan_health', {}),
                    sdk={k:sdk.get(k) for k in ('sdk_connected','control_held','state_fresh','command_zero','motion')},
                    preview_action_ready=preview_ready, preview_costmap_ready=costmap_ok, preview_costmap_error=costmap_error,
                    webpage_pose=web.get('pose_current'), navigation_ready=web.get('navigation_ready'), web_error=web_error)
                print('[HEALTH] ' + json.dumps(snapshot, ensure_ascii=False), flush=True)
                (WORK / 'start_stack_status.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
                next_print = now + 5
                if not missing and not inactive and not duplicates and sdk_ok and preview_ready and costmap_ok and web.get('pose_current') and web.get('navigation_ready'):
                    print('[READY] http://192.168.168.100:8767/?scene=old；服务就绪，未发送导航目标或姿态命令。', flush=True)
                    return 0
        print('[NOT_READY] 服务已保留运行，但就绪检查未通过；见 work/start_stack_status.json 和各服务日志。没有自动重定位或发送运动指令。', flush=True)
        return 2
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, BlockingIOError, OSError) as error:
        print('[START_FAILED] ' + str(error), file=sys.stderr, flush=True)
        raise SystemExit(1)
