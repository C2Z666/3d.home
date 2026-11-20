"""Exercise Walk reset against a socket-free SDK double in ROS domain 177."""
import json,os,signal,subprocess,sys,time,shutil,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
out=Path(tempfile.mkdtemp(prefix='walk_reset_test_',dir=ROOT/'work'))
fake=out/'libfake_robot_sdk.so';pkg=ROOT/'src/d1_host_control'
subprocess.run(['g++','-std=c++17','-shared','-fPIC','-pthread','-I'+str(pkg/'third_party/robot_sdk/include'),str(pkg/'test/fake_sdk.cpp'),'-o',str(fake)],check=True)
os.environ.update(ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('walk_reset_fixture');status={}
n.create_subscription(String,'/d1/sdk/d1_motion_status',lambda m:status.update(json.loads(m.data)),1)
action=n.create_publisher(String,'/d1/sdk/d1_action',1)
commands=out/'events';commands.write_text('');sdklog=out/'sdk.log';log=(out/'bridge.log').open('w')
env=dict(os.environ,LD_PRELOAD=str(fake),D1_TEST_SDK_LOG=str(sdklog),D1_TEST_SDK_COMMANDS=str(commands))
bridge=subprocess.Popen([str(ROOT/'install/d1_host_control/lib/d1_host_control/d1_bridge'),'--ros-args','-r','__ns:=/d1/sdk','-r','cmd_vel:=supervised_cmd_vel','-p','robot_ip:=127.0.0.1','-p','stair_max_lin_vel:=0.5','-p','stair_max_ang_vel:=0.05'],env=env,stdout=log,stderr=subprocess.STDOUT)
child=None;auto_ack=True;acked=set()
def spin():
    rclpy.spin_once(n,timeout_sec=.03)
    key=status.get('pose_request_id')
    if auto_ack and status.get('motion')=='StandUp' and status.get('motion_pending') and key not in acked:
        with commands.open('a') as f:f.write('walk\n')
        acked.add(key)
def wait(predicate,seconds=8):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        spin()
        if predicate():return
    raise AssertionError(status)
try:
    wait(lambda:status.get('control_held'))
    assert 'offline-test' in (out/'bridge.log').read_text()
    for mode in ['initial_lying','stair','crawl','standup','no_ack']:
        if mode not in ('initial_lying','no_ack'):
            before=status['pose_request_id'];action.publish(String(data=mode))
            wait(lambda:status.get('pose_request_id',0)>before and status.get('motion_pending') is False)
        folder=out/mode;shutil.copytree(ROOT/'scripts',folder/'scripts');shutil.copytree(ROOT/'config',folder/'config');(folder/'work').mkdir()
        (folder/'scripts/scene_registry.py').write_text("def scene_bundle(root,name):return {'map_id':'d1_hall_100832_hall_clean','maps':{'lower':{},'upper':{}}}\n")
        fs=dict(map_id='d1_hall_100832_hall_clean',actual_floor=None if mode=='stair' else 'floor1',active_nav_floor='floor1',phase='STAIR',connection_id='A',target_floor='floor2')
        (folder/'work/floor_state.json').write_text(json.dumps(fs))
        logfile=(folder/'reset.log').open('w')
        auto_ack=mode!='no_ack'
        child=subprocess.Popen([sys.executable,str(folder/'scripts/walk_reset.py'),'--owner-pid',str(os.getpid())],stdout=logfile,stderr=subprocess.STDOUT)
        wait(lambda:child.poll() is not None,30);logfile.close()
        if mode=='no_ack':
            assert child.returncode!=0
            assert json.loads((folder/'work/floor_state.json').read_text())==fs,'No acknowledgement must not reset floor state'
            print('PASS missing Walk confirmation fails closed',flush=True);child=None;continue
        assert child.returncode==0,(folder/'reset.log').read_text()
        wait(lambda:status.get('motion')=='Walk' and status.get('command_zero') is True)
        final=json.loads((folder/'work/floor_state.json').read_text())
        assert final['connection_id'] is None and final['target_floor'] is None
        assert final['actual_floor']==fs['actual_floor']
        assert final['phase']==('NEEDS_LOCALIZATION' if mode=='stair' else 'ON_FLOOR')
        print('PASS Walk reset from',mode,final['phase'],flush=True)
        child=None
finally:
    if child and child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=5)
    bridge.send_signal(signal.SIGINT)
    try:bridge.wait(timeout=5)
    except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
    log.close();n.destroy_node();rclpy.shutdown()
