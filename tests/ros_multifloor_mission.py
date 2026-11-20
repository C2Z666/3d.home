"""Exercise the real mission process on synthetic ROS data, with no SDK sockets."""
import fcntl,json,math,os,shutil,signal,subprocess,tempfile,time
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
os.environ.update(ROS_DOMAIN_ID='179',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import sys
sys.path.insert(0,str(ROOT/'scripts'))
from geometry import imu_from_base,LIO_TO_NAV
from floor_routes import load_registry
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String,Bool
from rclpy.qos import qos_profile_sensor_data


def main():
    cancel_test='--cancel' in sys.argv
    drift_test='--yaw-drift' in sys.argv
    stall_test='--raw-stall' in sys.argv
    jump_test='--raw-jump' in sys.argv
    fault_test=stall_test or jump_test
    folder=Path(tempfile.mkdtemp(prefix='mission_sim_',dir=ROOT/'work'))
    shutil.copytree(ROOT/'scripts',folder/'scripts');shutil.copytree(ROOT/'config',folder/'config')
    (folder/'work/navigation').mkdir(parents=True)
    config=yaml.safe_load((folder/'config/floor_connections.yaml').read_text())
    c=config['connections'][0];c['entry']=dict(x=0.,y=0.,yaw=0.)
    c['exit']=None if '--unmarked' in sys.argv else dict(floor_id='floor2',x=.7,y=0.,yaw=0.)
    c['preview_exit']=c['exit']
    c['arrival_line']=None if '--unmarked' in sys.argv else dict(floor_id='floor2',x=.7,y=0.,yaw=0.,half_width_m=.6)
    c['completion'].update(min_rise_m=.2,min_forward_m=.2,stable_sec=.3,level_travel_m=.1)
    (folder/'config/floor_connections.yaml').write_text(yaml.safe_dump(config))
    (folder/'scripts/scene_registry.py').write_text("def scene_bundle(root,name):return {'map_id':'d1_hall_100832_hall_clean','maps':{'lower':{},'upper':{}}}\n")
    (folder/'scripts/site_identity.py').write_text("def verify(root):return 'd1_hall_100832_hall_clean'\n")
    (folder/'scripts/floor_map_control.py').write_text("from floor_routes import write_state\ndef activate_maps(node,root,scene,registry,fid):write_state(root,registry,active_nav_floor=fid)\n")
    (folder/'scripts/navigate_to_goal.py').write_text("import json,sys,time\nfrom pathlib import Path\na=sys.argv[1:];d={k:a[i+1] for i,k in enumerate(a) if k.startswith('--') and k!='--live'}\np=Path(__file__).resolve().parents[1]/'work/flat_calls.jsonl'\nwith p.open('a') as f:f.write(json.dumps(d)+'\\n')\ntime.sleep(.2)\n")
    registry=load_registry(folder,dict(map_id=config['map_id'],maps={'lower':{},'upper':{}}))
    plan=dict(scene='old',floor='lower',map_id=config['map_id'],localization_session='test',localization_epoch=0,
              start=dict(x=0.,y=0.,yaw=0.),goal=dict(x=2.,y=0.,yaw=0.),
              mission=dict(source_floor='floor1',target_floor='floor2',connections=[c],registry_revision=registry['revision'],transition_only=False))
    route=folder/'work/navigation/test.path.json';route.write_text(json.dumps(dict(plan=plan,poses=[c['entry']])))
    rclpy.init();n=rclpy.create_node('isolated_multifloor_simulation')
    pubs={k:n.create_publisher(t,topic,q) for k,t,topic,q in [
        ('sdk',String,'/d1/sdk/d1_motion_status',1),('loc',String,'/d1/localization/status',1),
        ('pose',PoseStamped,'/d1/localization/pose6d',10),('raw',Odometry,'/Odometry',qos_profile_sensor_data)]}
    state=dict(motion='Walk',motion_pending=False,pose_request_id=0,pose_confirmed_id=0,
               state_sequence=0,safety_epoch=0,bridge_session='fake',sdk_connected=True,control_held=True,
               state_fresh=True,soft_stop=False,hardware_stop=False,command_zero=True,
               reported_forward_speed=0.,reported_lateral_speed=0.,reported_yaw_rate=0.,stair_speed_cap=.5,stair_yaw_rate_cap=.05,stair_fault=False)
    commanded=0.;turn=0.;turn_commands=[];command_at=permit_at=-math.inf;allowed=False;actions=[]
    def action(m):
        actions.append(m.data);state['motion']='Stair' if m.data=='stair' else 'Walk'
        state['pose_request_id']+=1;state['pose_confirmed_id']=state['pose_request_id']
    def velocity(m):
        nonlocal commanded,command_at,turn
        commanded=m.linear.x;command_at=time.monotonic()
        turn=m.angular.z;turn_commands.append(turn)
        assert abs(turn)<=.05+1e-9,'Stair turn exceeds cap'
        if not drift_test:assert abs(turn)<1e-7,'No heading error should not turn'
    def permission(m):
        nonlocal allowed,permit_at
        allowed=m.data;permit_at=time.monotonic()
    n.create_subscription(String,'/d1/sdk/d1_action',action,1)
    n.create_subscription(Twist,'/d1/sdk/stair_cmd_vel',velocity,1)
    n.create_subscription(Bool,'/d1/sdk/stair_motion_permit',permission,1)
    log=(folder/'mission.log').open('w')
    proc=subprocess.Popen([sys.executable,str(folder/'scripts/multifloor_mission.py'),'--live','--path-file',str(route),'--max-runtime','30','--owner-pid',str(os.getpid())],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
    C=np.eye(4);C[:3,:3]=LIO_TO_NAV;I_B=imu_from_base(folder/'config/calibration.yaml')
    x=0.;z=0.;yaw=0.;injected=False;frozen_stamp=None;previous=time.monotonic();saw_locked=False;cancel_sent=False
    try:
        end=time.monotonic()+35
        while proc.poll() is None and time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic();dt=min(.05,now-previous);previous=now
            speed=commanded if allowed and now-permit_at<.3 and now-command_at<.3 and state['motion']=='Stair' else 0.
            x+=speed*dt;z=min(.25,x*.5)
            if speed>0:yaw+=turn*dt
            if drift_test and x>.1 and not injected:yaw+=.12;injected=True
            if cancel_test and x>.05 and not cancel_sent:
                proc.send_signal(signal.SIGINT);cancel_sent=True
            stamp=n.get_clock().now().to_msg();T=np.eye(4);T[:3,3]=[x+(1. if jump_test and x>.1 else 0.),0,z];T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
            raw=np.linalg.inv(C)@T@np.linalg.inv(I_B);q=Rotation.from_matrix(raw[:3,:3]).as_quat()
            od=Odometry();od.header.frame_id='d1_lio_init';od.child_frame_id='d1_imu';od.header.stamp=stamp
            if stall_test and x>.1:
                if frozen_stamp is None:frozen_stamp=stamp
                od.header.stamp=frozen_stamp
            od.pose.pose.position.x,od.pose.pose.position.y,od.pose.pose.position.z=map(float,raw[:3,3])
            od.pose.pose.orientation.x,od.pose.pose.orientation.y,od.pose.pose.orientation.z,od.pose.pose.orientation.w=map(float,q)
            pubs['raw'].publish(od)
            ps=PoseStamped();ps.header.frame_id='map';ps.header.stamp=stamp;ps.pose.position.x=x;ps.pose.position.z=z;ps.pose.orientation.z=math.sin(yaw/2);ps.pose.orientation.w=math.cos(yaw/2);pubs['pose'].publish(ps)
            state['state_sequence']+=1;state['command_zero']=speed==0.;state['reported_forward_speed']=speed;state['reported_yaw_rate']=turn if speed else 0.
            pubs['sdk'].publish(String(data=json.dumps(state)))
            pubs['loc'].publish(String(data=json.dumps(dict(session='test',epoch=0,map_id=config['map_id'],ready=True,tracking_valid=True))))
            if (folder/'work/navigation_mission.lock').exists():
                with (folder/'work/navigation_mission.lock').open('a') as lock:
                    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    except BlockingIOError:saw_locked=True
            time.sleep(.025)
        if proc.poll() is None:raise AssertionError('mission timeout')
        log.flush();assert proc.returncode==(130 if cancel_test else 1 if fault_test else 0),(folder/'mission.log').read_text()
        if drift_test:assert min(turn_commands)<-.01 and abs(yaw)<.12,(yaw,turn_commands)
        calls=[json.loads(line) for line in (folder/'work/flat_calls.jsonl').read_text().splitlines()]
        assert [c['--floor'] for c in calls]==(['floor1'] if cancel_test or fault_test else ['floor1','floor2']),calls
        assert actions==(['stair'] if cancel_test or fault_test else ['stair','standup']),actions
        final=json.loads((folder/'work/floor_state.json').read_text())
        if fault_test:assert final['actual_floor'] is None and final['phase']=='FAULT',final
        elif cancel_test:assert final['actual_floor'] is None and final['phase']=='CANCELED',final
        else:assert final['actual_floor']=='floor2' and final['active_nav_floor']=='floor2' and final['phase']=='ON_FLOOR',final
        assert saw_locked
        print('PASS:', 'fault stop without Walk transition' if fault_test else 'operator cancellation' if cancel_test else 'heading correction and landing' if drift_test else 'floor1 approach -> Stair -> landing stop -> Walk -> floor2 goal')
        print('LOG',folder/'mission.log')
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
