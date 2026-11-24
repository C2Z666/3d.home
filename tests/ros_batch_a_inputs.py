"""Isolated ROS smoke: synthetic input + planner costmap; NO motion publisher/SDK."""
import os
os.environ.update(ROS_DOMAIN_ID='187',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import json,signal,subprocess,sys,tempfile,time
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_policy import configure_costmaps
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import GetCostmap


def main():
    config=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
    rclpy.init();node=rclpy.create_node('batch_a_input_smoke')
    pubs=[node.create_publisher(Odometry,config['odom_topic'],qos_profile_sensor_data),
          node.create_publisher(PointCloud2,config['cloud_topic'],qos_profile_sensor_data)]
    scans=[];health=[];delay=.53;next_cloud=0.;enabled=True
    node.create_subscription(LaserScan,'/d1/nav/scan',lambda m:scans.append((time.monotonic(),m)),qos_profile_sensor_data)
    node.create_subscription(String,'/d1/nav/scan_status',lambda m:health.append((time.monotonic(),json.loads(m.data))),1)
    tf=StaticTransformBroadcaster(node);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='nav_base';t.transform.rotation.w=1.;tf.sendTransform(t)
    def stamp(seconds):
        from builtin_interfaces.msg import Time
        ns=int(seconds*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
    def step():
        nonlocal next_cloud
        now=node.get_clock().now().nanoseconds*1e-9
        od=Odometry();od.header.frame_id=config['odom_frame'];od.child_frame_id=config['body_frame'];od.header.stamp=stamp(now);od.pose.pose.orientation.w=1.;pubs[0].publish(od)
        if enabled and time.monotonic()>=next_cloud:
            next_cloud=time.monotonic()+.1
            cloud=PointCloud2();cloud.header.frame_id=config['cloud_frame'];cloud.header.stamp=stamp(now-delay)
            cloud.height=1;cloud.width=64;cloud.point_step=12;cloud.row_step=768;cloud.is_dense=True
            cloud.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')]
            cloud.data=np.tile(np.array([[1.,.1,.1]],dtype='<f4'),(64,1)).tobytes();pubs[1].publish(cloud)
        rclpy.spin_once(node,timeout_sec=.01)
    def until(predicate,timeout=12):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            step()
            if predicate():return
        raise AssertionError('timed out waiting for smoke condition')
    def duration(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:step()
    def lifecycle(transition):
        client=node.create_client(ChangeState,'/planner_server/change_state')
        until(client.service_is_ready)
        req=ChangeState.Request();req.transition.id=transition
        future=client.call_async(req);until(future.done)
        assert future.result().success,transition
    processes=[]
    with tempfile.TemporaryDirectory(prefix='batch_a_ros_') as temp:
        logs=[]
        try:
            log=open(Path(temp)/'cloud.log','w');logs.append(log)
            processes.append(subprocess.Popen([sys.executable,str(ROOT/'scripts/cloud_scan.py')],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy()))
            until(lambda:len(scans)>=4 and health and health[-1][1]['ready'])
            timing=health[-1][1]['scan_timing']
            age=(timing['publish_ns']-timing['source_ns'])/1e9
            assert .50<age<.60,age
            print('PASS: cloud producer publishes acquisition-aged 500..600ms input',flush=True)
            cfg=configure_costmaps(yaml.safe_load((ROOT/'config/nav2.yaml').read_text()))
            cost=cfg['global_costmap']['global_costmap']['ros__parameters']
            cost.update(plugins=['obstacle_layer','inflation_layer'],width=4,height=4,rolling_window=True,track_unknown_space=False)
            params=Path(temp)/'planner.yaml';params.write_text(yaml.safe_dump(cfg))
            log=open(Path(temp)/'planner.log','w');logs.append(log)
            processes.append(subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy()))
            lifecycle(1);lifecycle(3)
            client=node.create_client(GetCostmap,'/global_costmap/get_costmap');until(client.service_is_ready)
            future=client.call_async(GetCostmap.Request());until(future.done)
            assert future.result().map.metadata.size_x>0
            print('PASS: modified temporal plugin configures/activates with shared 600ms parameters',flush=True)
            delay=.7;duration(1.)
            assert health and not health[-1][1]['ready']
            assert time.monotonic()-scans[-1][0]>.6
            print('PASS: stale source stops output and withdraws ready',flush=True)
            delay=.1;until(lambda:health[-1][1]['ready'] and time.monotonic()-scans[-1][0]<.2)
            enabled=False;duration(.8)
            assert not health[-1][1]['ready']
            print('PASS: fresh input recovers; dropout cannot refresh source timestamp',flush=True)
            lifecycle(4);lifecycle(2)
        except Exception:
            for log in logs:log.flush()
            for path in Path(temp).glob('*.log'):print(path.name,path.read_text()[-6000:])
            raise
        finally:
            for proc in processes:
                if proc.poll() is None:proc.send_signal(signal.SIGINT)
            for proc in processes:
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
            for log in logs:log.close()
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
