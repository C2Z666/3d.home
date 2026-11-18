#!/usr/bin/env python3
"""Calibrated raw front LiDAR -> gravity-level Nav2 scan; no robot commands."""
import argparse,collections,copy,json,math,time
from pathlib import Path
import numpy as np,yaml
from geometry import calibration,imu_from_base,pose_matrix,planar,interpolate,transform,LIO_TO_NAV,navigation_pose
from cloud_pairing import CloudQueue
from clearing_evidence import ClearingEvidence,voxel_sample


def cloud_xyz(m):
 fields={f.name:f for f in m.fields}
 if any(k not in fields or fields[k].datatype!=7 or fields[k].count!=1 for k in ('x','y','z')):
  raise ValueError('expected scalar FLOAT32 x/y/z fields')
 dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if m.is_bigendian else '<')+'f4']*3,
  offsets=[fields[k].offset for k in ('x','y','z')],itemsize=m.point_step))
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
 return np.stack([a[k].reshape(-1) for k in ('x','y','z')],axis=1)

def obstacle_points(points,N_L,cfg,diagnostic=None):
 p=transform(N_L,points);d=np.linalg.norm(p[:,:2],axis=1)
 keep=np.isfinite(p).all(axis=1)&(d>=cfg['range_min_m'])&(d<=cfg['range_max_m'])
 lo=cfg['obstacle_min_above_floor_m']-cfg['base_height_m'];hi=cfg['obstacle_max_above_floor_m']-cfg['base_height_m']
 keep&=(p[:,2]>=lo)&(p[:,2]<=hi);p=p[keep];d=d[keep]
 # Reject isolated returns only in the lowest 3 cm of the obstacle band.
 # Higher points and supported small obstacles retain the original semantics.
 rejected=0
 if len(p) and cfg.get('floor_edge_support_filter',False):
  from scipy.spatial import cKDTree
  near_floor=p[:,2]<lo+.03
  retain=np.ones(len(p),dtype=bool)
  if near_floor.any():
   distances,_=cKDTree(p).query(p[near_floor],k=3,distance_upper_bound=.08)
   retain[near_floor]=np.isfinite(distances[:,2])
  rejected=int((~retain).sum());p=p[retain];d=d[retain]
 if diagnostic is not None:diagnostic.update(floor_edge_rejected=rejected,obstacle_points=len(p))
 return p


def project(points,N_L,cfg,diagnostic=None,return_points=False):
 p=obstacle_points(points,N_L,cfg,diagnostic);d=np.linalg.norm(p[:,:2],axis=1)
 bins=np.floor((np.arctan2(p[:,1],p[:,0])+math.pi)/(2*math.pi/720)).astype(int)%720
 out=np.full(720,np.inf);np.minimum.at(out,bins,d)
 return (out,p) if return_points else out

def main():
 root=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=root/'config/runtime.yaml');a=p.parse_args()
 c=yaml.safe_load(a.config.read_text(encoding='utf-8'))
 if not isinstance(c.get('cloud_frame'),str) or not c['cloud_frame']:p.error('set runtime.yaml cloud_frame to the observed raw front LiDAR frame')
 if not (0<c['base_height_m']<2 and 0<c['obstacle_min_above_floor_m']<c['obstacle_max_above_floor_m'] and 0<c['range_min_m']<c['range_max_m']):p.error('invalid projection limits')
 _,_,B_L,_=calibration(root/'config/calibration.yaml');I_B=imu_from_base(root/'config/calibration.yaml')
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import PointCloud2,LaserScan,PointField
 from geometry_msgs.msg import TransformStamped
 from tf2_ros import TransformBroadcaster
 from std_msgs.msg import String
 rclpy.init();n=rclpy.create_node('d1_cloud_scan');history=collections.deque(maxlen=100);pending=None;clouds=CloudQueue()
 clear_evidence=ClearingEvidence();sensor_tf=TransformBroadcaster(n)
 marked_pub=n.create_publisher(PointCloud2,'/d1/nav/obstacle_points',qos_profile_sensor_data)
 clear_pub=n.create_publisher(PointCloud2,'/d1/nav/clearing_points',qos_profile_sensor_data)
 def points_msg(points,header):
  data=np.asarray(points,dtype='<f4').reshape(-1,3);out=PointCloud2();out.header=copy.deepcopy(header);out.header.frame_id='nav_base'
  out.height=1;out.width=len(data);out.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
  out.is_bigendian=False;out.point_step=12;out.row_step=12*len(data);out.is_dense=True;out.data=data.tobytes();return out
 last=-math.inf;fault='waiting for calibrated cloud and odometry';points=0;projection_info={}
 pub=n.create_publisher(LaserScan,'/d1/nav/scan',qos_profile_sensor_data);health=n.create_publisher(String,'/d1/nav/scan_status',1)
 def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9
 def odom(m):
  nonlocal fault
  if m.header.frame_id!=c['odom_frame'] or m.child_frame_id!=c['body_frame']:fault='FAST-LIO frame mismatch';history.clear();return
  t=stamp(m.header)
  if abs(n.get_clock().now().nanoseconds*1e-9-t)>.5:return
  if history and t<=history[-1][0]:history.clear();clouds.clear();clear_evidence.clear()
  history.append((t,pose_matrix(m.pose.pose)@I_B))
 def cloud(m):
  clouds.add(stamp(m.header),m)
 def tick():
  nonlocal pending,last,fault,points
  if history:pending=clouds.take(n.get_clock().now().nanoseconds*1e-9,history[0][0],history[-1][0])
  if pending is not None:
   m=pending;t=stamp(m.header)
   if m.header.frame_id!=c['cloud_frame']:fault='raw cloud frame mismatch';pending=None
   elif abs(n.get_clock().now().nanoseconds*1e-9-t)>.5:fault='stale cloud';pending=None
   elif history and t<=history[-1][0]:
    pending=None
    try:
     O=interpolate(list(history),t)
     # FAST-LIO on this platform uses a Y-up world (static IMU acceleration is +Y).
     # Convert that world convention to the Z-up nav convention before leveling.
     W_B=navigation_pose(O)
     N_B=np.linalg.inv(planar(W_B))@W_B;N_B[:3,3]=0.0
     xyz=cloud_xyz(m);points=int(np.isfinite(xyz).all(axis=1).sum())
     if points<30:raise ValueError('too few finite raw points')
     # Keep the raw LiDAR axes and apply the calibrated base<-LiDAR extrinsic.
     ranges,obstacles=project(xyz,N_B@B_L,c,projection_info,return_points=True)
     N_L=N_B@B_L
     obstacles=voxel_sample(obstacles)
     free=clear_evidence.update(xyz,t,W_B@B_L)
     free=transform(N_L,free)
     tf=TransformStamped();tf.header.stamp=m.header.stamp;tf.header.frame_id='nav_base';tf.child_frame_id='d1_nav_lidar_origin'
     tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,N_L[:3,3]);tf.transform.rotation.w=1.
     sensor_tf.sendTransform(tf)
     marked_pub.publish(points_msg(obstacles,m.header));clear_pub.publish(points_msg(free,m.header))
     projection_info.update(marking_points=len(obstacles),clearing_points=len(free),clearing_state=clear_evidence.reason)
     # Bounded paired evidence at exactly the cloud acquisition time.
     body_points=transform(N_B@B_L,xyz)
     selected=body_points[np.isfinite(body_points).all(axis=1)&(body_points[:,0]>.3)&(body_points[:,0]<2.)&(np.abs(body_points[:,1])<.6)]
     selected=selected[(selected[:,2]>-.6)&(selected[:,2]<.3)]
     sample=selected[::max(1,len(selected)//24)][:24]
     paired=transform(planar(W_B),sample)
     projection_info.update(source_stamp=t,near_front_count=len(selected),near_front_base=sample.round(4).tolist(),near_front_odom=paired.round(4).tolist(),assumed_base_height_m=c['base_height_m'])
     out=LaserScan();out.header.stamp=m.header.stamp;out.header.frame_id='nav_base'
     out.angle_min=-math.pi;out.angle_increment=2*math.pi/720;out.angle_max=math.pi-out.angle_increment
     out.range_min=float(c['range_min_m']);out.range_max=float(c['range_max_m']);out.ranges=ranges.tolist();pub.publish(out)
     last=time.monotonic();fault=''
    except Exception as e:fault=str(e)
  health.publish(String(data=json.dumps(dict(ready=time.monotonic()-last<.4,reason=fault,finite_input_points=points,**projection_info))))
 n.create_subscription(Odometry,c['odom_topic'],odom,qos_profile_sensor_data);n.create_subscription(PointCloud2,c['cloud_topic'],cloud,qos_profile_sensor_data);n.create_timer(.04,tick)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
