#!/usr/bin/env python3
"""Calibrated raw front LiDAR -> gravity-level Nav2 scan; no robot commands."""
import argparse,collections,json,math,time
from pathlib import Path
import numpy as np,yaml
from geometry import calibration,imu_from_base,pose_matrix,planar,interpolate,transform

# Measured FAST-LIO world (Y-up) -> nav convention (Z-up): nav_x=-lio_z, nav_y=-lio_x, nav_z=lio_y.
LIO_TO_NAV=np.array([[0.0,0.0,-1.0],[-1.0,0.0,0.0],[0.0,1.0,0.0]])

def cloud_xyz(m):
 fields={f.name:f for f in m.fields}
 if any(k not in fields or fields[k].datatype!=7 or fields[k].count!=1 for k in ('x','y','z')):
  raise ValueError('expected scalar FLOAT32 x/y/z fields')
 dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if m.is_bigendian else '<')+'f4']*3,
  offsets=[fields[k].offset for k in ('x','y','z')],itemsize=m.point_step))
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
 return np.stack([a[k].reshape(-1) for k in ('x','y','z')],axis=1)

def project(points,N_L,cfg):
 p=transform(N_L,points);d=np.linalg.norm(p[:,:2],axis=1)
 keep=np.isfinite(p).all(axis=1)&(d>=cfg['range_min_m'])&(d<=cfg['range_max_m'])
 lo=cfg['obstacle_min_above_floor_m']-cfg['base_height_m'];hi=cfg['obstacle_max_above_floor_m']-cfg['base_height_m']
 keep&=(p[:,2]>=lo)&(p[:,2]<=hi);p=p[keep];d=d[keep]
 bins=np.floor((np.arctan2(p[:,1],p[:,0])+math.pi)/(2*math.pi/720)).astype(int)%720
 out=np.full(720,np.inf);np.minimum.at(out,bins,d)
 return out

def main():
 root=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=root/'config/runtime.yaml');a=p.parse_args()
 c=yaml.safe_load(a.config.read_text(encoding='utf-8'))
 if not isinstance(c.get('cloud_frame'),str) or not c['cloud_frame']:p.error('set runtime.yaml cloud_frame to the observed raw front LiDAR frame')
 if not (0<c['base_height_m']<2 and 0<c['obstacle_min_above_floor_m']<c['obstacle_max_above_floor_m'] and 0<c['range_min_m']<c['range_max_m']):p.error('invalid projection limits')
 _,_,B_L,_=calibration(root/'config/calibration.yaml');I_B=imu_from_base(root/'config/calibration.yaml')
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import PointCloud2,LaserScan
 from std_msgs.msg import String
 rclpy.init();n=rclpy.create_node('d1_cloud_scan');history=collections.deque(maxlen=100);pending=None
 last=-math.inf;fault='waiting for calibrated cloud and odometry';points=0
 pub=n.create_publisher(LaserScan,'/d1/nav/scan',qos_profile_sensor_data);health=n.create_publisher(String,'/d1/nav/scan_status',1)
 def stamp(h):return h.stamp.sec+h.stamp.nanosec*1e-9
 def odom(m):
  nonlocal fault
  if m.header.frame_id!=c['odom_frame'] or m.child_frame_id!=c['body_frame']:fault='FAST-LIO frame mismatch';history.clear();return
  t=stamp(m.header)
  if abs(n.get_clock().now().nanoseconds*1e-9-t)>.5:return
  if history and t<=history[-1][0]:history.clear()
  history.append((t,pose_matrix(m.pose.pose)@I_B))
 def cloud(m):
  nonlocal pending
  pending=m
 def tick():
  nonlocal pending,last,fault,points
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
     W_B=np.eye(4);W_B[:3,:3]=LIO_TO_NAV@O[:3,:3]
     W_B[:3,3]=LIO_TO_NAV@O[:3,3]
     N_B=np.linalg.inv(planar(W_B))@W_B;N_B[:3,3]=0.0
     xyz=cloud_xyz(m);points=int(np.isfinite(xyz).all(axis=1).sum())
     if points<30:raise ValueError('too few finite raw points')
     # Keep the raw LiDAR axes and apply the calibrated base<-LiDAR extrinsic.
     ranges=project(xyz,N_B@B_L,c)
     out=LaserScan();out.header.stamp=m.header.stamp;out.header.frame_id='nav_base'
     out.angle_min=-math.pi;out.angle_increment=2*math.pi/720;out.angle_max=math.pi-out.angle_increment
     out.range_min=float(c['range_min_m']);out.range_max=float(c['range_max_m']);out.ranges=ranges.tolist();pub.publish(out)
     last=time.monotonic();fault=''
    except Exception as e:fault=str(e)
  health.publish(String(data=json.dumps(dict(ready=time.monotonic()-last<.4,reason=fault,finite_input_points=points))))
 n.create_subscription(Odometry,c['odom_topic'],odom,qos_profile_sensor_data);n.create_subscription(PointCloud2,c['cloud_topic'],cloud,qos_profile_sensor_data);n.create_timer(.04,tick)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
