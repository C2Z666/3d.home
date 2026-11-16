#!/usr/bin/env python3
"""Read sensor interfaces only. Never publishes movement or posture commands."""
import argparse,json,math,time
from pathlib import Path
import yaml

def main():
 r=Path(__file__).resolve().parents[1];p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=8)
 p.add_argument('--write-frames',action='store_true',help='record observed frames; topic must be raw front LiDAR, Odometry must be IMU-referenced')
 a=p.parse_args();path=r/'config/runtime.yaml';c=yaml.safe_load(path.read_text())
 import rclpy
 from rclpy.qos import qos_profile_sensor_data
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import Image,PointCloud2
 from std_msgs.msg import String
 rclpy.init();n=rclpy.create_node('d1_phase3_preflight');seen={};counts={}
 def record(key,m):
  counts[key]=counts.get(key,0)+1
  d=dict(frame=m.header.frame_id,age_sec=n.get_clock().now().nanoseconds*1e-9-m.header.stamp.sec-m.header.stamp.nanosec*1e-9)
  if key=='odom':d['child_frame']=m.child_frame_id
  if key=='image':d.update(width=m.width,height=m.height,encoding=m.encoding)
  if key=='cloud':d.update(points=m.width*m.height,fields=[f.name for f in m.fields])
  seen[key]=d
 for key,typ,topic in [('odom',Odometry,c['odom_topic']),('image',Image,c['image_topic']),('cloud',PointCloud2,c['cloud_topic'])]:
  n.create_subscription(typ,topic,lambda m,k=key:record(k,m),qos_profile_sensor_data)
 def status(key,m):
  try:seen[key]=json.loads(m.data)
  except ValueError:pass
 for key,topic in [('localization','/d1/localization/status'),('scan','/d1/nav/scan_status'),('sdk','/d1/sdk/d1_motion_status')]:
  n.create_subscription(String,topic,lambda m,k=key:status(k,m),1)
 end=time.monotonic()+a.seconds
 try:
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 finally:n.destroy_node();rclpy.try_shutdown()
 report=dict(observed=seen,counts=counts,required_topics={k:c[k] for k in ('odom_topic','image_topic','cloud_topic')})
 print(json.dumps(report,indent=2,ensure_ascii=False))
 (r/'work').mkdir(exist_ok=True);(r/'work/preflight.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
 if a.write_frames:
  if any(k not in seen or counts.get(k,0)<2 or abs(seen[k]['age_sec'])>.5 for k in ('odom','image','cloud')):raise SystemExit('No fresh odometry/image/cloud; runtime configuration not changed')
  im=seen['image']
  if (im['width'],im['height'])!=(1920,1080) or im['encoding'] not in ('rgb8','bgr8'):raise SystemExit('Image calibration/encoding mismatch')
  c.update(odom_frame=seen['odom']['frame'],body_frame=seen['odom']['child_frame'],cloud_frame=seen['cloud']['frame'])
  path.with_suffix('.yaml.bak').write_bytes(path.read_bytes());path.write_text(yaml.safe_dump(c,sort_keys=False))
  print('Recorded observed frames. Confirm IMU pose reference and raw LiDAR coordinates before navigation.')
if __name__=='__main__':main()
