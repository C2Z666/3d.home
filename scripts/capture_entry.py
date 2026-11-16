#!/usr/bin/env python3
"""Save a measured, operator-identified stair approach pose into site.yaml."""
import argparse,collections,json,math,time
from pathlib import Path
import yaml

def main():
 p=argparse.ArgumentParser(); p.add_argument('--confirm-lower-entry',action='store_true',required=True)
 args=p.parse_args()
 from site_identity import verify
 map_id=verify(Path(__file__).resolve().parents[1])
 import rclpy
 from rclpy.node import Node
 from geometry_msgs.msg import PoseStamped
 from std_msgs.msg import String
 rclpy.init(); n=Node('d1_capture_stair_entry'); pose=None; health={}; health_at=pose_at=-math.inf
 def onpose(m):
  nonlocal pose,pose_at
  if m.header.frame_id=='map': pose=m; pose_at=time.monotonic()
 def onhealth(m):
  nonlocal health,health_at
  try: health=json.loads(m.data); health_at=time.monotonic()
  except ValueError: pass
 n.create_subscription(PoseStamped,'/d1/localization/pose6d',onpose,1)
 n.create_subscription(String,'/d1/localization/status',onhealth,1)
 samples=collections.deque(); last_stamp=None; identity=None
 deadline=time.monotonic()+30
 try:
  while time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=.1)
   now=time.monotonic()
   if pose and now-pose_at<.4 and now-health_at<.4 and health.get('ready') is True and health.get('map_id')==map_id:
    current=(health.get('session'),health.get('epoch'))
    if current!=identity: samples.clear();identity=current
    stamp=(pose.header.stamp.sec,pose.header.stamp.nanosec)
    if stamp==last_stamp: continue
    last_stamp=stamp
    q=pose.pose.orientation; xyz=pose.pose.position
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    values=(xyz.x,xyz.y,xyz.z,yaw)
    if not all(math.isfinite(v) for v in values): samples.clear(); continue
    samples.append((now,values))
    while samples and now-samples[0][0]>2.5: samples.popleft()
    if len(samples)<5 or now-samples[0][0]<2: continue
    first=samples[0][1]
    if any(math.dist(v[:3],first[:3])>.04 or abs(math.atan2(math.sin(v[3]-first[3]),math.cos(v[3]-first[3])))>.05 for _,v in samples): continue
    path=Path(__file__).resolve().parents[1]/'config/site.yaml'
    site=yaml.safe_load(path.read_text()); site['maps_validated']=False; site['scan_source_validated']=False; site['entry']=dict(x=xyz.x,y=xyz.y,z=xyz.z,yaw=yaw,frame='map')
    site['entry_map_id']=map_id
    path.write_text(yaml.safe_dump(site,sort_keys=False,allow_unicode=True))
    print('Saved measured lower entry:',site['entry']); return
   else: samples.clear()
  raise RuntimeError('no fresh verified visual/FAST-LIO pose')
 finally:
  n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
