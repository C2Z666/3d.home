#!/usr/bin/env python3
import argparse,subprocess,sys,os
from pathlib import Path
import yaml
r=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--scene',default=os.environ.get('D1_SCENE','old'));p.add_argument('stage',choices=['localization','cloud','nav2','mission','preview','fastlio']);p.add_argument('extra',nargs=argparse.REMAINDER);a=p.parse_args()
c=yaml.safe_load((r/'config/runtime.yaml').read_text())
if a.stage=='localization':
 from scene_registry import scene_bundle
 scene=scene_bundle(r,a.scene)
 cmd=[sys.executable,str(r/'scripts/localization_live.py'),'--scene',a.scene,'--visual-map',str(scene['visual_map']),'--odom-frame',c['odom_frame'],'--body-frame',c['body_frame'],'--max-visual-age',str(c['max_visual_age']),'--matching-profile',c.get('matching_profile','relaxed'),'--ros-args','-r','/Odometry:='+c['odom_topic'],'-r','/front_camera/image:='+c['image_topic']]
elif a.stage=='cloud':cmd=[sys.executable,str(r/'scripts/cloud_scan.py')]
elif a.stage=='nav2':cmd=['ros2','launch',str(r/'scripts/nav2_bringup.launch.py'),'scene:='+a.scene]+a.extra
elif a.stage=='preview':cmd=['ros2','launch',str(r/'scripts/map_preview.launch.py')]+a.extra
elif a.stage=='mission':cmd=[sys.executable,str(r/'scripts/navigate_to_goal.py'),'--scene',a.scene]+a.extra
else:
 p.error('Use your already running FAST-LIO workspace; reference parameters are in assets/fastlio/replay.yaml')
import os
os.execvp(cmd[0],cmd)
