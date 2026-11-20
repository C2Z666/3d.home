"""Freeze a map-frame arrival plane in continuous odometry at stair entry."""
import math


def bind_arrival_line(connection, map_pose, paired_raw, origin):
    line=connection.get('arrival_line')
    if line is None:return None
    if line['floor_id']!=connection['to_floor']:raise ValueError('arrival line belongs to wrong floor')
    if abs(map_pose['stamp']-paired_raw['stamp'])>.1:raise ValueError('arrival line reference timestamps mismatch')
    angle=paired_raw['yaw']-map_pose['yaw'];c,s=math.cos(angle),math.sin(angle)
    dx,dy=line['x']-map_pose['x'],line['y']-map_pose['y']
    x=paired_raw['x']+c*dx-s*dy;y=paired_raw['y']+s*dx+c*dy
    yaw=line['yaw']+angle
    return dict(x=x,y=y,yaw=yaw,floor_id=line['floor_id'],connection_id=connection['id'])


def crossed(line,sample,target_floor,connection_id):
    if line is None:return None,None
    distance=(sample['x']-line['x'])*math.cos(line['yaw'])+(sample['y']-line['y'])*math.sin(line['yaw'])
    valid=(line['floor_id']==target_floor and line['connection_id']==connection_id and
           sample.get('candidate_floor')==target_floor and sample.get('connection_id')==connection_id)
    return valid and distance>=0.,distance
