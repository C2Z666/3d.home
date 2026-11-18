"""Manual map alignment seeds; never modify the FAST-LIO estimator."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import navigation_pose, rigid
from floor_routes import finite


def alignment(request, raw_base, map_id):
    if request.get('map_id')!=map_id:raise ValueError('手动初值地图身份不匹配')
    if not all(finite(request.get(k)) for k in ('x','y','z','yaw')):
        raise ValueError('手动初值需要有限的 x、y、z 和 yaw')
    pose=navigation_pose(raw_base)
    old_yaw=math.atan2(pose[1,0],pose[0,0])
    pose[:3,:3]=Rotation.from_euler('z',request['yaw']-old_yaw).as_matrix()@pose[:3,:3]
    pose[:3,3]=[request[k] for k in ('x','y','z')]
    return rigid(pose@np.linalg.inv(raw_base))
