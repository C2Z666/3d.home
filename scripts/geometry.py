import numpy as np
import yaml
from scipy.spatial.transform import Rotation,Slerp

def rigid(T):
    T=np.array(T,dtype=float,copy=True)
    if T.shape!=(4,4) or not np.isfinite(T).all(): raise ValueError('invalid transform')
    u,_,v=np.linalg.svd(T[:3,:3]); R=u@v
    if np.linalg.det(R)<0: raise ValueError('reflected rotation')
    T[:3,:3]=R
    return T

def calibration(path):
    c=yaml.safe_load(path.read_text(encoding='utf-8')); e=c['transforms']
    # Preserve original chain: project rotation only AFTER composing base<-IMU<-lidar.
    B_L=rigid(np.array(e['imu_front_to_base'])@np.array(e['lidar_front_to_imu_front']))
    B_C=rigid(B_L@np.array(e['camera_front_to_lidar_front']))
    K=np.array(c['K'],dtype=float)/2; K[2,2]=1
    return K,np.array(c['D']).reshape(-1),B_L,B_C

def imu_from_base(path):
    c=yaml.safe_load(path.read_text(encoding='utf-8')); _,_,B_L,_=calibration(path)
    I_L=rigid(c['transforms']['lidar_front_to_imu_front'])
    return rigid(I_L@np.linalg.inv(B_L))

def transform(T,xyz): return np.asarray(xyz)@T[:3,:3].T+T[:3,3]
def pose_matrix(p):
    T=np.eye(4); q=p.orientation
    T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    T[:3,3]=[p.position.x,p.position.y,p.position.z]
    return rigid(T)
def planar(T):
    out=np.eye(4); yaw=np.arctan2(T[1,0],T[0,0])
    out[:3,:3]=Rotation.from_euler('z',yaw).as_matrix(); out[:2,3]=T[:2,3]
    return out

def interpolate(history,t,max_gap=.25):
    times=np.array([v[0] for v in history]); j=int(np.searchsorted(times,t))
    if j<len(times) and abs(times[j]-t)<1e-7: return history[j][1].copy()
    if j==0 or j==len(times): raise ValueError('no odometry bracket; no extrapolation')
    a,A=history[j-1]; b,B=history[j]
    if b-a>max_gap or b<=a: raise ValueError('odometry gap')
    w=(t-a)/(b-a); T=np.eye(4); T[:3,3]=(1-w)*A[:3,3]+w*B[:3,3]
    T[:3,:3]=Slerp([a,b],Rotation.from_matrix([A[:3,:3],B[:3,:3]]))([t]).as_matrix()[0]
    return T

class Correction:
    def __init__(self): self.T=None; self.candidate=None; self.count=0; self.last_visual=None; self.candidate_time=None
    def update(self,V,O,t):
        if self.candidate_time is not None and t<=self.candidate_time: return False
        C=rigid(V@np.linalg.inv(O))
        if self.T is not None:
            delta=np.linalg.inv(self.T@O)@V
            if np.linalg.norm(delta[:3,3])>.4 or Rotation.from_matrix(delta[:3,:3]).magnitude()>np.deg2rad(20):
                return False
        else:
            if self.candidate is not None:
                d=np.linalg.inv(self.candidate)@C
                if np.linalg.norm(d[:3,3])>.25 or Rotation.from_matrix(d[:3,:3]).magnitude()>np.deg2rad(10):
                    self.count=0
            self.candidate=C; self.candidate_time=t; self.count+=1
            if self.count<2: return False
        self.T=C; self.last_visual=t
        return True

# Fixed basis measured on this FAST-LIO deployment: Y-up world -> Z-up navigation.
LIO_TO_NAV=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
def navigation_pose(O):
    C=np.eye(4);C[:3,:3]=LIO_TO_NAV
    return C@O
