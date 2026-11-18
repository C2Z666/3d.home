"""Measured 3D rays only. No synthesized free rays for missing/filtered returns.

Clearing uses the minimum measured range across >=3 distinct stationary frames
spanning .3s. Moving/occluded/missing beams never inherit old free evidence.
The VoxelLayer retains the height of occupied cells, unlike a flattened scan.
"""
from collections import deque
import numpy as np


def voxel_sample(points,size=.05):
    if not len(points):return np.empty((0,3),dtype=np.float32)
    keys=np.floor(points/size).astype(np.int64)+1024
    packed=(keys[:,0]<<42)+(keys[:,1]<<21)+keys[:,2]
    _,idx=np.unique(packed,return_index=True)
    return np.asarray(points[idx],dtype=np.float32)


class ClearingEvidence:
    def __init__(self,span=.3,max_gap=.25):
        self.rows=deque(maxlen=8);self.span=span;self.max_gap=max_gap;self.reason='waiting'

    def clear(self):self.rows.clear()

    def update(self,points,stamp,world_sensor):
        empty=np.empty((0,3),dtype=np.float32)
        if self.rows and stamp<=self.rows[-1][0]:
            if stamp<self.rows[-1][0]:self.clear()
            self.reason='duplicate_or_regressed_stamp';return empty
        if self.rows and stamp-self.rows[-1][0]>self.max_gap:self.clear()
        if self.rows:
            anchor=self.rows[0][1];relative=anchor[:3,:3].T@world_sensor[:3,:3]
            angle=np.arccos(np.clip((np.trace(relative)-1)/2,-1,1))
            if np.linalg.norm(world_sensor[:3,3]-anchor[:3,3])>.02 or angle>np.deg2rad(.5):
                self.clear();self.reason='moving'
        p=np.asarray(points);distance=np.linalg.norm(p,axis=1)
        valid=np.isfinite(p).all(axis=1)&(distance>.3)&(distance<30.)
        p=p[valid];distance=distance[valid]
        if not len(p):self.clear();self.reason='no_measured_returns';return empty
        az=np.floor((np.arctan2(p[:,1],p[:,0])+np.pi)/np.deg2rad(2.)).astype(int)%180
        el=np.clip(np.floor((np.arcsin(p[:,2]/distance)+np.pi/2)/np.deg2rad(2.)).astype(int),0,89)
        bins=el*180+az
        ranges=np.full(180*90,np.inf,dtype=np.float32);np.minimum.at(ranges,bins,distance)
        # Retain an actual measured direction, never a fabricated bin-centre ray.
        nearest=np.flatnonzero(distance<=ranges[bins]+1e-5)
        _,first=np.unique(bins[nearest],return_index=True);idx=nearest[first]
        self.rows.append((stamp,world_sensor.copy(),ranges))
        while len(self.rows)>3 and stamp-self.rows[1][0]>=self.span:self.rows.popleft()
        if len(self.rows)<3 or stamp-self.rows[0][0]<self.span-1e-6:
            self.reason='confirming_stationary_frames';return empty
        stack=np.stack([r[2] for r in self.rows]);seen=np.isfinite(stack[:,bins[idx]]).all(axis=0)
        idx=idx[seen]
        if not len(idx):self.reason='no_consistent_beams';return empty
        minimum=np.min(stack[:,bins[idx]],axis=0)
        # Stop before measured surfaces; VoxelLayer also shortens clearing rays.
        length=np.minimum(minimum,4.1)-.05
        result=p[idx]*(length/distance[idx])[:,None]
        self.reason='confirmed';return np.asarray(result,dtype=np.float32)
