"""Bounded acquisition-time planar pose matching; never extrapolates."""
import math
from collections import deque

class PoseHistory:
    def __init__(self):
        self.samples=deque(maxlen=600)
    def clear(self): self.samples.clear()
    def add(self,p):
        if self.samples and p['stamp']<=self.samples[-1]['stamp']: self.clear()
        self.samples.append(dict(p))
        while self.samples and p['stamp']-self.samples[0]['stamp']>5: self.samples.popleft()
    def at(self,t):
        for b in self.samples:
            if abs(b['stamp']-t)<1e-6:return dict(b,stamp=t),''
        for a,b in zip(self.samples,list(self.samples)[1:]):
            if a['stamp']<t<b['stamp']:
                gap=b['stamp']-a['stamp']
                if gap>.25:return None,'定位采样间隔过大'
                w=(t-a['stamp'])/gap
                dy=math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw']))
                return dict(x=a['x']+w*(b['x']-a['x']),y=a['y']+w*(b['y']-a['y']),yaw=a['yaw']+w*dy,stamp=t),''
        return None,'等待采集时刻的定位配对'
