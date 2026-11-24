"""Small, current-command stopping sweeps. No global grid or distance transform."""
import math
import time
import numpy as np
from navigation_control import quaternion_yaw


class LocalGrid:
    def __init__(self, message):
        info=message.info
        self.res=float(info.resolution);self.w=int(info.width);self.h=int(info.height)
        self.origin=(info.origin.position.x,info.origin.position.y,quaternion_yaw(info.origin.orientation))
        if self.res<=0 or min(self.w,self.h)<=0 or not all(math.isfinite(v) for v in (*self.origin,self.res)):
            raise ValueError('invalid local map metadata')
        self.data=np.asarray(message.data,dtype=np.int16).reshape(self.h,self.w)
        if ((self.data < -1)|(self.data > 100)).any():raise ValueError('invalid occupancy values')

    def pose(self,p):
        c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        x,y=p[0]-self.origin[0],p[1]-self.origin[1]
        return c*x+s*y,-s*x+c*y,p[2]-self.origin[2]

    def rectangle(self,pose,length,width,pad=0.):
        x,y,a=self.pose(pose);r=self.res
        c,s=math.cos(a),math.sin(a);ac,ass=abs(c),abs(s)
        L,W=length+pad,width+pad
        ex,ey=L*ac+W*ass,L*ass+W*ac
        c0,c1=math.floor((x-ex)/r),math.floor((x+ex)/r)
        r0,r1=math.floor((y-ey)/r),math.floor((y+ey)/r)
        if c0<0 or r0<0 or c1>=self.w or r1>=self.h:
            return 'DATA_UNAVAILABLE',dict(reason='stopping footprint outside local map')
        cx,cy=math.floor(x/r),math.floor(y/r)
        centre=int(self.data[cy,cx])
        if centre<0:return 'DATA_UNAVAILABLE',dict(reason='unknown robot centre',cell=[cx,cy,centre])
        if centre>=99:return 'BLOCKED',dict(reason='forbidden robot centre',cell=[cx,cy,centre])
        area=self.data[r0:r1+1,c0:c1+1]
        rr,cc=np.nonzero((area<0)|(area>=100))
        if not len(rr):return 'CLEAR',{}
        dx,dy=(cc+c0+.5)*r-x,(rr+r0+.5)*r-y
        half=r/2
        hits=(np.abs(c*dx+s*dy)<=L+half*(ac+ass))&(np.abs(-s*dx+c*dy)<=W+half*(ac+ass))
        # The bounding box already supplies the other two separating axes.
        indices=np.flatnonzero(hits)
        if len(indices):
            # Known collision takes precedence over unavailable cells elsewhere.
            known=[i for i in indices if area[rr[i],cc[i]]>=100]
            i=known[0] if known else indices[0];cost=int(area[rr[i],cc[i]])
            return ('BLOCKED' if cost>=100 else 'DATA_UNAVAILABLE'),dict(reason='occupied footprint' if cost>=100 else 'unknown footprint',cell=[int(cc[i]+c0),int(rr[i]+r0),cost])
        return 'CLEAR',{}


def stopping_sweep(grid,pose,command,measured,half_length,half_width,source_age,
                   reaction_sec=.20,decel=.5,angular_decel=1.):
    """Check commanded AND measured braking arcs against the latest local map.

    Occupancy 99 forbids the centre; only 100/-1 forbids the whole footprint.
    Source age contributes to reaction distance; it never refreshes observations.
    This uses the configured acceleration limits, not a measured braking guarantee.
    """
    begin=time.monotonic()
    values=(*pose,*command,*measured,half_length,half_width,source_age,reaction_sec,decel,angular_decel)
    if grid is None or not all(math.isfinite(v) for v in values) or min(decel,angular_decel)<=0:
        return 'DATA_UNAVAILABLE',dict(reason='invalid stopping input')
    samples=0;radius=math.hypot(half_length,half_width)
    # Intermediate speeds are covered by checking both endpoints plus a bounded
    # interpolation pad for one control period, including lateral measured drift.
    pad=.05*(abs(command[0]-measured[0])+radius*abs(command[1]-measured[2]))
    reaction=reaction_sec+max(0.,source_age)
    branches=[(command[0],0.,command[1])]
    if tuple(measured)!=branches[0]:branches.append(tuple(measured))
    for v,vy,w in branches:
        x,y,a=pose
        duration=reaction+max(abs(v)/decel,abs(vy)/decel,abs(w)/angular_decel)
        dt=min(.05,grid.res/(4*max(abs(v)+abs(vy)+radius*abs(w),.01)))
        count=max(1,math.ceil(duration/dt));dt=duration/count
        for i in range(count+1):
            t=i*dt
            speed=lambda value,limit:math.copysign(max(0.,abs(value)-limit*max(0.,t-reaction)),value)
            vx,side,wz=speed(v,decel),speed(vy,decel),speed(w,angular_decel)
            sample_pad=pad+dt*(abs(vx)+abs(side)+radius*abs(wz))/2+1e-6
            state,detail=grid.rectangle((x,y,a),half_length,half_width,sample_pad)
            samples+=1
            if state!='CLEAR':
                return state,dict(detail,samples=samples,compute_ms=(time.monotonic()-begin)*1000)
            mid=a+wz*dt/2
            x+=(vx*math.cos(mid)-side*math.sin(mid))*dt
            y+=(vx*math.sin(mid)+side*math.cos(mid))*dt
            a+=wz*dt
    return 'CLEAR',dict(samples=samples,compute_ms=(time.monotonic()-begin)*1000)
