"""One local progress decision and one event-driven stationary search gate."""
import math
import numpy as np
from navigation_control import wrap


class LocalProgress:
    def __init__(self,unavailable_sec=2.,progress_sec=3.,progress_m=.10):
        if not (0<unavailable_sec<=5 and 0<progress_sec<=10 and .02<=progress_m<=.5):
            raise ValueError('invalid local progress limits')
        self.unavailable_sec=unavailable_sec;self.progress_sec=progress_sec;self.progress_m=progress_m
        self.reset()

    def reset(self):
        self.anchor=None;self.at=None;self.unavailable_at=None;self.turn_origin=None;self.turn_best=0.
        self.visited=set();self.last_progress=False

    def update(self,now,pose,command,valid=True):
        self.last_progress=False
        if not valid:self.reset();return 'DATA_UNAVAILABLE'
        if self.anchor is None:
            self.anchor=tuple(pose[:2]);self.at=now
            self.visited.add(self.cell(pose))
        moved=math.dist(self.anchor,pose[:2])
        cell=self.cell(pose)
        # Net movement into a new region is progress even while going around an
        # obstacle away from the goal. Repeated shuffling over old cells is not.
        if moved>=self.progress_m and cell not in self.visited:
            self.anchor=tuple(pose[:2]);self.at=now;self.visited.add(cell)
            self.last_progress=True;self.turn_origin=None;self.turn_best=0.
            if len(self.visited)>4096:self.visited={cell}
        vx,wz=command
        moving=abs(vx)>.01 or abs(wz)>.03
        if not moving:
            if self.unavailable_at is None:self.unavailable_at=now
            if now-self.unavailable_at>=self.unavailable_sec:return 'NO_LOCAL_MOTION'
            return 'WAIT_LOCAL_MOTION'
        self.unavailable_at=None
        # Deliberate heading change gets credit, bounded to a half-turn without
        # translation. Pure spinning or yaw oscillation cannot renew it forever.
        if abs(vx)<.04 and abs(wz)>.03:
            if self.turn_origin is None:self.turn_origin=pose[2]
            turned=abs(wrap(pose[2]-self.turn_origin))
            if self.turn_best+.05<=turned<math.pi-.1:
                self.turn_best=turned;self.at=now
        if now-self.at>=self.progress_sec:return 'NO_PROGRESS'
        return 'ADVANCE'

    def cell(self,pose):
        return tuple(math.floor(v/self.progress_m) for v in pose[:2])


class SearchGate:
    """No new process job until a previous one finishes and the scene changes.

    Grid publication times and soft inflation noise are not changes. Retry when
    hard geometry changes in the local obstruction area, or start/goal changes.
    The search itself already includes the short bypass + one full-search fallback.
    """
    def __init__(self):
        self.busy=False;self.last=None

    def evidence(self,grid,start,goal):
        r=float(grid['resolution']);w,h=int(grid['width']),int(grid['height'])
        ox,oy,angle=grid['origin'];c,s=math.cos(angle),math.sin(angle)
        dx,dy=start[0]-ox,start[1]-oy
        x,y=math.floor((c*dx+s*dy)/r),math.floor((-s*dx+c*dy)/r)
        radius=math.ceil(3./r)
        box=(max(0,x-radius),max(0,y-radius),min(w,x+radius+1),min(h,y+radius+1))
        data=np.asarray(grid['data'],np.uint8).reshape(h,w)
        return dict(start=tuple(start),goal=tuple(goal),meta=(r,w,h,ox,oy,angle),box=box,
                    hard=(data>=254),area_cells=max(1,math.ceil(.01/(r*r))))

    def changed(self,evidence):
        old=self.last
        if old is None:return True
        if evidence['meta']!=old['meta'] or evidence['goal']!=old['goal']:return True
        p,q=evidence['start'],old['start']
        if math.dist(p[:2],q[:2])>=.10 or abs(wrap(p[2]-q[2]))>=.15:return True
        x0,y0,x1,y1=old['box']
        return np.count_nonzero(evidence['hard'][y0:y1,x0:x1]!=old['hard'][y0:y1,x0:x1])>=old['area_cells']

    def begin(self,evidence):
        if self.busy or not self.changed(evidence):return False
        self.last=evidence;self.busy=True;return True

    def finish(self):self.busy=False
