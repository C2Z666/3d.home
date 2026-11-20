"""Rectangular-body planning and continuous swept-volume certification.

Costs are raw Nav2 bytes: centre >=253, body >=254, unknown/outside blocked.
Search keeps heading as part of the state. Clearance is a soft preference only.
No modification of the costmap or its inflation radius is performed.
"""
import heapq
import math
import time
import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def turn_drift_margin(delta):
    """Allow up to 10 cm drift for a pivot, tapering for small heading changes."""
    return .10 * min(1., abs(delta) / (math.pi / 4))


class PlanningTimeout(ValueError):
    pass


class BodyGrid:
    def __init__(self, grid, half_length=.50, half_width=.275):
        self.r = float(grid['resolution'])
        self.w, self.h = int(grid['width']), int(grid['height'])
        self.ox, self.oy, self.angle = grid['origin']
        self.data = np.asarray(grid['data'], dtype=np.uint8).reshape(self.h, self.w)
        self.l, self.b = half_length, half_width
        if self.r <= 0 or min(self.l, self.b) <= 0:
            raise ValueError('invalid body/grid dimensions')
        self.radius = math.hypot(self.l, self.b)
        self.hard = self.data >= 254
        self.clearance = distance_transform_edt(np.pad(~self.hard, 1))[1:-1, 1:-1]*self.r
        self.fields = {}
        self.blocked_evidence = {}

    def local(self, p):
        x, y = p[0]-self.ox, p[1]-self.oy
        c, s = math.cos(self.angle), math.sin(self.angle)
        return np.array([c*x+s*y, -s*x+c*y, wrap(p[2]-self.angle)])

    def world(self, p):
        c, s = math.cos(self.angle), math.sin(self.angle)
        return [self.ox+c*p[0]-s*p[1], self.oy+s*p[0]+c*p[1], wrap(p[2]+self.angle)]

    def pose_clear(self, x, y, yaw, pad=0.):
        col, row = math.floor(x/self.r), math.floor(y/self.r)
        if not (0 <= col < self.w and 0 <= row < self.h) or self.data[row, col] >= 253:
            self.blocked_evidence=dict(kind='centre',cell=[col,row],cost=int(self.data[row,col]) if 0<=col<self.w and 0<=row<self.h else 255)
            return False
        # Fast certificate, including cell extent and subcell centre displacement.
        if self.clearance[row, col] > self.radius+math.sqrt(2)*pad+2*self.r:
            return True
        l, b = self.l+pad, self.b+pad
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = abs(c)*l+abs(s)*b, abs(s)*l+abs(c)*b
        if x-ex <= 0 or y-ey <= 0 or x+ex >= self.w*self.r or y+ey >= self.h*self.r:
            self.blocked_evidence=dict(kind="map_boundary",cost=255)
            return False
        c0, c1 = math.floor((x-ex)/self.r), math.floor((x+ex)/self.r)
        r0, r1 = math.floor((y-ey)/self.r), math.floor((y+ey)/self.r)
        rr, cc = np.nonzero(self.hard[r0:r1+1, c0:c1+1])
        dx, dy = (cc+c0+.5)*self.r-x, (rr+r0+.5)*self.r-y
        half = self.r/2
        # Rectangle versus closed cell square, all four separating axes.
        hit = ((np.abs(dx) <= ex+half) & (np.abs(dy) <= ey+half) &
               (np.abs(c*dx+s*dy) <= l+half*(abs(c)+abs(s))) &
               (np.abs(-s*dx+c*dy) <= b+half*(abs(c)+abs(s))))
        if hit.any():
            k=int(np.flatnonzero(hit)[0]);col=int(cc[k]+c0);row=int(rr[k]+r0)
            local=np.array([(col+.5)*self.r,(row+.5)*self.r,0.])
            self.blocked_evidence=dict(kind='body',cell=[col,row],cost=int(self.data[row,col]),
                                       world=self.world(local)[:2])
        return not hit.any()

    def turn_clear(self, p, yaw):
        delta = wrap(yaw-p[2])
        n = max(1, math.ceil(abs(delta)/math.radians(3)))
        # Every intermediate rotated body is contained in a sampled body padded
        # by the maximum point displacement to the nearest angular sample.
        pad = turn_drift_margin(delta)+self.radius*abs(delta)/(2*n)+1e-6
        return all(self.pose_clear(p[0], p[1], p[2]+delta*i/n, pad) for i in range(n+1))

    def line_clear(self, p, q, yaw):
        distance = math.hypot(q[0]-p[0], q[1]-p[1])
        n = max(1, math.ceil(distance/(self.r/2)))
        pad = distance/(2*n)+1e-6
        # Also pad centre path against forbidden centre cells, independently of
        # the body padding. Sample spacing <= half cell plus exact grid traversal.
        for i in range(n+1):
            f=i/n
            if not self.pose_clear(p[0]+f*(q[0]-p[0]), p[1]+f*(q[1]-p[1]), yaw, pad):
                return False
        # Supercover centre traversal (including short diagonal corner crossings).
        cuts = [0., 1.]
        for axis in (0, 1):
            d=q[axis]-p[axis]
            if abs(d)>1e-12:
                lo,hi=sorted((p[axis],q[axis]))
                cuts.extend((k*self.r-p[axis])/d for k in range(math.floor(lo/self.r)+1,math.ceil(hi/self.r)))
        cuts=sorted(cuts)
        for f in cuts+[(a+b)/2 for a,b in zip(cuts,cuts[1:])]:
            x,y=p[0]+f*(q[0]-p[0]),p[1]+f*(q[1]-p[1])
            # Touching a forbidden centre cell also counts as blocked.
            for epsx,epsy in ((0,0),(1e-9,1e-9),(-1e-9,-1e-9),(1e-9,-1e-9),(-1e-9,1e-9)):
                c,r=math.floor((x+epsx)/self.r),math.floor((y+epsy)/self.r)
                if not (0<=c<self.w and 0<=r<self.h) or self.data[r,c]>=253:
                    self.blocked_evidence=dict(kind='centre_segment',cell=[c,r],cost=int(self.data[r,c]) if 0<=c<self.w and 0<=r<self.h else 255)
                    return False
        return True

    def connect(self, p, q, final_yaw=None):
        distance=math.hypot(q[0]-p[0],q[1]-p[1])
        yaw=math.atan2(q[1]-p[1],q[0]-p[0]) if distance>1e-8 else p[2]
        if not self.turn_clear(p,yaw) or not self.line_clear(p,q,yaw):return False
        return final_yaw is None or self.turn_clear([q[0],q[1],yaw],final_yaw)

    def field(self, heading, turn=False):
        key=(heading%8,turn)
        if key in self.fields:return self.fields[key]
        # Translation fields only prune endpoint/midpoint collisions; line_clear
        # certifies the complete edge. Do not add a circular clearance envelope.
        pad = 0. if not turn else turn_drift_margin(math.pi/4)+self.radius*math.pi/64+1e-6
        size=math.ceil(math.hypot(self.l+pad,self.b+pad)/self.r)+2
        yy,xx=np.mgrid[-size:size+1,-size:size+1]*self.r
        mask=np.zeros(xx.shape,bool)
        angles=[heading*math.pi/4] if not turn else np.linspace(heading*math.pi/4,(heading+1)*math.pi/4,9)
        for a in angles:
            c,s=math.cos(a),math.sin(a);l,b=self.l+pad,self.b+pad;hh=self.r/2
            mask |= ((abs(xx)<=abs(c)*l+abs(s)*b+hh)&(abs(yy)<=abs(s)*l+abs(c)*b+hh)&
                     (abs(c*xx+s*yy)<=l+hh*(abs(c)+abs(s)))&(abs(-s*xx+c*yy)<=b+hh*(abs(c)+abs(s))))
        result=cv2.dilate(self.hard.astype(np.uint8),mask.astype(np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=1).astype(bool)
        result |= self.data>=253
        self.fields[key]=result
        return result


def remaining_points(points, current):
    p=np.asarray(points,dtype=float)
    start=int(np.argmin(np.sum((p[:,:2]-np.asarray(current[:2]))**2,axis=1)))
    while start<len(p)-1:
        passed=float(np.dot(np.asarray(current[:2])-p[start,:2],p[start+1,:2]-p[start,:2]))>1e-9
        if np.linalg.norm(p[start,:2]-current[:2])<.03 or passed:start+=1
        else:break
    return p[start:],start


def departure_heading(points,current):
    p,_=remaining_points(points,current)
    return math.atan2(p[0,1]-current[1],p[0,0]-current[0]) if np.linalg.norm(p[0,:2]-current[:2])>1e-7 else current[2]


def certify(points, current, grid, half_length=.50, half_width=.275, goal_yaw=None, trim=True):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or len(p)==0 or p.shape[1]<2 or not np.isfinite(p).all():return False,dict(reason='invalid_path')
    body=grid if isinstance(grid,BodyGrid) else BodyGrid(grid,half_length,half_width)
    cur=body.local(current)
    remaining,start=remaining_points(p,current) if trim else (p,0)
    clear_distance=0.
    for i,v in enumerate(remaining):
        q=body.local([v[0],v[1],0.])
        if math.hypot(*(q[:2]-cur[:2]))<1e-7:continue
        if not body.connect(cur,q):
            col,row=math.floor(q[0]/body.r),math.floor(q[1]/body.r)
            cost=int(body.data[row,col]) if 0<=col<body.w and 0<=row<body.h else 255
            return False,dict(reason='swept_body_blocked',segment=start+i,x=float(v[0]),y=float(v[1]),distance_m=float(np.linalg.norm(v[:2]-current[:2])),centre_cost=cost,clear_prefix_m=clear_distance,blocked=body.blocked_evidence)
        clear_distance+=float(np.linalg.norm(q[:2]-cur[:2]))
        cur=np.array([q[0],q[1],math.atan2(q[1]-cur[1],q[0]-cur[0])])
    if goal_yaw is not None and not body.turn_clear(cur,wrap(goal_yaw-body.angle)):
        return False,dict(reason='goal_turn_blocked',x=float(p[-1,0]),y=float(p[-1,1]),clear_prefix_m=clear_distance,blocked=body.blocked_evidence)
    return True,dict(reason='swept_body_clear',checked_segments=len(p)-start,remaining_start_index=start)


def plan_body(grid, start, goal, half_length=.50, half_width=.275, timeout=8.):
    begin=time.monotonic();body=BodyGrid(grid,half_length,half_width)
    s,t=body.local(start),body.local(goal)
    if not body.pose_clear(*s):raise ValueError('起点机身覆盖实体障碍或未知区，无法认证起步')
    if not body.pose_clear(*t):raise ValueError('目标机身或朝向与障碍冲突，请调整目标')
    # 8-heading state lattice, 10 cm translations and explicit certified turns.
    step=2;dirs=[(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    sc,sr=int(s[0]/body.r),int(s[1]/body.r)
    dist={};parent={};queue=[];serial=0
    def pose(state):
        c,r,h=state;return [(c+.5)*body.r,(r+.5)*body.r,h*math.pi/4]
    def push(state,cost,prev):
        nonlocal serial
        if cost>=dist.get(state,float('inf')):return
        dist[state]=cost;parent[state]=prev;serial+=1
        q=pose(state);heur=math.hypot(q[0]-t[0],q[1]-t[1])
        heapq.heappush(queue,(cost+heur,serial,cost,state))
    for dc,dr in [(0,0),(-1,0),(1,0),(0,-1),(0,1)]:
        for h in range(8):
            state=(sc+dc,sr+dr,h);q=pose(state)
            if body.connect(s,q,q[2]):push(state,math.hypot(q[0]-s[0],q[1]-s[1])+.08*abs(wrap(q[2]-s[2])),None)
    # A subcell offset must not force a 45-degree turn at a narrow-corridor
    # start. Connect farther lattice anchors with the same full sweep check.
    for h in range(8):
        for distance in (.25,.5,.75):
            a=h*math.pi/4
            state=(int((s[0]+distance*math.cos(a))/body.r),int((s[1]+distance*math.sin(a))/body.r),h)
            q=pose(state)
            if body.connect(s,q,q[2]):push(state,math.hypot(q[0]-s[0],q[1]-s[1])+.08*abs(wrap(q[2]-s[2])),None)
    expanded=0;end=None
    while queue:
        if time.monotonic()-begin>timeout:raise PlanningTimeout('机身约束搜索超时，未发布未经验证的路线')
        _,_,cost,state=heapq.heappop(queue)
        if cost!=dist[state]:continue
        expanded+=1;c,r,h=state;q=pose(state)
        if math.hypot(q[0]-t[0],q[1]-t[1])<.9 and body.connect(q,t,t[2]):end=state;break
        for dh in (-1,1):
            nh=(h+dh)%8;field=body.field(h if dh==1 else nh,True)
            if 0<=c<body.w and 0<=r<body.h and not field[r,c]:push((c,r,nh),cost+.10,state)
        dc,dr=dirs[h];nc,nr=c+dc*step,r+dr*step
        if not (0<=nc<body.w and 0<=nr<body.h):continue
        field=body.field(h)
        if field[r,c] or field[nr,nc] or field[r+dr,c+dc]:continue
        # Exact centre-cell traversal is cheap and closes diagonal corner gaps.
        if not body.line_clear(q,pose((nc,nr,h)),q[2]):continue
        clearance=body.clearance[nr,nc]
        soft=1.+2.*max(0.,(.80-clearance)/.80)**2+float(body.data[nr,nc])/252.*.5
        push((nc,nr,h),cost+body.r*step*math.hypot(dc,dr)*soft,state)
    if end is None:raise ValueError('已搜索可达位置与朝向，未找到满足机身及转向约束的绕行路线')
    states=[]
    while end is not None:states.append(pose(end));end=parent[end]
    states.reverse()
    # Remove in-place states; final certification checks all actual heading changes.
    raw=[s]
    for q in states:
        if np.linalg.norm(np.asarray(q[:2])-np.asarray(raw[-1][:2]))>1e-7:raw.append(np.asarray(q))
    raw.append(t)
    # Shortcut only when fully swept-clear AND not sacrificing wall clearance.
    out=[s];idx=0
    while idx<len(raw)-1:
        chosen=idx+1
        for j in range(min(len(raw)-1,idx+20),idx,-1):
            q=raw[j];direction=math.atan2(q[1]-out[-1][1],q[0]-out[-1][0])
            final=t[2] if j==len(raw)-1 else math.atan2(raw[j+1][1]-q[1],raw[j+1][0]-q[0])
            if not body.connect(out[-1],q,final):continue
            samples=np.linspace(out[-1][:2],q[:2],max(2,math.ceil(np.linalg.norm(q[:2]-out[-1][:2])/.05)))
            cc=np.clip((samples[:,0]/body.r).astype(int),0,body.w-1);rr=np.clip((samples[:,1]/body.r).astype(int),0,body.h-1)
            old=[body.clearance[min(body.h-1,max(0,int(v[1]/body.r))),min(body.w-1,max(0,int(v[0]/body.r)))] for v in raw[idx:j+1]]
            if body.clearance[rr,cc].min()+.025<min(.65,min(old)):continue
            chosen=j;break
        q=raw[chosen];out.append(np.array([q[0],q[1],math.atan2(q[1]-out[-1][1],q[0]-out[-1][0])]))
        idx=chosen
    points=[body.world(q)[:2] for q in out]
    ok,detail=certify(points,start,body,goal_yaw=goal[2],trim=False)
    if not ok:raise ValueError('候选路线未通过统一连续机身复核: '+str(detail))
    # Densify straight certified segments for DWB, preserving exact corners.
    result=[]
    for a,b in zip(points,points[1:]):
        yaw=math.atan2(b[1]-a[1],b[0]-a[0]);n=max(1,math.ceil(math.dist(a,b)/.05))
        result.extend(dict(x=a[0]+(b[0]-a[0])*i/n,y=a[1]+(b[1]-a[1])*i/n,yaw=yaw) for i in range(n))
    result.append(dict(x=goal[0],y=goal[1],yaw=goal[2]))
    return result,dict(planner='rectangular_lattice',expanded=expanded,compute_sec=time.monotonic()-begin,certified=True)
