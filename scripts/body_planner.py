"""Rectangular-body planning and continuous swept-volume certification.

Costs are raw Nav2 bytes: centre >=253, body >=254, unknown/outside blocked.
Search keeps heading as part of the state. Clearance is a soft preference only.
No modification of the costmap or its inflation radius is performed.
"""
import json
import heapq
import math
import time
import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt


# Integer cell displacements keep every heading aligned with its translation.
DIRECTIONS = ((1,0),(2,1),(1,1),(1,2),(0,1),(-1,2),(-1,1),(-2,1),
              (-1,0),(-2,-1),(-1,-1),(-1,-2),(0,-1),(1,-2),(1,-1),(2,-1))
ANGLES = tuple(math.atan2(y,x) % (2*math.pi) for x,y in DIRECTIONS)


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
        # A lattice edge in open space can be certified without sampling the
        # footprint repeatedly. The centre can still encounter inflated cells.
        col,row = math.floor(p[0]/self.r),math.floor(p[1]/self.r)
        if (0<=col<self.w and 0<=row<self.h and
                self.clearance[row,col] > distance+self.radius+3*self.r):
            c,r = math.floor(q[0]/self.r),math.floor(q[1]/self.r)
            box=self.data[min(row,r)-1:max(row,r)+2,min(col,c)-1:max(col,c)+2]
            if box.size and box.max()<253:
                return True
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
        key=(heading%len(ANGLES),turn)
        if key in self.fields:return self.fields[key]
        # Translation fields only prune endpoint/midpoint collisions; line_clear
        # certifies the complete edge. Do not add a circular clearance envelope.
        angle = ANGLES[heading]
        delta = (ANGLES[(heading+1)%len(ANGLES)]-angle) % (2*math.pi)
        pad = 0. if not turn else turn_drift_margin(delta)+self.radius*delta/16+1e-6
        size=math.ceil(math.hypot(self.l+pad,self.b+pad)/self.r)+2
        yy,xx=np.mgrid[-size:size+1,-size:size+1]*self.r
        mask=np.zeros(xx.shape,bool)
        angles=[angle] if not turn else np.linspace(angle,angle+delta,9)
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


def progress_index(points,current,first=0):
    """Advance only along successive segments of this route."""
    p=np.asarray(points)
    pos=np.asarray(current[:2])
    best=first;nearest=math.inf;distance=0.
    for i in range(first,len(p)-1):
        delta=p[i+1,:2]-p[i,:2]
        length=float(np.linalg.norm(delta))
        if length<1e-8:continue
        f=float(np.clip(np.dot(pos-p[i,:2],delta)/length**2,0.,1.))
        error=np.linalg.norm(pos-(p[i,:2]+f*delta))
        if error<nearest-1e-8:
            best=i+int(f>=1.-1e-8);nearest=error
        distance+=length
        if distance>=1.25:break
    return best


def tracking_suffix(points,current):
    p=np.asarray(points)
    i=progress_index(p,current)
    if i==len(p)-1:
        return p[i:],i
    # Rejoin the current straight leg ahead, never look through a corner.
    j=i+1
    direction=p[j,:2]-p[i,:2]
    while j<len(p)-1 and np.linalg.norm(p[j,:2]-current[:2])<.35:
        following=p[j+1,:2]-p[j,:2]
        if np.dot(direction,following)<=0 or abs(direction[0]*following[1]-direction[1]*following[0])>1e-7:
            break
        j+=1
    return p[j:],j


def departure_heading(points,current):
    p,_=tracking_suffix(points,current)
    return math.atan2(p[0,1]-current[1],p[0,0]-current[0]) if np.linalg.norm(p[0,:2]-current[:2])>1e-7 else current[2]


def certify(points, current, grid, half_length=.50, half_width=.275, goal_yaw=None, trim=True):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or len(p)==0 or p.shape[1]<2 or not np.isfinite(p).all():return False,dict(reason='invalid_path')
    body=grid if isinstance(grid,BodyGrid) else BodyGrid(grid,half_length,half_width)
    cur=body.local(current)
    remaining,start=tracking_suffix(p,current) if trim else (p,0)
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


def corridor_heuristic(body, target, scale=4):
    """Estimate distance around obstacles on a coarse centre-point grid."""
    h = (body.h + scale - 1) // scale
    w = (body.w + scale - 1) // scale
    # Sampling the centres keeps this grid permissive. It guides search only;
    # the full-resolution footprint checks remain authoritative.
    rows = np.minimum(np.arange(h)*scale + scale//2, body.h-1)
    cols = np.minimum(np.arange(w)*scale + scale//2, body.w-1)
    blocked = body.data[np.ix_(rows, cols)] >= 253
    tc = min(w-1, max(0, int(target[0]/body.r)//scale))
    tr = min(h-1, max(0, int(target[1]/body.r)//scale))
    distance = np.full((h, w), np.inf)
    distance[tr, tc] = 0.
    queue = [(0., tc, tr)]
    while queue:
        cost, c, r = heapq.heappop(queue)
        if cost != distance[r, c]:
            continue
        for dc, dr in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nc, nr = c+dc, r+dr
            if not (0 <= nc < w and 0 <= nr < h) or blocked[nr,nc]:
                continue
            value = cost + body.r*scale*math.hypot(dc,dr)
            if value < distance[nr,nc]:
                distance[nr,nc] = value
                heapq.heappush(queue, (value,nc,nr))
    return distance


def plan_body(grid, start, goal, half_length=.50, half_width=.275, timeout=8.):
    begin=time.monotonic();body=BodyGrid(grid,half_length,half_width)
    s,t=body.local(start),body.local(goal)
    if not body.pose_clear(*s):raise ValueError('起点机身覆盖实体障碍或未知区，无法认证起步')
    if not body.pose_clear(*t):raise ValueError('目标机身或朝向与障碍冲突，请调整目标')
    step=2;dirs=DIRECTIONS
    sc,sr=int(s[0]/body.r),int(s[1]/body.r)
    guidance=corridor_heuristic(body,t)
    dist={};parent={};queue=[];serial=0
    def pose(state):
        c,r,h=state;return [(c+.5)*body.r,(r+.5)*body.r,ANGLES[h]]
    def push(state,cost,prev):
        nonlocal serial
        if cost>=dist.get(state,float('inf')):return
        dist[state]=cost;parent[state]=prev;serial+=1
        q=pose(state);heur=math.hypot(q[0]-t[0],q[1]-t[1])
        c,r,_=state
        if 0<=r<body.h and 0<=c<body.w and math.isfinite(guidance[r//4,c//4]):
            heur=max(heur,float(guidance[r//4,c//4]))
        heapq.heappush(queue,(cost+3.*heur,serial,cost,state))
    for dc,dr in [(0,0),(-1,0),(1,0),(0,-1),(0,1)]:
        for h in range(len(ANGLES)):
            state=(sc+dc,sr+dr,h);q=pose(state)
            if body.connect(s,q,q[2]):push(state,math.hypot(q[0]-s[0],q[1]-s[1])+.08*abs(wrap(q[2]-s[2])),None)
    # A subcell offset must not force a 45-degree turn at a narrow-corridor
    # start. Connect farther lattice anchors with the same full sweep check.
    for h in range(len(ANGLES)):
        for distance in (.25,.5,.75):
            a=ANGLES[h]
            state=(int((s[0]+distance*math.cos(a))/body.r),int((s[1]+distance*math.sin(a))/body.r),h)
            q=pose(state)
            if body.connect(s,q,q[2]):push(state,math.hypot(q[0]-s[0],q[1]-s[1])+.08*abs(wrap(q[2]-s[2])),None)
    heading_clearance={}
    expanded=0;end=None
    while queue:
        if time.monotonic()-begin>timeout:raise PlanningTimeout('机身约束搜索超时，未发布未经验证的路线')
        _,_,cost,state=heapq.heappop(queue)
        if cost!=dist[state]:continue
        expanded+=1;c,r,h=state;q=pose(state)
        prev=parent[state]
        # The reconstructed route omits in-place lattice states. Only finish
        # after a translation, so the incoming heading matches the last edge.
        if (prev is not None and prev[:2] != state[:2] and
                math.hypot(q[0]-t[0],q[1]-t[1])<.9 and body.connect(q,t,t[2])):
            end=state;break
        for dh in (-1,1):
            nh=(h+dh)%len(ANGLES);field=body.field(h if dh==1 else nh,True)
            if 0<=c<body.w and 0<=r<body.h and not field[r,c]:push((c,r,nh),cost+.10*abs(wrap(ANGLES[nh]-ANGLES[h]))/(math.pi/4),state)
        dc,dr=dirs[h];nc,nr=c+dc*step,r+dr*step
        if not (0<=nc<body.w and 0<=nr<body.h):continue
        field=body.field(h)
        if field[r,c] or field[nr,nc] or field[r+dr,c+dc]:continue
        # Exact centre-cell traversal is cheap and closes diagonal corner gaps.
        if not body.line_clear(q,pose((nc,nr,h)),q[2]):continue
        if h not in heading_clearance:
            heading_clearance[h]=cv2.distanceTransform((~field).astype(np.uint8),cv2.DIST_L2,3)*body.r
        clearance=body.clearance[nr,nc]
        soft=1.+2.*max(0.,(.80-clearance)/.80)**2+float(body.data[nr,nc])/252.*.5
        soft+=3.*max(0.,1.-heading_clearance[h][nr,nc]/.30)**2
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
    route=certified_dense_route(points,start,goal,body)
    if route is None:raise ValueError('离散执行路线未通过机身复核')
    return route,dict(planner='rectangular_lattice',expanded=expanded,compute_sec=time.monotonic()-begin,certified=True)


def dense_route(points, goal):
    # Densify straight certified segments for DWB, preserving exact corners.
    result=[]
    for a,b in zip(points,points[1:]):
        if math.dist(a,b)<1e-8:continue
        yaw=math.atan2(b[1]-a[1],b[0]-a[0]);n=max(1,math.ceil(math.dist(a,b)/.05))
        result.extend(dict(x=a[0]+(b[0]-a[0])*i/n,y=a[1]+(b[1]-a[1])*i/n,yaw=yaw) for i in range(n))
    result.append(dict(x=goal[0],y=goal[1],yaw=goal[2]))
    return result

def certified_dense_route(points,start,goal,body):
    route=dense_route(points,goal)
    xy=[[v['x'],v['y']] for v in route]
    ok,_=certify(xy,start,body,goal_yaw=goal[2],trim=False)
    return route if ok else None


def body_margin(body, local_xy, angles):
    """Sample rectangle edges against the distance field for ranking only."""
    lx=np.linspace(-body.l,body.l,11)
    wy=np.linspace(-body.b,body.b,7)[1:-1]
    x=np.r_[lx,lx,0*wy-body.l,0*wy+body.l]
    y=np.r_[0*lx-body.b,0*lx+body.b,wy,wy]
    c=np.cos(angles).reshape(-1,1);s=np.sin(angles).reshape(-1,1)
    cols=np.floor((local_xy[:,0,None]+c*x-s*y)/body.r).astype(int)
    rows=np.floor((local_xy[:,1,None]+s*x+c*y)/body.r).astype(int)
    if (cols<0).any() or (cols>=body.w).any() or (rows<0).any() or (rows>=body.h).any():
        return 0.
    return max(0.,float(body.clearance[rows,cols].min())-math.sqrt(2)*body.r)


def preview_body(grid, start, goal, half_length, half_width, timeout=8.):
    """Try direct and single-bend approaches before searching the heading lattice."""
    begin = time.monotonic()
    body = BodyGrid(grid, half_length, half_width)
    for pose, message in ((start, '起点机身覆盖实体障碍或未知区'), (goal, '目标机身或朝向与障碍冲突')):
        if not body.pose_clear(*body.local(pose)):
            raise ValueError(message)
    a, b = np.asarray(start[:2]), np.asarray(goal[:2])
    distance = np.linalg.norm(b-a)
    normal = np.array([a[1]-b[1], b[0]-a[0]]) / max(distance, 1e-9)
    directions = [normal, [math.cos(goal[2]), math.sin(goal[2])],
                  [math.cos(body.angle), math.sin(body.angle)], [-math.sin(body.angle), math.cos(body.angle)]]
    candidates = [[a,b]]
    for offset in (.55, 1., 1.5):
        for sign in (1,-1):
            for direction in directions:
                candidates.append([a,b+sign*offset*np.asarray(direction),b])
            candidates.append([a,(a+b)/2+sign*offset*normal,b])

    # Follow the free corridor backwards from the goal. A blocked section of
    # this ray also rules out every longer straight approach on the same ray.
    local_goal = body.local(goal)
    heading = local_goal[2]
    backward = -np.array([math.cos(heading), math.sin(heading)])
    previous = local_goal[:2]
    spacing = max(.1, 2*body.r)
    for offset in np.arange(spacing, min(distance, 10.)+1e-9, spacing):
        approach = local_goal[:2]+offset*backward
        if not body.line_clear(previous, approach, heading):
            break
        corner = body.world([*approach, heading])[:2]
        candidates.append([a, corner, b])
        previous = approach

    # Probe departures on both sides of the robot, including a certified
    # U-turn into the corridor it arrived through. Every leg is driven forward.
    local_start = body.local(start)
    step = max(.2, 4*body.r)
    departure_angles = [local_start[2]+math.pi, local_start[2], *ANGLES]
    for angle in departure_angles:
        if not body.turn_clear(local_start, angle):
            continue
        direction = np.array([math.cos(angle), math.sin(angle)])
        previous = local_start[:2]
        for offset in np.arange(step, min(distance, 8.)+1e-9, step):
            pivot = local_start[:2]+offset*direction
            if not body.line_clear(previous, pivot, angle):
                break
            candidates.append([a,body.world([*pivot,angle])[:2],b])
            previous = pivot

    # Score the complete centreline against real obstacles before running the
    # more expensive continuous rectangle and pivot checks on the best options.
    ranked = []
    for points in candidates:
        cost = 0.
        margin = .30
        previous_angle=start[2]-body.angle
        for p,q in zip(points,points[1:]):
            segment = math.dist(p,q)
            local = np.linspace(body.local([*p,0])[:2], body.local([*q,0])[:2],
                                max(2,math.ceil(segment/max(.1,body.r))+1))
            cells = np.floor(local/body.r).astype(int)
            x,y = cells.T
            if ((x<0)|(y<0)|(x>=body.w)|(y>=body.h)).any() or (body.data[y,x]>=253).any():
                break
            angle=math.atan2(q[1]-p[1],q[0]-p[0])-body.angle
            sweep=wrap(angle-previous_angle)
            turns=previous_angle+np.linspace(0.,sweep,max(2,math.ceil(abs(sweep)/.15)+1))
            margin=min(margin,body_margin(body,local,np.array([angle])),
                       body_margin(body,local[:1],turns))
            previous_angle=angle
            clearance = body.clearance[y,x]
            cost += segment*(1+4*np.mean(np.maximum(0.,1-clearance/.85)**2))
        else:
            sweep=wrap(goal[2]-body.angle-previous_angle)
            turns=previous_angle+np.linspace(0.,sweep,max(2,math.ceil(abs(sweep)/.15)+1))
            margin=min(margin,body_margin(body,local[-1:],turns))
            ranked.append((cost+18*(.30-margin),points))
    for _, points in sorted(ranked,key=lambda item:item[0]):
        if time.monotonic()-begin > min(1.,timeout/3):
            break
        ok,_ = certify(points,start,body,goal_yaw=goal[2],trim=False)
        if ok:
            route=certified_dense_route(points,start,goal,body)
            if route is None:continue
            return route,dict(planner='preview_direct' if len(points)==2 else 'preview_single_bend',
                                                 compute_sec=time.monotonic()-begin,certified=True)
    remaining = timeout-(time.monotonic()-begin)
    return plan_body(grid,start,goal,half_length,half_width,timeout=remaining)


def replan_body(grid, start, goal, length, width, reference, detail, snapshot_prefix):
    """Try both sides of the blocked section before the full lattice search."""
    begin = time.monotonic()
    body = BodyGrid(grid, length, width)
    try:
        if detail.get('reason') == 'swept_body_blocked' and len(reference) >= 2:
            remaining, _ = tracking_suffix(reference, start)
            route = np.vstack((start[:2], remaining[:,:2]))
            steps = np.linalg.norm(np.diff(route, axis=0), axis=1)
            route = route[np.r_[True, steps > 1e-6]]
            along = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
            hit = min(along[-1], detail['clear_prefix_m'] + length)
            join = min(along[-1], hit + 1.6)
            def at(distance):
                return np.array([np.interp(distance, along, axis) for axis in route.T])
            direction = at(min(along[-1],hit+.3)) - at(max(0.,hit-.3))
            normal = np.array([-direction[1],direction[0]]) / max(np.linalg.norm(direction),1e-9)
            choices = []
            for offset in (.45,-.45,.70,-.70,1.,-1.):
                lead, tail = max(0.,hit-.5), min(join,hit+.5)
                points = np.vstack((route[0], at(lead)+offset*normal, at(tail)+offset*normal,
                                    at(join), route[along>join]))
                # Rank by minimum clearance along the entire bypass, then length.
                samples = np.vstack([np.linspace(a,b,max(2,math.ceil(math.dist(a,b)/body.r)+1))
                                     for a,b in zip(points[:3],points[1:4])])
                cells = np.array([body.local([x,y,0])[:2]/body.r for x,y in samples]).astype(int)
                if ((cells<0).any() or (cells[:,0]>=body.w).any() or (cells[:,1]>=body.h).any()):continue
                clearance = body.clearance[cells[:,1],cells[:,0]].min()
                bypass_margin=.30
                previous_angle=start[2]-body.angle
                for p,q in zip(points[:3],points[1:4]):
                    if math.dist(p,q)<1e-8:continue
                    angle=math.atan2(q[1]-p[1],q[0]-p[0])-body.angle
                    local=np.linspace(body.local([*p,0])[:2],body.local([*q,0])[:2],
                                      max(2,math.ceil(math.dist(p,q)/.10)+1))
                    turns=previous_angle+np.linspace(0.,wrap(angle-previous_angle),20)
                    bypass_margin=min(bypass_margin,body_margin(body,local,np.array([angle])),
                                      body_margin(body,local[:1],turns))
                    previous_angle=angle
                choices.append((bypass_margin+.1*min(.85,clearance),-abs(offset),offset,points))
            for _,__,offset,points in sorted(choices,key=lambda item:item[:2],reverse=True):
                if time.monotonic()-begin>1.:break
                ok,_ = certify(points,start,body,goal_yaw=goal[2],trim=False)
                if ok:
                    route=certified_dense_route(points,start,goal,body)
                    if route is None:continue
                    return route,dict(planner='side_detour',offset_m=offset,
                                compute_sec=time.monotonic()-begin,certified=True)
        return plan_body(grid,start,goal,length,width,timeout=max(1.,8.-(time.monotonic()-begin)))
    except ValueError as error:
        # One compressed grid per failed search, scoped to its mission.
        filename = str(snapshot_prefix)+'.'+str(time.time_ns())+'.npz'
        try:
            meta=dict(start=list(map(float,start)),goal=list(map(float,goal)),half_length=length,half_width=width,
                      resolution=body.r,origin=grid['origin'],trigger=detail,error=str(error))
            np.savez_compressed(filename,grid=body.data,reference=np.asarray(reference),
                                metadata=json.dumps(meta,ensure_ascii=False))
        except (OSError,TypeError,ValueError) as snapshot_error:
            print('[REPLAN_SNAPSHOT_FAILED]',snapshot_error,flush=True)
        raise


def timed_replan_body(grid,start,goal,length,width,reference,detail,snapshot_prefix,submitted):
    started=time.monotonic()
    try:
        route,certificate=replan_body(grid,start,goal,length,width,reference,detail,snapshot_prefix)
    except ValueError as error:
        error.timing=dict(worker_queue_ms=(started-submitted)*1000,
                          planning_ms=(time.monotonic()-started)*1000)
        raise
    certificate['timing']=dict(worker_queue_ms=(started-submitted)*1000,
                               planning_ms=(time.monotonic()-started)*1000)
    return route,certificate
