"""Static-map reference paths only; never sent to the robot controller."""
import heapq,math
from functools import lru_cache
import cv2,numpy as np
from scipy.ndimage import distance_transform_edt

@lru_cache(maxsize=4)
def clearance(png,resolution):
    image=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_GRAYSCALE)
    if image is None:raise ValueError('目标楼层底图无法解码')
    free=np.flipud(image)>=250
    return distance_transform_edt(np.pad(free,1,constant_values=False))[1:-1,1:-1]*resolution

def static_path(meta,png,start,goal,radius=.275):
    if not png:raise ValueError('目标楼层静态地图不可用')
    res=meta['resolution'];ox,oy,angle=meta['origin'];c,s=math.cos(angle),math.sin(angle)
    dist=clearance(png,res);h,w=dist.shape
    def cell(p):
        x,y=p['x']-ox,p['y']-oy
        return math.floor((c*x+s*y)/res),math.floor((-s*x+c*y)/res)
    a,b=cell(start),cell(goal)
    def valid(p):
        if not (0<=p[0]<w and 0<=p[1]<h):return False
        clearance_m=dist[p[1],p[0]]
        return clearance_m>=radius or (clearance_m>0 and math.hypot(p[0]-a[0],p[1]-a[1])*res<=.35)
    if not valid(a):raise ValueError('出口贴近静态障碍或未知区域，无法生成安全参考路线')
    if not valid(b):raise ValueError('二楼目标贴近静态障碍或未知区域，请向空旷处移动')
    heap=[(0.,a)];cost={a:0.};parent={};closed=set()
    while heap:
        _,u=heapq.heappop(heap)
        if u in closed:continue
        if u==b:break
        closed.add(u)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            v=u[0]+dx,u[1]+dy
            if not valid(v) or (dx and dy and (not valid((u[0]+dx,u[1])) or not valid((u[0],u[1]+dy)))):continue
            value=cost[u]+math.hypot(dx,dy)*(1+2*max(0.,(.4-dist[v[1],v[0]])/.4))
            if value>=cost.get(v,math.inf):continue
            cost[v]=value;parent[v]=u;heapq.heappush(heap,(value+math.hypot(v[0]-b[0],v[1]-b[1]),v))
    else:raise ValueError('出口与所选目标在静态地图上不连通')
    cells=[b]
    while cells[-1]!=a:cells.append(parent[cells[-1]])
    points=[]
    for x,y in reversed(cells):
        x=(x+.5)*res;y=(y+.5)*res;points.append([ox+c*x-s*y,oy+s*x+c*y])
    points[0]=[start['x'],start['y']]
    if len(points)==1:points.append([goal['x'],goal['y']])
    else:points[-1]=[goal['x'],goal['y']]
    return points
