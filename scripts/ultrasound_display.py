"""Read-only ultrasonic display values; no motion or navigation interfaces."""
import math
MOUNTS={s:dict(x=.1792,y=y,yaw=a,fov=math.pi/3) for s,y,a in [('left',.1002,math.pi/2),('right',-.1002,-math.pi/2)]}
def reading(m):
    r=float(m.range)
    valid=math.isfinite(r) and .01<=r<=3 and (m.max_range<=m.min_range or m.min_range<=r<=m.max_range)
    raw=r if math.isfinite(r) and r>0 else None
    return dict(distance=r if valid else None,raw_distance=raw,status='valid' if valid else 'over_range' if raw is not None and raw>3 else 'no_echo' if math.isinf(r) else 'invalid')
def display(value,age,source_age):
    out=dict(value)
    out['message_fresh']=age<1 and -.1<=source_age<1
    out['fresh']=out['message_fresh'] and out['status']=='valid'
    return out
