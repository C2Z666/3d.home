import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from scan_pose_history import PoseHistory
h=PoseHistory()
def p(t,x=0,yaw=0):return dict(stamp=t,x=x,y=0,yaw=yaw)
h.add(p(1,0,math.radians(179)));h.add(p(1.2,2,math.radians(-179)))
r,e=h.at(1.1);assert abs(r['x']-1)<1e-8 and abs(abs(r['yaw'])-math.pi)<1e-8
assert h.at(.9)[0] is None and h.at(1.3)[0] is None
h.clear();h.add(p(1));h.add(p(2));assert h.at(1.5)[0] is None
h.add(p(.5));assert h.at(1)[0] is None
# Static world point (4,0) observed during a 90-degree/s turn.
h.clear();h.add(p(10,0,0));h.add(p(10.2,0,math.pi*.1))
r,_=h.at(10.1);a=math.pi*.05;x,y=4*math.cos(a),-4*math.sin(a)
assert abs(math.cos(r['yaw'])*x-math.sin(r['yaw'])*y-4)<1e-8
assert abs(math.sin(r['yaw'])*x+math.cos(r['yaw'])*y)<1e-8
print('PASS: translation, yaw wrap, missing brackets, gaps, reset, rotating static wall')

from web_view import ViewerState
import time
v=ViewerState(Path('.'),'test')
loc=dict(session='a',epoch=0,map_id='test',ready=True,tracking_valid=True)
v.put('localization',loc)
v.put('pose',p(10,0,0),10);v.put('pose',p(10.2,2,.2),10.2)
v.put('scan',[[4,0]],10.1)
out=v.snapshot(time.monotonic(),10.25)
assert abs(out['scan_pose']['x']-1)<1e-8 and out['scan_pose']['stamp']==10.1
v.put('localization',dict(loc,epoch=1))
assert v.snapshot(time.monotonic(),10.25)['scan_pose'] is None
v.put('pose',p(10.3),10.3);v.put('localization',dict(loc,tracking_valid=False))
assert v.snapshot(time.monotonic(),10.35)['scan_pose'] is None
print('PASS: ViewerState acquisition-time matching and reset/invalid history clearing')

from ultrasound_display import reading,display
from types import SimpleNamespace
for distance,status in [(1.2,'valid'),(3.1,'over_range'),(float('inf'),'no_echo'),(float('nan'),'invalid'),(0,'invalid')]:
    item=reading(SimpleNamespace(range=distance,min_range=0,max_range=0))
    assert item['status']==status
    assert not display(item,2,0)['fresh']
    assert not display(item,0,2)['fresh']
v.put('localization',loc)
v.put('pose',p(20),20);v.put('pose',p(20.2,2),20.2)
v.put('uss_left',reading(SimpleNamespace(range=1.2,min_range=0,max_range=0)),20.1)
out=v.snapshot(time.monotonic(),20.3)['ultrasound']['left']
assert out['fresh'] and out['map_ready'] and abs(out['map_pose']['x']-1)<1e-8
assert not v.snapshot(time.monotonic(),21.3)['ultrasound']['left']['map_ready']
print('PASS: ultrasound validity, stale hiding, acquisition-time arc pose')

v.put('uss_left',reading(SimpleNamespace(range=1.4,min_range=0,max_range=0)),20.3)
out=v.snapshot(time.monotonic(),20.35)['ultrasound']['left']
assert out['map_ready'] and out['map_stamp']==20.1 and out['map_distance']==1.2 and out['distance']==1.4
v.put('uss_left',reading(SimpleNamespace(range=float('inf'),min_range=0,max_range=0)),20.4)
assert not v.snapshot(time.monotonic(),20.45)['ultrasound']['left']['map_ready']
print('PASS: delayed pose uses matched measurement; no-echo clears previous arc')
