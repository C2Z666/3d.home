import math
import sys
import unittest
import json
import tempfile
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from body_planner import BodyGrid, plan_body, certify, replan_body, ANGLES, preview_body


def grid(a):
    return dict(width=a.shape[1],height=a.shape[0],resolution=.05,origin=[0,0,0],data=a.ravel().tolist())


class BodyPlannerTests(unittest.TestCase):
    def test_detours_and_independent_polygon_oracle(self):
        a=np.zeros((100,120),np.uint8);a[35:65,55:65]=254
        start=[1,2.5,0];goal=[5,2.5,0]
        path,_=plan_body(grid(a),start,goal)
        xy=[[p['x'],p['y']] for p in path]
        self.assertTrue(certify(xy,start,grid(a),goal_yaw=0)[0])
        self.assertGreater(max(abs(p['y']-2.5) for p in path),.75)
        # Independent polygon intersection, including dense intermediate turns.
        obstacle=np.float32([[2.75,1.75],[3.25,1.75],[3.25,3.25],[2.75,3.25]])
        corners=np.array([[-.5,-.275],[.5,-.275],[.5,.275],[-.5,.275]])
        previous=0.
        for p,q in zip(path,path[1:]):
            yaw=math.atan2(q['y']-p['y'],q['x']-p['x'])
            delta=math.atan2(math.sin(yaw-previous),math.cos(yaw-previous))
            poses=[(p['x'],p['y'],previous+f*delta) for f in np.linspace(0,1,max(2,math.ceil(abs(delta)/.01)))]
            poses += [(p['x']+f*(q['x']-p['x']),p['y']+f*(q['y']-p['y']),yaw) for f in np.linspace(0,1,8)]
            for x,y,angle in poses:
                c,s=math.cos(angle),math.sin(angle)
                poly=np.float32(corners@np.array([[c,s],[-s,c]])+[x,y])
                area,_=cv2.intersectConvexConvex(poly,obstacle)
                self.assertLessEqual(area,1e-8)
            previous=yaw

    def test_narrow_corridor_and_goal_orientation(self):
        a=np.full((80,120),254,np.uint8);a[33:47,:]=0
        path,_=plan_body(grid(a),[1,2,0],[5,2,0])
        self.assertTrue(certify([[p['x'],p['y']] for p in path],[1,2,0],grid(a),goal_yaw=0)[0])
        with self.assertRaises(ValueError):plan_body(grid(a),[1,2,0],[5,2,math.pi/2])
        self.assertFalse(BodyGrid(grid(a)).turn_clear([2,2,0],math.pi/2))

    def test_shallow_headings_and_certified_side_detour(self):
        gaps=np.diff([*ANGLES,2*math.pi])
        self.assertLess(np.max(gaps),math.radians(27))
        a=np.zeros((100,140),np.uint8)
        a[49:52,59:62]=254
        # Force the detour onto the open side of the passage.
        a[:37,:]=254
        g=grid(a);start=[1.,2.5,0.];goal=[5.,2.5,0.]
        ref=np.linspace(start[:2],goal[:2],81)
        ok,detail=certify(ref,start,g)
        self.assertFalse(ok)
        with tempfile.TemporaryDirectory() as directory:
            path,info=replan_body(g,start,goal,.50,.275,ref,detail,Path(directory)/'round')
        self.assertEqual(info['planner'],'side_detour')
        self.assertGreater(info['offset_m'],0.)
        self.assertTrue(certify([[p['x'],p['y']] for p in path],start,g,goal_yaw=0.)[0])

    def test_preview_bends_around_corner(self):
        a=np.zeros((100,160),np.uint8)
        a[49:60,72:79]=254
        start=[1.,2.5,0.]; goal=[6.,2.5,math.pi/2]
        g=grid(a)
        self.assertFalse(certify([start[:2],goal[:2]],start,g,goal_yaw=goal[2])[0])
        path,info=preview_body(g,start,goal,.50,.275)
        self.assertEqual(info['planner'],'preview_single_bend')
        self.assertTrue(certify([[p['x'],p['y']] for p in path],start,g,goal_yaw=goal[2])[0])

    def test_preview_long_final_corridor(self):
        cells=np.full((140,180),254,np.uint8)
        cells[23:57,10:158]=0
        cells[23:125,125:158]=0
        start=[1.4,2.,0.];goal=[7.05,5.4,math.pi/2];g=grid(cells)
        path,info=preview_body(g,start,goal,.50,.275)
        self.assertEqual(info['planner'],'preview_single_bend')
        points=[[p['x'],p['y']] for p in path]
        self.assertTrue(certify(points,start,g,goal_yaw=goal[2])[0])
        final_leg=[p for p in path if abs(p['yaw']-goal[2])<1e-6]
        self.assertGreater(goal[1]-final_leg[0]['y'],2.)

    def test_preview_turn_back_out_of_corridor(self):
        cells=np.full((140,180),254,np.uint8)
        cells[23:57,10:158]=0
        cells[23:125,125:158]=0
        start=[7.05,5.4,math.pi/2];goal=[1.4,2.,math.pi];g=grid(cells)
        path,info=preview_body(g,start,goal,.50,.275)
        self.assertEqual(info['planner'],'preview_single_bend')
        self.assertLess(math.cos(path[0]['yaw']-start[2]),-.9)
        self.assertTrue(certify([[p['x'],p['y']] for p in path],start,g,goal_yaw=goal[2])[0])

    def test_failed_search_snapshot(self):
        a=np.zeros((80,100),np.uint8);a[40,20]=254
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                replan_body(grid(a),[1,2,0],[3,2,0],.5,.275,[],{},Path(directory)/'round')
            with np.load(next(Path(directory).glob('round.*.npz'))) as snapshot:
                np.testing.assert_array_equal(snapshot['grid'],a)
                self.assertEqual(json.loads(str(snapshot['metadata']))['start'],[1,2,0])

    def test_cost_categories_and_unknown(self):
        a=np.zeros((80,100),np.uint8);a[20:25,20:35]=253
        body=BodyGrid(grid(a))
        self.assertTrue(body.pose_clear(1.5,1.4,0))
        self.assertFalse(body.pose_clear(1.5,1.1,0))
        a[20:25,20:35]=255
        self.assertFalse(BodyGrid(grid(a)).pose_clear(1.5,1.4,0))

    def test_rotated_map(self):
        a=np.zeros((100,120),np.uint8);g=grid(a);g['origin']=[3,-2,.4]
        b=BodyGrid(g);start=b.world([1,2,0]);goal=b.world([4,2,0])
        path,_=plan_body(g,start,goal)
        self.assertTrue(certify([[p['x'],p['y']] for p in path],start,g,goal_yaw=goal[2])[0])


if __name__=='__main__':unittest.main()
