import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from clearing_evidence import ClearingEvidence
from route_validation import check_route,ReplanPolicy


class Repairs(unittest.TestCase):
    def test_clearing_requires_repeated_actual_rays(self):
        c=ClearingEvidence();T=np.eye(4);rays=np.array([[3.,0,0],[3.,0,-1.]])
        for t in (0,.1,.2):self.assertEqual(len(c.update(rays,t,T)),0)
        out=c.update(rays,.3,T);self.assertEqual(len(out),2)
        self.assertTrue(np.all(np.linalg.norm(out,axis=1)<np.linalg.norm(rays,axis=1).max()))
        self.assertEqual(len(c.update(np.empty((0,3)),.4,T)),0)
        self.assertEqual(len(c.update(rays,.5,T)),0)

    def test_closer_occluder_limits_ray_and_motion_resets(self):
        c=ClearingEvidence();T=np.eye(4)
        for t in (0,.1,.2):c.update(np.array([[3.,0,0]]),t,T)
        out=c.update(np.array([[1.,0,0]]),.3,T)
        self.assertLess(out[0,0],1.)
        T[0,3]=.03;self.assertEqual(len(c.update(np.array([[3.,0,0]]),.4,T)),0)

    def test_missing_beam_cannot_inherit_free_space(self):
        c=ClearingEvidence();T=np.eye(4)
        c.update(np.array([[3.,0,0]]),0,T)
        c.update(np.array([[0.,3,0]]),.1,T)
        c.update(np.array([[3.,0,0]]),.2,T)
        self.assertEqual(len(c.update(np.array([[3.,0,0]]),.3,T)),0)
        self.assertEqual(len(c.update(np.array([[3.,0,0]]),1.,T)),0)

    def grid(self):return dict(width=100,height=100,resolution=.05,origin=[-2.,-2.,0.],data=np.zeros((100,100),np.uint8))

    def test_body_collision_without_centre_collision(self):
        g=self.grid();g['data'][44,60]=254 # x=1.025, y=.225, inside body.
        ok,why=check_route([[0,0],[2,0]],[0,0,0],g)
        self.assertFalse(ok);self.assertEqual(why['centre_cost'],0)
        g['data'][44,60]=252;self.assertTrue(check_route([[0,0],[2,0]],[0,0,0],g)[0])
        g['data'][44,60]=255;self.assertFalse(check_route([[0,0],[2,0]],[0,0,0],g)[0])

    def test_obstacle_behind_passed_path_does_not_replan(self):
        g=self.grid();g['data'][40,40]=254
        self.assertTrue(check_route([[0,0],[1,0],[2,0]],[1,0,0],g)[0])

    def test_cooldown_and_bounded_replans(self):
        p=ReplanPolicy();p.blocked(0);self.assertFalse(p.ready(1));self.assertTrue(p.ready(3.))
        self.assertTrue(p.request(3.));p.blocked(4);self.assertFalse(p.ready(3.9));self.assertTrue(p.ready(7.))
        for t in (7.,10.,13.,16.,19.,22.,25.):self.assertTrue(p.request(t))
        self.assertFalse(p.request(28))

if __name__=='__main__':unittest.main()
