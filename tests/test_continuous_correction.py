"""Regression: delayed visual matches, large turns and time-based correction."""
import math,sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fusion_policy import FusionPolicy
from navigation_control import HeadingAlignment

def pose(x=0.,y=0.,yaw=0.):
    M=np.eye(4);M[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();M[:2,3]=[x,y];return M

def initialized():
    f=FusionPolicy()
    for t in (0.,.4,.8):f.update(pose(),pose(),t)
    f.advance(pose(),1.)
    return f

class ContinuousCorrectionTests(unittest.TestCase):
    def test_camera_updates_never_directly_replace_live_pose(self):
        f=initialized();old=f.T.copy()
        for t in (1.1,1.5,1.9):f.update(pose(.5,0,.5),pose(),t)
        np.testing.assert_allclose(f.T,old)
        f.advance(pose(),1.91)
        self.assertLessEqual(np.linalg.norm(f.T[:3,3]),.003001)
        self.assertLessEqual(Rotation.from_matrix(f.T[:3,:3]).magnitude(),math.radians(.150001))

    def test_equal_wall_time_not_camera_frame_count(self):
        def run(interval):
            f=initialized()
            for i in range(1,101):
                t=1.+i*.01
                if i==1 or i%interval==0:f.update(pose(.14,0,.08),pose(),t)
                f.advance(pose(),t)
            return f.T
        np.testing.assert_allclose(run(20),run(5),atol=1e-10)

    def test_no_matching_stops_correction_but_odometry_keeps_moving(self):
        f=initialized();f.update(pose(.1),pose(),1.1)
        for i in range(1,101):f.advance(pose(),1.+i*.01)
        f.advance(pose(),3.)
        old=f.T.copy();f.advance(pose(2.,0,1.),20.)
        np.testing.assert_allclose(f.T,old)
        self.assertFalse(f.required);self.assertIsNone(f.target)
        self.assertGreater((f.T@pose(2.))[0,3],2.)

    def test_delayed_visual_uses_capture_pose_not_current_robot_position(self):
        f=initialized();C=pose(.1,0,.05)
        old_odom=pose(1.,0,.2);current_odom=pose(1.5,.1,.8)
        self.assertTrue(f.update(C@old_odom,old_odom,1.2))
        np.testing.assert_allclose(f.target,C,atol=1e-12)
        before=f.T@current_odom;f.advance(current_odom,1.7);after=f.T@current_odom
        self.assertLessEqual(np.linalg.norm(after[:3,3]-before[:3,3]),.003001)

    def test_large_odom_origin_does_not_amplify_turn_correction(self):
        f=initialized();O=pose(1000.,-500.,2.7)
        V=O.copy();V[:3,:3]=Rotation.from_euler('z',2.75).as_matrix()
        self.assertTrue(f.update(V,O,1.1))
        for i in range(1,31):
            O=pose(1000.+i*.01,-500.,2.7+i*.01)
            before=f.T@O;f.advance(O,1.1+i*.01);after=f.T@O
            self.assertLessEqual(np.linalg.norm(after[:3,3]-before[:3,3]),.003001 if i==1 else .000301)
            self.assertLessEqual(Rotation.from_matrix(before[:3,:3].T@after[:3,:3]).magnitude(),math.radians(.150001 if i==1 else .015001))

    def test_reacquisition_after_long_dropout_does_not_jump(self):
        f=initialized();old=f.T.copy();f.expire(100.,5.)
        for t in (100.,100.4,100.8):f.update(pose(.7,0,.3),pose(),t)
        np.testing.assert_allclose(f.T,old)
        f.advance(pose(),100.81)
        self.assertLessEqual(f.T[0,3],.003001);self.assertEqual(f.epoch,0)

    def test_visual_shift_is_not_physical_turn_drift(self):
        a=HeadingAlignment([0,0,0],[0,0,2.5],0,[10,5,1.],check_goal_position=False)
        self.assertEqual(a.command([.3,.2,1.],5,[10,5,2.])[1],'')
        d=a.diagnostics([.3,.2,1.],[10,5,2.])
        self.assertEqual(d['odom_drift_m'],0.)
        self.assertAlmostEqual(d['map_alignment_shift_m'],math.hypot(.3,.2))
        self.assertIn('ALIGN_ODOM_DRIFT',a.command([.3,.2,1.],5,[10.13,5,2.])[1])

    def test_final_goal_shift_requests_reacquisition_not_odom_fault(self):
        a=HeadingAlignment([0,0,0],[0,0,2.5],0,[10,5,1.])
        self.assertIn('ALIGN_GOAL_SHIFT',a.command([.16,0,1.],5,[10,5,2.])[1])

if __name__=='__main__':unittest.main()
