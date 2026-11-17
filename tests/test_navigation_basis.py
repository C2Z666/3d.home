import unittest,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from geometry import navigation_pose,planar,LIO_TO_NAV

class NavigationBasisTests(unittest.TestCase):
 def test_ground_motion_and_yaw(self):
  for yaw in [-1.,0.,1.]:
   desired=np.eye(4);desired[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();desired[:3,3]=[2.,3.,.4]
   raw=np.eye(4);raw[:3,:3]=LIO_TO_NAV.T@desired[:3,:3];raw[:3,3]=LIO_TO_NAV.T@desired[:3,3]
   np.testing.assert_allclose(navigation_pose(raw),desired,atol=1e-12)
 def test_map_pose_preserved_by_tf_chain(self):
  raw=np.eye(4);raw[:3,:3]=Rotation.from_euler('xyz',[1.5,.01,1.]).as_matrix();raw[:3,3]=[1,2,3]
  fusion=np.eye(4);fusion[:3,:3]=Rotation.from_euler('xyz',[-1.5,.1,.4]).as_matrix()
  P=planar(navigation_pose(raw));Q=planar(fusion@raw)
  np.testing.assert_allclose((Q@np.linalg.inv(P))@P,Q,atol=1e-12)
 def test_vertical_noise_does_not_become_horizontal_heading(self):
  for tilt in [-.003,0,.003]:
   desired=np.eye(4);desired[:3,:3]=Rotation.from_euler('xyz',[tilt,tilt,.7]).as_matrix()
   raw=np.eye(4);raw[:3,:3]=LIO_TO_NAV.T@desired[:3,:3]
   got=planar(navigation_pose(raw))
   self.assertAlmostEqual(np.arctan2(got[1,0],got[0,0]),.7,places=10)

if __name__=='__main__':unittest.main()
