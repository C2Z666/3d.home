import sys,unittest,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from geometry import calibration,imu_from_base,interpolate,Correction,planar

class GeometryTests(unittest.TestCase):
 def test_sensor_reference(self):
  _,_,B_L,_=calibration(ROOT/'config/calibration.yaml'); I_B=imu_from_base(ROOT/'config/calibration.yaml')
  self.assertTrue(np.allclose(np.linalg.det(B_L[:3,:3]),1)); self.assertTrue(np.allclose(np.linalg.det(I_B[:3,:3]),1))
  self.assertAlmostEqual(B_L[0,3],.366149882,places=6)
 def test_same_time_correction(self):
  C=np.eye(4); C[:3,3]=[5,2,1]; fusion=Correction()
  for t,x in ((1,0),(2,.1)):
   O=np.eye(4); O[0,3]=x; fusion.update(C@O,O,t)
  self.assertTrue(np.allclose(fusion.T,C))
  O=np.eye(4); O[0,3]=2; self.assertTrue(np.allclose((fusion.T@O)[:3,3],[7,2,1]))
 def test_no_single_image_initialization(self):
  c=Correction(); self.assertFalse(c.update(np.eye(4),np.eye(4),1)); self.assertIsNone(c.T)
 def test_reject_visual_jump(self):
  c=Correction(); c.update(np.eye(4),np.eye(4),1); c.update(np.eye(4),np.eye(4),2)
  V=np.eye(4); V[0,3]=10; self.assertFalse(c.update(V,np.eye(4),3)); self.assertAlmostEqual(c.T[0,3],0)
 def test_interpolation(self):
  A=np.eye(4); B=np.eye(4); B[0,3]=1
  self.assertAlmostEqual(interpolate([(0,A),(.1,B)],.05)[0,3],.5)
  with self.assertRaises(ValueError): interpolate([(0,A),(.1,B)],.2)
  with self.assertRaises(ValueError): interpolate([(0,A),(1,B)],.5)
 def test_planar_chain_preserves_xy_yaw(self):
  C=np.eye(4); C[:3,:3]=Rotation.from_euler('xyz',[.1,.2,.7]).as_matrix(); C[:3,3]=[2,4,2]
  O=np.eye(4); O[:3,:3]=Rotation.from_euler('xyz',[.2,.1,.3]).as_matrix(); O[:3,3]=[1,2,.1]
  M=C@O; P=planar(O); Q=planar(M); tf=Q@np.linalg.inv(P)
  self.assertTrue(np.allclose(tf@P,planar(M))); self.assertEqual(Q[2,3],0)
if __name__=='__main__': unittest.main(verbosity=2)
