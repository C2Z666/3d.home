import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cloud_scan import LIO_TO_NAV
from geometry import calibration
class ScanAxisTests(unittest.TestCase):
 def test_measured_lio_basis_is_proper_rotation(self):
  self.assertTrue(np.allclose(LIO_TO_NAV@LIO_TO_NAV.T,np.eye(3)))
  self.assertAlmostEqual(np.linalg.det(LIO_TO_NAV),1.)
 def test_static_robot_base_is_z_up_after_conversion(self):
  # Measured stationary world<-base orientation from the live Orin diagnostic.
  O=np.array([[-.0125110264,-.9999211036,-.0011228313],[-.0020348637,-.0010974567,.9999973275],[-.9999196635,.0125132778,-.0020209729]])
  nav=LIO_TO_NAV@O
  self.assertGreater(nav[2,2],.99)
  self.assertLess(abs(nav[0,2]),.03)
  self.assertLess(abs(nav[1,2]),.03)
 def test_raw_ground_normal_maps_to_nav_up(self):
  _,_,B_L,_=calibration(Path(__file__).resolve().parents[1]/'config/calibration.yaml')
  raw=np.array([-1.,0.,0.])
  # User-confirmed relation: cloud z->body +X, cloud y->body +Y, cloud x->body -Z.
  expected=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
  self.assertTrue(np.allclose(B_L[:3,:3],expected,atol=2e-3))
  self.assertGreater((B_L[:3,:3]@raw)[2],.99)
  # FAST-LIO's measured world is Y-up; this is distinct from the body Z-up frame.
  self.assertGreater((LIO_TO_NAV@np.array([0.,1.,0.]))[2],.99)
if __name__=='__main__':unittest.main(verbosity=2)
