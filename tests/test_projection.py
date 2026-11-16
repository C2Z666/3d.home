import sys,unittest,math
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cloud_scan import project,cloud_xyz
class Projection(unittest.TestCase):
 def setUp(self):self.cfg=dict(range_min_m=.3,range_max_m=8.,base_height_m=.47,obstacle_min_above_floor_m=.12,obstacle_max_above_floor_m=1.2)
 def test_floor_and_high_points_excluded(self):
  a=project(np.array([[1.,0,-.47],[2,0,0],[3,0,1.5],[1,0,float('nan')]]),np.eye(4),self.cfg)
  self.assertEqual(np.isfinite(a).sum(),1);self.assertAlmostEqual(a[360],2.)
 def test_extrinsic_translation_and_nearest_return(self):
  T=np.eye(4);T[0,3]=.4
  a=project(np.array([[1.,0,0],[2,0,0]]),T,self.cfg);self.assertAlmostEqual(a[360],1.4)
 def test_organized_cloud_with_row_padding(self):
  buf=bytearray(40);np.ndarray((2,3),dtype='<f4',buffer=buf,strides=(20,4))[:]=[[1,2,3],[4,5,6]]
  m=S(fields=[S(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')],point_step=12,row_step=20,width=1,height=2,is_bigendian=False,data=buf)
  self.assertTrue(np.array_equal(cloud_xyz(m),[[1,2,3],[4,5,6]]))
 def test_no_return_does_not_invent_free_observations(self):
  a=project(np.array([[1.,0,-.47]]),np.eye(4),self.cfg);self.assertTrue(np.isinf(a).all())
if __name__=='__main__':unittest.main(verbosity=2)
