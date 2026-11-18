import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import SettledGoalReview
from cloud_scan import project
import numpy as np
class Regression(unittest.TestCase):
 def test_sdk_reports_count_during_window(self):
  r=SettledGoalReview()
  for t,seq in [(0,327),(.5,328),(1.01,328)]:a=r.update([.178,0,.09],[0,0,0],t,t,True,seq,[0,0,0])
  self.assertEqual(a,'DONE')
 def test_one_sdk_report_cannot_finish(self):
  r=SettledGoalReview()
  for t in [0,.5,1.1]:a=r.update([.178,0,.09],[0,0,0],t,t,True,327,[0,0,0])
  self.assertEqual(a,'WAIT')
 def test_real_motion_does_not_finish(self):
  r=SettledGoalReview()
  for t in [0,.5,1.1]:a=r.update([.178,0,.09],[0,0,0],t,t,True,int(t*10),[t*.03,0,0])
  self.assertEqual(a,'WAIT')
 def cfg(self):return dict(range_min_m=.3,range_max_m=8.,base_height_m=.47,obstacle_min_above_floor_m=.12,obstacle_max_above_floor_m=1.2,floor_edge_support_filter=True)
 def test_isolated_floor_edge_removed(self):
  d={};r=project(np.array([[.895,.213,-.348]]),np.eye(4),self.cfg(),d)
  self.assertFalse(np.isfinite(r).any());self.assertEqual(d['floor_edge_rejected'],1)
 def test_supported_low_obstacle_retained(self):
  r=project(np.array([[.9,.21,-.348],[.91,.22,-.34],[.92,.21,-.33]]),np.eye(4),self.cfg())
  self.assertTrue(np.isfinite(r).any())
 def test_higher_single_point_retained(self):
  r=project(np.array([[.9,.21,-.30]]),np.eye(4),self.cfg());self.assertTrue(np.isfinite(r).any())
if __name__=='__main__':unittest.main()
