import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from route_validation import ReplanPolicy,check_route
from body_planner import PlanningTimeout,remaining_points
import numpy as np,pickle

class Episodes(unittest.TestCase):
 def test_background_path_does_not_turn_back_to_passed_waypoint(self):
  points,index=remaining_points([[0,0],[.1,0],[.2,0]],[.04,0,0])
  self.assertEqual(index,1);self.assertGreater(points[0,0],.04)
 def test_three_second_obstacle_wait(self):
  p=ReplanPolicy();p.blocked(0)
  self.assertFalse(p.ready(2.99));self.assertTrue(p.ready(3.))
  p.cleared();self.assertFalse(p.ready(5))
 def test_new_path_alone_cannot_reset_count_but_progress_can(self):
  p=ReplanPolicy();p.blocked(0)
  for t in (3,6,9,12):self.assertTrue(p.request(t))
  self.assertFalse(p.request(15));p.cleared();self.assertEqual(p.attempts,4)
  self.assertFalse(p.progress(16,5.));self.assertFalse(p.progress(18,4.4))
  self.assertTrue(p.progress(19,4.4));self.assertEqual(p.attempts,0)
  p.blocked(20);self.assertTrue(p.request(23))
 def test_waiting_or_oscillation_is_not_progress(self):
  p=ReplanPolicy();p.blocked(0);p.request(3)
  p.progress(4,5);p.progress(10,5.1);self.assertEqual(p.attempts,1)
  p.progress(11,0,False);self.assertFalse(p.progress(30,4))
  self.assertEqual(p.attempts,1)
 def test_compute_timeout_refunded_but_stall_budget_remains(self):
  p=ReplanPolicy();p.blocked(0);p.request(3);p.timeout(11)
  self.assertEqual(p.attempts,0);p.blocked(11)
  self.assertFalse(p.ready(15));self.assertTrue(p.ready(16))
  self.assertIn('STALL_TIMEOUT',p.stall_error(60))
  self.assertIsInstance(pickle.loads(pickle.dumps(PlanningTimeout('test'))),PlanningTimeout)
 def test_background_timeout_does_not_refund_previous_failure(self):
  p=ReplanPolicy();p.blocked(0);p.request(3)
  p.timeout(12,charged=False);self.assertEqual(p.attempts,1)
 def test_collision_logs_actual_body_cell_not_only_centre(self):
  a=np.zeros((100,100),np.uint8);a[54,70]=254
  grid=dict(width=100,height=100,resolution=.05,origin=[-2.5,-2.5,0.],data=a.ravel())
  ok,info=check_route([[0,0],[.3,0],[.6,0],[1.,0]],[0,0,0],grid)
  self.assertFalse(ok);self.assertEqual(info['centre_cost'],0)
  self.assertEqual(info['blocked']['cost'],254)
  self.assertEqual(info['blocked']['cell'],[70,54])
  self.assertIn('clear_prefix_m',info)
if __name__=='__main__':unittest.main()
