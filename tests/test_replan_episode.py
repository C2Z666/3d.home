import sys,unittest,ast,copy
from unittest.mock import Mock
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from route_validation import ReplanPolicy,check_route,bounded_route,near_obstacle_action,ClearWindow
from body_planner import PlanningTimeout,remaining_points,progress_index,tracking_suffix
import numpy as np,pickle

class Episodes(unittest.TestCase):
 def test_resume_restarts_canceled_check_with_fresh_map(self):
  source=Path(__file__).resolve().parents[1]/'scripts/navigate_to_goal.py'
  tree=ast.parse(source.read_text())
  function=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='restart_interrupted_check'))
  # Execute the production recovery function with fake ROS futures/services.
  function.body=[ast.Global(n.names) if isinstance(n,ast.Nonlocal) else n for n in function.body]
  module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
  for pending_map in (None,Mock()):
   for pending_check in (None,Mock()):
    for body in (None,Mock()):
     request=Mock();costmap=Mock()
     env=dict(route_future=pending_map,check_future=pending_check,check_context=('old',),
              route_last_valid=True,route_next_check=100.,body_future=body,
              request_route_check=request,route_costmap=costmap)
     exec(compile(module,str(source),'exec'),env)
     env['restart_interrupted_check'](10.)
     self.assertIsNone(env['route_last_valid']);self.assertIsNone(env['check_context'])
     self.assertIsNone(env['route_future']);self.assertIsNone(env['check_future'])
     self.assertEqual(env['route_next_check'],10.)
     if body is None:request.assert_called_once_with(10.)
     else:request.assert_not_called()
     if pending_map is not None:costmap.remove_pending_request.assert_called_once_with(pending_map)
     if pending_check is not None:pending_check.cancel.assert_called_once()
     if pending_map is not None:pending_map.reset_mock()
     if pending_check is not None:pending_check.reset_mock()
 def test_bounded_route_preserves_turns_and_distance(self):
  points,complete=bounded_route([[1,0],[1,1],[1,3]],[0,0,0])
  self.assertEqual(points,[[1.,0.],[1.,1.],[1.,2.]])
  self.assertFalse(complete)
 def test_bounded_route_limits_points_without_skipping(self):
  source=[[i*.01,0] for i in range(1,201)]
  points,complete=bounded_route(source,[0,0,0])
  self.assertEqual(points,source[:80]);self.assertFalse(complete)
 def test_bounded_route_detects_goal_in_range(self):
  points,complete=bounded_route([[1,0],[1,1]],[0,0,0])
  self.assertEqual(points,[[1.,0.],[1.,1.]]);self.assertTrue(complete)
 def test_background_path_does_not_turn_back_to_passed_waypoint(self):
  points,index=remaining_points([[0,0],[.1,0],[.2,0]],[.04,0,0])
  self.assertEqual(index,1);self.assertGreater(points[0,0],.04)
 def test_passed_obstacle_and_corner_rejoin(self):
  a=np.zeros((100,140),np.uint8);a[40,20]=254
  grid=dict(width=140,height=100,resolution=.05,origin=[0.,0.,0.],data=a.ravel())
  points=[[x,2.] for x in np.arange(.5,4.01,.05)]+[[4.,2.05],[4.,2.1]]
  cursor=0
  for x in np.arange(.5,3.93,.1):
   cursor=progress_index(points,[x,2.,0.],cursor)
  pos=[3.94,2.04,0.]
  tail,j=tracking_suffix(points[cursor:],pos)
  self.assertAlmostEqual(tail[0,0],4.)
  self.assertAlmostEqual(tail[0,1],2.)
  self.assertTrue(check_route(points[cursor:],pos,grid)[0])
  self.assertGreaterEqual(progress_index(points,[3.82,2.,0.],cursor),cursor)
 def test_near_obstacle_early_clear_and_deadline(self):
  self.assertEqual(near_obstacle_action(1.,5.,True),"resume")
  self.assertEqual(near_obstacle_action(4.99,5.,False),"wait")
  self.assertEqual(near_obstacle_action(5.,5.,False),"replan")
  self.assertEqual(near_obstacle_action(5.,5.,None),"wait")
 def test_confirm_then_new_block(self):
  w=ClearWindow()
  for t,stamp in [(3,10),(3.25,11)]:self.assertFalse(w.observe(t,stamp,stamp,True))
  self.assertTrue(w.observe(3.5,12,12,True))
  self.assertFalse(w.observe(4.,13,13,False))
  self.assertEqual(near_obstacle_action(.5,5.,False),'wait')
 def test_flicker_duplicate_and_gap(self):
  w=ClearWindow()
  for t,m,s,ok in [(0,1,1,True),(.25,2,2,False),(.4,3,3,True),(.7,4,3,True),
                    (1.1,5,4,True),(1.3,5,5,True)]:
   self.assertFalse(w.observe(t,m,s,ok))
  self.assertEqual(near_obstacle_action(5.,5.,False),'replan')
 def test_three_second_obstacle_wait(self):
  p=ReplanPolicy();p.blocked(0)
  self.assertFalse(p.ready(2.99));self.assertTrue(p.ready(3.))
  p.cleared();self.assertFalse(p.ready(5))
 def test_new_path_alone_cannot_reset_count_but_progress_can(self):
  p=ReplanPolicy();p.blocked(0)
  for t in range(3,25,3):self.assertTrue(p.request(t))
  self.assertFalse(p.request(27));p.cleared();self.assertEqual(p.attempts,8)
  self.assertFalse(p.progress(28,5.));self.assertFalse(p.progress(30,4.4))
  self.assertTrue(p.progress(31,4.4));self.assertEqual(p.attempts,0)
  p.blocked(32);self.assertTrue(p.request(35))
 def test_stall_budget_excludes_contiguous_wait_phases(self):
  p=ReplanPolicy();p.blocked(10);p.set_waiting(29,True);p.set_waiting(34,True)
  self.assertFalse(p.stall_error(140));p.set_waiting(140,False)
  self.assertFalse(p.stall_error(180.99));self.assertTrue(p.stall_error(181))
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
