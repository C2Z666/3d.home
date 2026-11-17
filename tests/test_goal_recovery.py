import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import MotionGuard,replan_errors,POSITION_TOLERANCE,POSITION_RELEASE,HeadingAlignment

class GoalRecoveryTests(unittest.TestCase):
 def test_planner_five_cm_endpoint_is_valid(self):
  reason,metrics=replan_errors((0,0),(.95,0),(0,0),(1,0),.1)
  self.assertEqual(reason,'');self.assertAlmostEqual(metrics['end_error_m'],.05)
 def test_bad_endpoint_and_moved_start_are_distinguished(self):
  self.assertEqual(replan_errors((0,0),(.89,0),(0,0),(1,0),.1)[0],'REPLAN_ENDPOINT_MISMATCH')
  self.assertEqual(replan_errors((0,0),(1,0),(.21,0),(1,0),.1)[0],'REPLAN_START_CHANGED')
 def test_approach_and_release_hysteresis(self):
  self.assertEqual(POSITION_TOLERANCE,.15);self.assertEqual(POSITION_RELEASE,.20)
  a=HeadingAlignment([.14,0,0],[0,0,1.],0,[0,0,0])
  self.assertEqual(a.command([.18,0,.5],1,[.04,0,.5])[1],'')
  self.assertIn('ALIGN_GOAL_SHIFT',a.command([.21,0,.5],1,[.07,0,.5])[1])
 def test_slow_real_progress_is_not_stall(self):
  g=MotionGuard()
  for i in range(101):
   self.assertEqual(g.check(.087,.021,(.010,-.005,.003),i*.1,(i*.001,0)),'')
 def test_no_progress_has_bounded_wait_and_reset(self):
  g=MotionGuard();self.assertEqual(g.check(.087,0,(0,0,0),0,(0,0)),'')
  self.assertEqual(g.check(.087,0,(0,0,0),4.9,(0,0)),'')
  self.assertIn('NO_MOTION_RESPONSE',g.check(.087,0,(0,0,0),5.1,(0,0)))
  g.reset();self.assertEqual(g.check(.087,0,(0,0,0),20,(0,0)),'')
 def test_gait_jitter_is_not_net_progress(self):
  g=MotionGuard()
  for i in range(50):self.assertEqual(g.check(.1,0,(0,0,0),i*.1,(.005*(i%2),0)),'')
  self.assertIn('NO_MOTION_RESPONSE',g.check(.1,0,(0,0,0),5.1,(0,0)))

if __name__=='__main__':unittest.main()
