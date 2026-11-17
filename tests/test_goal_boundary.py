import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import SettledGoalReview,MotionGuard
class GoalBoundaryTests(unittest.TestCase):
 def test_single_boundary_spike_returns_to_done_without_replan(self):
  r=SettledGoalReview()
  self.assertEqual(r.update([.203,0,0],[0,0,0],0,1,True),'WAIT')
  self.assertEqual(r.update([.19,0,0],[0,0,0],.2,2,True),'WAIT')
  self.assertEqual(r.update([.19,0,0],[0,0,0],1.3,3,True),'DONE')
 def test_persistent_outside_requires_one_second(self):
  r=SettledGoalReview()
  self.assertEqual(r.update([.203,0,0],[0,0,0],0,1,True),'WAIT')
  self.assertEqual(r.update([.203,0,0],[0,0,0],.9,2,True),'WAIT')
  self.assertEqual(r.update([.203,0,0],[0,0,0],1.1,3,True),'REPLAN')
 def test_duplicate_samples_do_not_complete_review(self):
  r=SettledGoalReview();r.update([.203,0,0],[0,0,0],0,1,True)
  self.assertEqual(r.update([.203,0,0],[0,0,0],2,1,True),'WAIT')
 def test_movement_restarts_stable_window(self):
  r=SettledGoalReview();r.update([.19,0,0],[0,0,0],0,1,True)
  r.update([.19,0,0],[0,0,0],.9,2,False)
  self.assertEqual(r.update([.19,0,0],[0,0,0],1.1,3,True),'WAIT')
  self.assertEqual(r.update([.19,0,0],[0,0,0],2.2,4,True),'DONE')
 def test_yaw_only_error_resumes_alignment(self):
  r=SettledGoalReview();r.update([.19,0,.3],[0,0,0],0,1,True)
  self.assertEqual(r.update([.19,0,.3],[0,0,0],1.1,2,True),'ALIGN_YAW')
 def test_recovery_reset_does_not_reuse_old_drift(self):
  g=MotionGuard();g.check(0,.3,(0,0,.2),0,(0,0))
  self.assertIn('ZERO_TRANSLATION_DRIFT',g.check(0,.3,(0,0,.2),8,(.153,0)))
  g.reset()
  self.assertEqual(g.check(0,.3,(0,0,.2),9,(.153,0)),'')
  self.assertEqual(g.check(0,.3,(0,0,.2),10,(.16,0)),'')
if __name__=='__main__':unittest.main()
