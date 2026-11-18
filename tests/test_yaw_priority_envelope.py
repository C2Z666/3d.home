import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import settled_goal_action,HeadingAlignment
class Envelope(unittest.TestCase):
 def test_position_does_not_veto_yaw(self):
  self.assertEqual(settled_goal_action([.24,0,1],[0,0,0],.35),'ALIGN_YAW')
 def test_accept_approximate_position(self):
  self.assertEqual(settled_goal_action([.24,0,.05],[0,0,0],.35),'DONE')
 def test_boundary_retained(self):
  self.assertEqual(settled_goal_action([.36,0,.05],[0,0,0],.35),'REPLAN')
 def test_turn_past_old_boundary(self):
  a=HeadingAlignment([.18,0,1],[0,0,0],0,[0,0,0],position_limit=.35)
  w,e=a.command([.24,0,1],1,[.06,0,0]);self.assertLess(w,0);self.assertFalse(e)
 def test_anomalous_drift_retained(self):
  a=HeadingAlignment([.18,0,1],[0,0,0],0,[0,0,0],position_limit=.35)
  w,e=a.command([.31,0,1],1,[.13,0,0]);self.assertEqual(w,0);self.assertTrue(e.startswith('[ALIGN_ODOM_DRIFT]'))
