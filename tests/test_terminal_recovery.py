import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import HeadingAlignment, settled_goal_action
from terminal_feedback import TerminalApproach,TerminalYaw,terminal_turn_position_limit

class Recovery(unittest.TestCase):
 def test_stopping_does_not_consume_alignment_progress_timer(self):
  a=HeadingAlignment([0,0,0],[0,0,.11],0,[0,0,0])
  a.pause(.5);a.resume(20.)
  self.assertEqual(a.command([0,0,0],20.1,[0,0,0])[1],'')
  self.assertIn('ALIGN_NO_PROGRESS',a.command([0,0,0],32.,[0,0,0])[1])
 def test_small_angle_improvement_resets_progress(self):
  a=HeadingAlignment([0,0,0],[0,0,.12],0,[0,0,0])
  self.assertEqual(a.command([0,0,.011],11.,[0,0,.011])[1],'')
  self.assertEqual(a.command([0,0,.011],15.,[0,0,.011])[1],'')
 def test_near_target_does_not_require_progress(self):
  a=HeadingAlignment([0,0,0],[0,0,.095],0,[0,0,0])
  self.assertEqual(a.command([0,0,0],20.,[0,0,0]),(0.,''))
 def test_large_turn_requires_margin(self):
  self.assertLess(terminal_turn_position_limit(2.05),.342)
  self.assertEqual(terminal_turn_position_limit(2.05),.20)
  self.assertEqual(terminal_turn_position_limit(.15),.30)
 def test_data_pause_freezes_approach_budget_and_progress(self):
  a=TerminalApproach(0,[0,0,0]);a.command([0,0,0],[.27,0],0,1,0,.1)
  a.resume_after_pause(3.44)
  self.assertEqual(a.command([-.04,0,0],[.27,0],0,2,4.1,.1)[1],'ADVANCE')
if __name__=='__main__':unittest.main()
