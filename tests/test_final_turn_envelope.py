import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import HeadingAlignment,MotionGuard
class FinalTurn(unittest.TestCase):
 def alignment(self):return HeadingAlignment([.145,0,0],[0,0,1],0,[0,0,0],position_limit=.35,yaw_priority=True)
 def test_twelve_cm_does_not_abort_final_yaw(self):
  a=self.alignment();w,e=a.command([.266,0,.8],2,[.121,0,.8]);self.assertEqual(e,'');self.assertGreater(w,0)
 def test_goal_envelope_still_stops(self):
  w,e=self.alignment().command([.351,0,.8],2,[.1,0,.8]);self.assertIn('ALIGN_GOAL_SHIFT',e)
 def test_odom_escape_cannot_be_hidden_by_visual(self):
  w,e=self.alignment().command([.15,0,.8],2,[.5,0,.8]);self.assertIn('ALIGN_ODOM_DRIFT',e)
 def test_departure_guard_preserved(self):
  a=HeadingAlignment([0,0,0],[0,0,1],0,[0,0,0],check_goal_position=False)
  self.assertIn('ALIGN_ODOM_DRIFT',a.command([.121,0,.8],2,[.121,0,.8])[1])
 def test_duplicate_guard_delegated_only_when_requested(self):
  for limit,expected in [(None,''),(.12,'ZERO_TRANSLATION_DRIFT')]:
   g=MotionGuard();g.check(0,.2,(0,0,.2),0,[0,0],zero_translation_limit=limit)
   e=g.check(0,.2,(0,0,.2),2.1,[.121,0],zero_translation_limit=limit)
   self.assertIn(expected,e) if expected else self.assertEqual(e,'')
if __name__=='__main__':unittest.main()
