import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import MotionGuard
class ZeroTranslationDriftTests(unittest.TestCase):
 def test_observed_slow_backward_motion_is_detected(self):
  g=MotionGuard();reason=''
  for i in range(70):
   reason=g.check(0,-.021,(-.025,0,0),i*.1,(-.0025*i,0))
   if reason:break
  self.assertIn('ZERO_TRANSLATION_DRIFT',reason)
  self.assertLess(i*.1,6.)
 def test_gait_oscillation_not_accumulated_as_distance(self):
  g=MotionGuard()
  for i in range(400):self.assertEqual(g.check(0,-.021,(-.02,0,0),i*.1,((i%2)*.015,0)), '')
 def test_normal_rotation_at_fixed_position(self):
  g=MotionGuard()
  for i in range(200):self.assertEqual(g.check(0,.3,(0,0,.25),i*.1,(0,0)), '')
 def test_forward_command_resets_zero_displacement_anchor(self):
  g=MotionGuard();g.check(0,0,(0,0,0),0,(0,0));g.check(.1,0,(.1,0,0),1,(.2,0))
  self.assertEqual(g.check(0,0,(0,0,0),3,(.3,0)), '')
 def test_reset_clears_zero_displacement_history(self):
  g=MotionGuard();g.check(0,0,(0,0,0),0,(0,0));g.reset()
  self.assertEqual(g.check(0,0,(0,0,0),10,(.3,0)), '')
 def test_low_translation_route_circle_is_bounded(self):
  g=MotionGuard();reason=''
  for i in range(100):
   reason=g.check(.008,-.4,(0,0,-.4),i*.1,(0,0),-.04*i)
   if reason:break
  self.assertIn('ROUTE_SPIN',reason);self.assertLess(i*.1,9.)
 def test_normal_heading_alignment_can_rotate_more_than_half_circle(self):
  g=MotionGuard()
  for i in range(100):self.assertEqual(g.check(0,.3,(0,0,.3),i*.1,(0,0)), '')
 def test_translation_resets_route_spin_accumulator(self):
  g=MotionGuard();g.check(.008,.4,(0,0,.4),0,(0,0),0)
  g.check(.008,.4,(0,0,.4),1,(0,0),2.)
  g.check(.1,.4,(.1,0,.4),2,(.1,0),2.)
  self.assertEqual(g.check(.008,.4,(0,0,.4),3,(.1,0),-2.),'')
if __name__=='__main__':unittest.main()
