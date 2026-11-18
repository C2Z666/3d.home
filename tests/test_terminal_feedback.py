import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from terminal_feedback import FreshFeedback, TerminalApproach, TerminalYaw

class FeedbackTests(unittest.TestCase):
    def test_duplicates_do_not_recompute(self):
        f=FreshFeedback();calls=[]
        def calc():calls.append(1);return .1,'GO'
        f.update(1.,0.,calc);f.update(1.,.1,calc)
        self.assertEqual(len(calls),1)
        self.assertEqual(f.update(1.,.21,calc),(0.,'WAIT_SAMPLE'))
    def test_backward_stamp_stops(self):
        f=FreshFeedback();f.update(2.,0.,lambda:(.1,'GO'))
        self.assertEqual(f.update(1.,.1,lambda:(.1,'GO')),(0.,'STAMP_REGRESSION'))
    def test_effective_forward_request(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.25,0],0,1.,.1,.07),(.12,'ADVANCE'))
    def test_no_turn_to_chase_position(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[0,.25],0,1.,.1,.07)[1],'POSITION_NOT_FORWARD')
    def test_predict_braking(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.13,0],.1,1.,.1,.1)[1],'POSITION_BRAKE')
    def test_no_progress_finishes_position_attempt(self):
        a=TerminalApproach(0.,[0,0,0]);a.command([0,0,0],[.25,0],0,1.,.1,.07)
        self.assertEqual(a.command([0,0,0],[.25,0],0,5.,4.2,.07)[1],'POSITION_NO_PROGRESS')
    def test_position_time_limit_even_without_new_sample(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.25,0],0,1.,12.,.07)[1],'POSITION_BUDGET')
    def test_stale_source_cannot_move(self):
        self.assertEqual(TerminalYaw().command(1.,0.,1.,0.,.4),(0.,'WAIT_SAMPLE'))
    def test_yaw_minimum_and_cap(self):
        self.assertEqual(TerminalYaw().command(.11,0,1,0,.07)[0],.12)
        self.assertEqual(TerminalYaw().command(-2,0,1,0,.07)[0],-.25)
    def test_yaw_brakes_before_overshoot(self):
        self.assertEqual(TerminalYaw().command(.11,.25,1,0,.1)[0],0.)
    def test_yaw_fresh_feedback_changes_direction(self):
        f=TerminalYaw();self.assertGreater(f.command(.2,0,1,0,.05)[0],0)
        self.assertLess(f.command(-.2,0,2,.1,.05)[0],0)

    def test_lateral_error_is_corrected_not_abandoned(self):
        a=TerminalApproach(0.,[0,0,0])
        v,status=a.command([0,0,0],[.25,.04],.01,1.,.1,.08)
        self.assertEqual(status,'ADVANCE');self.assertGreater(a.wz,0);self.assertLessEqual(a.wz,.18)
    def test_slow_response_increases_bounded_request(self):
        a=TerminalApproach(0.,[0,0,0])
        for i in range(25):
            v,status=a.command([i*.001,0,0],[.25,0],.01,i*.1,i*.1,.08)
        self.assertEqual(status,'ADVANCE');self.assertAlmostEqual(v,.15)
    def test_fast_response_reduces_request(self):
        a=TerminalApproach(0.,[0,0,0]);a.request=.15
        for i in range(15):v,status=a.command([0,0,0],[.25,0],.10,i*.1,i*.1,.08)
        self.assertEqual(status,'ADVANCE');self.assertLess(v,.15)
    def test_stale_feedback_stops_both_axes(self):
        a=TerminalApproach(0.,[0,0,0]);a.command([0,0,0],[.25,.04],.01,1.,.1,.08)
        self.assertEqual(a.command([0,0,0],[.25,.04],.01,1.,.5,.5),(0.,'WAIT_SAMPLE'))
        self.assertEqual(a.wz,0.)
    def test_zero_response_finishes_without_infinite_boost(self):
        a=TerminalApproach(0.,[0,0,0])
        for i in range(43):v,status=a.command([0,0,0],[.25,0],0.,i*.1,i*.1,.08)
        self.assertEqual(status,'POSITION_NO_PROGRESS');self.assertEqual(v,0.)
    def test_synthetic_weak_plant_with_lateral_offset(self):
        import math
        a=TerminalApproach(0.,[0,0,0]);p=[0.,0.,0.];speed=0.
        for i in range(121):
            v,status=a.command(p,[.25,.04],speed,i*.1,i*.1,.08)
            if status!='ADVANCE':break
            speed=max(0.,v-.07)*.6
            p[0]+=speed*math.cos(p[2])*.1;p[1]+=speed*math.sin(p[2])*.1
            p[2]+=a.wz*.7*.1
        self.assertEqual(status,'POSITION_BRAKE')
        self.assertLess(math.hypot(.25-p[0],.04-p[1]),.14)
