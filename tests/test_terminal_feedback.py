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
        self.assertEqual(a.command([0,0,0],[.25,0],0,1.,.1,.07),(.1,'ADVANCE'))
    def test_no_turn_to_chase_position(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[0,.25],0,1.,.1,.07)[1],'POSITION_NOT_FORWARD')
    def test_predict_braking(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.13,0],.1,1.,.1,.1)[1],'POSITION_BRAKE')
    def test_no_progress_finishes_position_attempt(self):
        a=TerminalApproach(0.,[0,0,0]);a.command([0,0,0],[.25,0],0,1.,.1,.07)
        self.assertEqual(a.command([0,0,0],[.25,0],0,3.,2.2,.07)[1],'POSITION_NO_PROGRESS')
    def test_position_time_limit_even_without_new_sample(self):
        a=TerminalApproach(0.,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.25,0],0,1.,6.,.07)[1],'POSITION_BUDGET')
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
