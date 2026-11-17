import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigate_to_goal import WalkGate

class WalkGateTests(unittest.TestCase):
 def state(self,seq=1,**kw):
  return dict(dict(motion='Walk',motion_pending=False,state_sequence=seq,pose_request_id=1,pose_confirmed_id=1),**kw)
 def test_already_walk_needs_two_distinct_reports_without_request(self):
  gate=WalkGate()
  self.assertEqual(gate.update(self.state()),'wait')
  self.assertEqual(gate.update(self.state()),'wait')
  self.assertEqual(gate.update(self.state(2)),'ready')
  self.assertIsNone(gate.request_before)
 def test_transition_requested_once_and_confirmation_required(self):
  gate=WalkGate()
  self.assertEqual(gate.update(self.state(motion='StandUp')),'request')
  self.assertEqual(gate.update(self.state(2,motion='StandUp')),'wait')
  self.assertEqual(gate.update(self.state(3)),'wait')
  self.assertEqual(gate.update(self.state(4,pose_request_id=2)),'wait')
  self.assertEqual(gate.update(self.state(5,pose_request_id=2,pose_confirmed_id=2)),'wait')
  self.assertEqual(gate.update(self.state(6,pose_request_id=2,pose_confirmed_id=2)),'ready')
 def test_pending_transition_is_not_reissued_and_resets_stability(self):
  gate=WalkGate();gate.update(self.state())
  self.assertEqual(gate.update(self.state(2,motion_pending=True)),'wait')
  self.assertIsNone(gate.request_before)
  self.assertEqual(gate.update(self.state(3)),'wait')
  self.assertEqual(gate.update(self.state(4)),'ready')
 def test_invalid_feedback_resets_confirmation(self):
  gate=WalkGate();gate.update(self.state());gate.reset_confirmation()
  self.assertEqual(gate.update(self.state(2)),'wait')
  self.assertEqual(gate.update(self.state(3)),'ready')

if __name__=='__main__':unittest.main()
