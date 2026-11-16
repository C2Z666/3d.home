import sys,unittest,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from stair_policy import StairPolicy
from geometry import calibration,imu_from_base,interpolate,Correction,planar

def status():
 return dict(sdk_connected=True,control_held=True,state_fresh=True,state_age_sec=.01,
 soft_stop=False,hardware_stop=False,nav_handoff_hold=False,flat_test_lock=False,
 motion_pending=False,command_zero=True,reported_forward_speed=0.,reported_lateral_speed=0.,reported_yaw_rate=0.,
 stair_speed_cap=.1,state_sequence=10,bridge_session='A',safety_epoch=0,motion='Walk',pose_request_id=1,pose_confirmed_id=1)
class StairTests(unittest.TestCase):
 def armed(self):
  p=StairPolicy(.05,3,0); s=status()
  self.assertEqual(p.update(s,0,0),'stop')
  s['state_sequence']=11; self.assertEqual(p.update(s,.1,0),'request_stair')
  self.assertEqual(p.update(s,.2,0),'stop')
  s.update(motion='Stair',state_sequence=12,pose_request_id=2,pose_confirmed_id=2)
  self.assertEqual(p.update(s,.3,0),'prompt'); return p,s
 def test_only_y_moves(self):
  p,s=self.armed(); self.assertEqual(p.update(s,1,0),'stop'); self.assertTrue(p.answer('y',1)); self.assertEqual(p.update(s,1.1,0),'forward')
 def test_old_state_not_confirmation(self):
  p=StairPolicy(.05,3,0); s=status(); p.update(s,0,0); s['state_sequence']=11; p.update(s,.1,0)
  s.update(motion='Stair',pose_request_id=2,pose_confirmed_id=2)
  self.assertEqual(p.update(s,.2,0),'stop'); self.assertEqual(p.state,'WAIT_MODE')
 def test_lost_heartbeat_latches_fault(self):
  p,s=self.armed(); p.answer('y',1); self.assertEqual(p.update(s,1.1,.6),'stop'); self.assertEqual(p.state,'FAULT'); self.assertEqual(p.update(s,1.2,0),'stop')
 def test_session_restart(self):
  p,s=self.armed(); s['bridge_session']='B'; self.assertEqual(p.update(s,.4,0),'stop'); self.assertEqual(p.state,'FAULT')
 def test_safety_epoch(self):
  p,s=self.armed(); s['safety_epoch']=1; p.update(s,.4,0); self.assertEqual(p.state,'FAULT')
 def test_actual_mode_loss(self):
  p,s=self.armed(); p.answer('y',1); s['motion']='Walk'; self.assertEqual(p.update(s,1.1,0),'stop'); self.assertEqual(p.state,'FAULT')
 def test_decline_and_eof(self):
  for answer in ('n',''):
   p,s=self.armed(); self.assertFalse(p.answer(answer,1)); self.assertEqual(p.state,'FAULT')
 def test_timeout_is_not_landing(self):
  p,s=self.armed(); p.answer('y',1); self.assertEqual(p.update(s,4.1,0),'stop'); self.assertEqual(p.state,'DONE'); self.assertIn('landing NOT inferred',p.reason)
 def test_two_distinct_stopped_states_required(self):
  p=StairPolicy(.05,3,0); s=status(); p.update(s,0,0); p.update(s,.1,0); self.assertEqual(p.state,'WAIT_READY')
 def test_lateral_movement_blocks_mode_change(self):
  p=StairPolicy(.05,3,0); s=status(); s['reported_lateral_speed']=.1
  self.assertEqual(p.update(s,0,0),'stop'); self.assertEqual(p.state,'WAIT_READY')
 def test_bad_parameters(self):
  for speed,duration in ((float('nan'),2),(.81,2),(.05,float('inf')),(.05,0)):
   with self.assertRaises(ValueError): StairPolicy(speed,duration,0)
 def test_speed_cap(self):
  p=StairPolicy(.1,3,0); s=status(); s['stair_speed_cap']=.05; p.update(s,0,0); self.assertEqual(p.state,'FAULT')
class GeometryTests(unittest.TestCase):
 def test_sensor_reference(self):
  _,_,B_L,_=calibration(ROOT/'config/calibration.yaml'); I_B=imu_from_base(ROOT/'config/calibration.yaml')
  self.assertTrue(np.allclose(np.linalg.det(B_L[:3,:3]),1)); self.assertTrue(np.allclose(np.linalg.det(I_B[:3,:3]),1))
  self.assertAlmostEqual(B_L[0,3],.366149882,places=6)
 def test_same_time_correction(self):
  C=np.eye(4); C[:3,3]=[5,2,1]; fusion=Correction()
  for t,x in ((1,0),(2,.1)):
   O=np.eye(4); O[0,3]=x; fusion.update(C@O,O,t)
  self.assertTrue(np.allclose(fusion.T,C))
  O=np.eye(4); O[0,3]=2; self.assertTrue(np.allclose((fusion.T@O)[:3,3],[7,2,1]))
 def test_no_single_image_initialization(self):
  c=Correction(); self.assertFalse(c.update(np.eye(4),np.eye(4),1)); self.assertIsNone(c.T)
 def test_reject_visual_jump(self):
  c=Correction(); c.update(np.eye(4),np.eye(4),1); c.update(np.eye(4),np.eye(4),2)
  V=np.eye(4); V[0,3]=10; self.assertFalse(c.update(V,np.eye(4),3)); self.assertAlmostEqual(c.T[0,3],0)
 def test_interpolation(self):
  A=np.eye(4); B=np.eye(4); B[0,3]=1
  self.assertAlmostEqual(interpolate([(0,A),(.1,B)],.05)[0,3],.5)
  with self.assertRaises(ValueError): interpolate([(0,A),(.1,B)],.2)
  with self.assertRaises(ValueError): interpolate([(0,A),(1,B)],.5)
 def test_planar_chain_preserves_xy_yaw(self):
  C=np.eye(4); C[:3,:3]=Rotation.from_euler('xyz',[.1,.2,.7]).as_matrix(); C[:3,3]=[2,4,2]
  O=np.eye(4); O[:3,:3]=Rotation.from_euler('xyz',[.2,.1,.3]).as_matrix(); O[:3,3]=[1,2,.1]
  M=C@O; P=planar(O); Q=planar(M); tf=Q@np.linalg.inv(P)
  self.assertTrue(np.allclose(tf@P,planar(M))); self.assertEqual(Q[2,3],0)
if __name__=='__main__': unittest.main(verbosity=2)
