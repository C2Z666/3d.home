import unittest,sys,tempfile,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mission_runtime import RecoveryBudget,MissionTrace,ZeroCommandWindow
class RuntimeTests(unittest.TestCase):
 def test_zero_feedback_bias_is_not_subtracted_or_treated_as_motion(self):
  w=ZeroCommandWindow();sdk=dict(command_zero=True,reported_forward_speed=.023,reported_lateral_speed=.006,reported_yaw_rate=0.)
  for i in range(26):r=w.update(i*.2,[0,0],sdk,[0,0,0],.02)
  self.assertAlmostEqual(r['sdk_mean'][0],.023);self.assertEqual(r['odom_net_m'],0.)
  self.assertIsNone(w.update(6,[.1,0],sdk,[0,0,0],.02))
 def test_real_displacement_is_reported_independently(self):
  w=ZeroCommandWindow();sdk=dict(command_zero=True,reported_forward_speed=.023,reported_lateral_speed=0.,reported_yaw_rate=0.)
  for i in range(26):r=w.update(i*.2,[0,0],sdk,[i*.004,0,0],.02)
  self.assertAlmostEqual(r['odom_net_m'],.1)
 def test_data_wait_does_not_exhaust_position_or_motion(self):
  b=RecoveryBudget()
  for i in range(8):b.begin_wait(i*2);b.end_wait(i*2+1)
  self.assertEqual(b.counts,dict(data_wait=8,position=0,motion=0))
  for _ in range(3):self.assertEqual(b.request('position'),'')
  self.assertIn('POSITION_RECOVERY_LIMIT',b.request('position'))
  self.assertEqual(b.request('motion'),'')
 def test_repeated_pause_does_not_restart_deadline(self):
  b=RecoveryBudget();b.begin_wait(0);b.begin_wait(29)
  self.assertIn('DATA_WAIT_TIMEOUT',b.wait_error(30))
  self.assertEqual(b.counts['data_wait'],1)
 def test_cumulative_wait_is_bounded(self):
  b=RecoveryBudget()
  for i in range(3):b.begin_wait(i*30);b.end_wait(i*30+20)
  self.assertIn('DATA_WAIT_BUDGET',b.wait_error(100))
 def test_route_is_bound_to_task_and_revision(self):
  with tempfile.TemporaryDirectory() as d:
   t=MissionTrace(d,'a');t.set_route([[0,0],[1,0]],'m','old');t.route_status('executing')
   t.sample(map_pose=[0,0,0],state='NAVIGATE');t.set_route([[0,0],[2,0]],'m','old');t.route_status('history');t.close()
   ui=json.loads((Path(d)/'a.telemetry.json').read_text())
   self.assertEqual(ui['route']['route_id'],'a:2');self.assertEqual(ui['route']['status'],'history')
   self.assertEqual(ui['trajectory'],[[0,0]])
 def test_disk_rotation_and_missing_measurements(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'a.telemetry.jsonl';p.write_bytes(b'x'*(4*1024*1024))
   t=MissionTrace(d,'a');t.sample(map_pose=[0,0,0],unavailable=None,invalid=float('nan'));t.close()
   self.assertTrue(p.with_suffix('.jsonl.1').exists());self.assertLess(p.stat().st_size,1000)
   row=json.loads(p.read_text());self.assertIsNone(row['unavailable']);self.assertIsNone(row['invalid'])
if __name__=='__main__':unittest.main()
