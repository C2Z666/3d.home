import ast,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_control import scan_input_state

class ScanRecoveryTests(unittest.TestCase):
    def test_0415s_producer_stale_does_not_end_mission(self):
        self.assertEqual(scan_input_state(dict(ready=False,reason='',finite_input_points=69961))[:2],(False,True))
    def test_delayed_cloud_and_odometry_are_waitable(self):
        for reason in ('stale cloud','no odometry bracket; no extrapolation','odometry gap'):
            self.assertEqual(scan_input_state(dict(ready=False,reason=reason))[:2],(False,True))
    def test_recovered_scan_is_usable(self):
        self.assertEqual(scan_input_state(dict(ready=True,reason=''))[:2],(True,True))
    def test_explicit_sensor_fault_cannot_pass_even_with_old_ready_flag(self):
        for reason in ('raw cloud frame mismatch','FAST-LIO frame mismatch','too few finite raw points','expected scalar FLOAT32 x/y/z fields'):
            for ready in (True,False):
                self.assertEqual(scan_input_state(dict(ready=ready,reason=reason))[:2],(False,False))
    def test_supervisor_uses_recovery_classification(self):
        tree=ast.parse((ROOT/'scripts/navigate_to_goal.py').read_text(encoding='utf-8'))
        safe=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='localization_safe')
        assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='recoverable' for t in n.targets))
        self.assertIn('scan_input_state(scanhealth)[0]',ast.unparse(safe))
        self.assertIn('scan_wait',ast.unparse(assignment.value))
        self.assertNotIn("scanhealth.get('ready')",ast.unparse(assignment.value))

if __name__=='__main__':unittest.main()
