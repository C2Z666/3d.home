import importlib.util,sys,subprocess,unittest,os
from pathlib import Path
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('project_stop',root/'stop.py');stop=importlib.util.module_from_spec(spec);spec.loader.exec_module(stop)
class Shutdown(unittest.TestCase):
 def test_exact_scope(self):
  self.assertEqual(stop.classify(['python3',str(root/'scripts/cloud_scan.py')]),'scan')
  self.assertIsNone(stop.classify(['ssh','robot@host']))
  self.assertIsNone(stop.classify(['bash','-c','cat '+str(root/'scripts/cloud_scan.py')]))
  self.assertIsNone(stop.classify(['/opt/vendor/lidar']))
 def test_group_shutdown_preserves_unrelated(self):
  owned=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)',str(root/'scripts/cloud_scan.py')],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  other=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
  try:
   stop.terminate(owned.pid);owned.wait(timeout=2)
   self.assertIsNone(other.poll())
   stop.terminate(owned.pid)
  finally:
   if owned.poll() is None:owned.kill();owned.wait()
   other.terminate();other.wait()
if __name__=='__main__':unittest.main()
