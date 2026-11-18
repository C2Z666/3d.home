import importlib.util,sys,subprocess,unittest,os
from pathlib import Path
root=Path(__file__).resolve().parents[1]
import types
os.environ['D1_PROJECT_ROOT']=str(root)
source=(root/'scripts/stop.sh').read_text().split("<<'PY_STOP_RUNTIME'\n",1)[1].rsplit('\nPY_STOP_RUNTIME',1)[0]
stop=types.ModuleType('project_stop')
exec(compile(source,str(root/'scripts/stop.sh'),'exec'),stop.__dict__)
class Shutdown(unittest.TestCase):
 def test_exact_scope(self):
  self.assertEqual(stop.classify(['python3',str(root/'scripts/cloud_scan.py')]),'scan')
  self.assertIsNone(stop.classify(['ssh','robot@host']))
  self.assertIsNone(stop.classify(['bash','-c','cat '+str(root/'scripts/cloud_scan.py')]))
  self.assertIsNone(stop.classify(['/opt/vendor/lidar']))
 def test_legacy_conflicts(self):
  self.assertEqual(stop.classify(['/home/robot/d1_airy_offline_ws/install/fast_lio/lib/fast_lio/fastlio_mapping']),'legacy')
  self.assertEqual(stop.classify(['python3','/home/robot/lmy/d1_stair_nav_phase3_orin_scan_axisfix_20261116/scripts/cloud_scan.py']),'legacy')
  self.assertIsNone(stop.classify(['python3','/opt/ros/humble/bin/ros2','launch','rslidar_sdk','zs_m1.launch.py']))
 def test_port_release(self):
  server=subprocess.Popen([sys.executable,'-c',"import socket,time;s=socket.socket();s.bind(('127.0.0.1',0));s.listen();print(s.getsockname()[1],flush=True);time.sleep(60)"],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,start_new_session=True)
  try:
   port=int(server.stdout.readline());self.assertIn(server.pid,stop.port_owners({port}));stop.free_ports({port});server.wait(timeout=2);self.assertFalse(stop.listening_inodes({port}))
  finally:
   if server.poll() is None:server.kill();server.wait()
   server.stdout.close()
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
