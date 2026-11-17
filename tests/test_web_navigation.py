import unittest,sys,tempfile,time,json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch,MagicMock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from web_navigation import NavigationManager

class WebNavigationTests(unittest.TestCase):
 def setUp(self):
  work=Path(__file__).resolve().parents[1]/'work';work.mkdir(exist_ok=True)
  self.tmp=tempfile.TemporaryDirectory(dir=work);self.root=Path(self.tmp.name)
  (self.root/'config').mkdir();(self.root/'config/site.yaml').write_text('maps_validated: true\nscan_source_validated: true\n')
  self.snap=dict(navigation_ready=True,pose=dict(x=0.,y=0.,z=0.,yaw=0.),localization=dict(map_id='m',session='s',epoch=0),sdk_age=0.,sdk=dict(sdk_connected=True,control_held=True,state_fresh=True,soft_stop=False,hardware_stop=False))
  self.node=MagicMock();self.node.get_publishers_info_by_topic.return_value=[]
  self.m=NavigationManager(self.root,SimpleNamespace(node=self.node,snapshot=lambda:self.snap))
  self.plan=dict(map_id='m',localization_session='s',localization_epoch=0,scene='old',floor='lower',start=dict(x=0.,y=0.,yaw=0.),goal=dict(x=.5,y=0.,yaw=.2),points=[[0,0],[.5,0]],_path=[dict(x=0.,y=0.,yaw=0.),dict(x=.5,y=0.,yaw=.2)])
  self.token=self.m.register(self.plan)['plan_id'];self.data=dict(plan_id=self.token,confirmed=True,max_runtime=10.)
 def tearDown(self):self.tmp.cleanup()
 def test_confirmation_and_runtime_validation(self):
  for d in [dict(self.data,confirmed=False),dict(self.data,max_runtime=True),dict(self.data,max_runtime=0),dict(self.data,max_runtime=float('inf'))]:
   with self.assertRaises(ValueError):self.m.start(d)
 def test_expired_and_relocalized(self):
  self.snap['localization']['epoch']=1
  with self.assertRaisesRegex(ValueError,'身份'):self.m.start(self.data)
  self.snap['localization']['epoch']=0;self.m.plans[self.token]['created']-=61
  with self.assertRaisesRegex(ValueError,'过期'):self.m.start(self.data)
 def test_robot_moved(self):
  self.snap['pose']['x']=.2
  with self.assertRaisesRegex(ValueError,'起点'):self.m.start(self.data)
 def test_sdk_and_multiple_publishers(self):
  self.snap['sdk']['hardware_stop']=True
  with self.assertRaisesRegex(ValueError,'SDK'):self.m.start(self.data)
  self.snap['sdk']['hardware_stop']=False;self.node.get_publishers_info_by_topic.return_value=[object()]
  with self.assertRaisesRegex(ValueError,'发布者'):self.m.start(self.data)
 def test_server_path_consumed_once_and_cancel(self):
  with patch('site_identity.verify',return_value='m'),patch('web_navigation.threading.Thread'),patch('web_navigation.subprocess.Popen') as spawn:
   proc=spawn.return_value;proc.poll.return_value=None
   result=self.m.start(dict(self.data,goal={'x':999},max_runtime=12.))
   args=spawn.call_args.args[0]
   self.assertEqual(args[args.index('--max-runtime')+1],'12.0')
   route=json.loads(Path(args[args.index('--path-file')+1]).read_text())
   self.assertEqual(route['poses'][-1]['x'],.5)
   self.assertNotIn(self.token,self.m.plans)
   with self.assertRaisesRegex(ValueError,'已有'):self.m.start(self.data)
   with self.assertRaisesRegex(ValueError,'编号'):self.m.stop({'task_id':'wrong'})
   self.m.stop({'task_id':result['task']['task_id']});proc.send_signal.assert_called_once()

if __name__=='__main__':unittest.main()
