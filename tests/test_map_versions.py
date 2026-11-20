import sys,tempfile,json,hashlib,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np,yaml
from map_versions import validate,install,catalog
from site_identity import verify
class Versions(unittest.TestCase):
 def setUp(self):
  self.t=tempfile.TemporaryDirectory();self.r=Path(self.t.name)
  for d in ['config','assets/v','maps']: (self.r/d).mkdir(parents=True)
  def put(p,b): (self.r/p).write_bytes(b)
  put('assets/v/cal.yaml',b'camera: test\n');h=hashlib.sha256((self.r/'assets/v/cal.yaml').read_bytes()).hexdigest()
  np.savez(self.r/'assets/v/bank.npz',calibration_sha256=h)
  put('assets/v/map.pgm',b'P5\n2 2\n255\n'+bytes([254]*4));put('assets/v/map.yaml',b'image: map.pgm\nresolution: 0.05\norigin: [1, 2, 0]\n')
  v=dict(label='test',map_id='new',visual_map='assets/v/bank.npz',calibration='assets/v/cal.yaml',maps={'lower':{'yaml':'assets/v/map.yaml','image':'assets/v/map.pgm'}})
  v['sha256']={str(p.relative_to(self.r)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in (self.r/'assets/v').iterdir()}
  put('config/map_versions.json',json.dumps({'active':'a','versions':{'a':v}}).encode());put('config/scenes.json',json.dumps({'default':'old','scenes':{'old':{'visual_map':'assets/output/demo/visual_map.npz'}}}).encode());put('config/site.yaml',b'entry: {x: 1}\nmaps_validated: true\n')
 def tearDown(self):self.t.cleanup()
 def test_identity_and_origin(self):
  install(self.r,'a');self.assertEqual(verify(self.r),'new');self.assertEqual(yaml.safe_load((self.r/'maps/lower.yaml').read_text())['origin'],[1,2,0]);self.assertIsNone(yaml.safe_load((self.r/'config/site.yaml').read_text())['entry'])
 def test_tamper_rejected(self):
  (self.r/'assets/v/map.pgm').write_bytes(b'changed')
  with self.assertRaises(ValueError):validate(self.r,'a')
 def test_reactivation_clears_same_map_stair_fault(self):
  (self.r/'work').mkdir()
  old={'map_id':'new','actual_floor':'floor2','active_nav_floor':'floor2','phase':'FAULT','target_floor':'floor2','connection_id':'stairs_A','base_z_by_floor':{'floor2':2.5},'reason':'old failure'}
  (self.r/'work/floor_state.json').write_text(json.dumps(old))
  install(self.r,'a')
  state=json.loads((self.r/'work/floor_state.json').read_text())
  self.assertEqual(state['actual_floor'],'lower')
  self.assertEqual(state['active_nav_floor'],'lower')
  self.assertEqual(state['phase'],'ON_FLOOR')
  self.assertIsNone(state['target_floor']);self.assertIsNone(state['connection_id'])
  self.assertEqual(state['base_z_by_floor'],{});self.assertEqual(state['reason'],'')
 def test_activation_uses_lower_map_not_registry_order(self):
  from floor_routes import reset_map_activation
  registry={'map_id':'new','initial_floor':'floor2','floors':{'floor2':{'map_key':'upper'},'floor1':{'map_key':'lower'}}}
  state=reset_map_activation(self.r,registry)
  self.assertEqual(state['actual_floor'],'floor1')
  self.assertEqual(state['active_nav_floor'],'floor1')
 def test_unknown_rejected(self):
  with self.assertRaises(ValueError):validate(self.r,'../unknown')
if __name__=='__main__':unittest.main()
