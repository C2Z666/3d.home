import copy,json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from floor_routes import load_registry,candidate_routes,read_state,write_state,floor_id
from stair_completion import LandingDetector
from manual_initialization import alignment
from fusion_policy import FusionPolicy
from geometry import navigation_pose
from cloud_pairing import CloudQueue
from multifloor_preview import MultiFloorPreview
from types import SimpleNamespace


class FloorTests(unittest.TestCase):
    def test_waiting_new_cloud_does_not_overwrite_pairable_cloud(self):
        q=CloudQueue();q.add(1.,'paired');q.add(1.1,'waiting')
        self.assertEqual(q.take(1.2,.9,1.05),'paired')
        self.assertIsNone(q.take(1.25,.9,1.05))
        self.assertEqual(q.take(1.3,.9,1.15),'waiting')
        q.add(1.2,'expired');self.assertIsNone(q.take(1.8,1.,1.8))
    def setUp(self):
        self.scene=dict(map_id='d1_hall_100832_hall_clean',maps={'lower':{},'upper':{}})
        self.registry=load_registry(ROOT,self.scene)
        self.c=copy.deepcopy(self.registry['connections'][0])
        self.c['exit']=None;self.c.pop('arrival_line',None)  # Unmarked stair fixture.
    def test_registry_and_parallel_connections(self):
        self.assertEqual(floor_id(self.registry,'upper'),'floor2')
        other=copy.deepcopy(self.c);other['id']='B';other['entry']['x']=100
        self.registry['connections'].append(other)
        routes=candidate_routes(self.registry,'floor1','floor2',self.c['entry'])
        self.assertEqual(len(routes),2);self.assertEqual(routes[0][0]['id'],self.c['id'])
        self.assertEqual(candidate_routes(self.registry,'floor2','floor1',self.c['entry']),[])
        self.assertEqual(candidate_routes(self.registry,'floor1','floor1',self.c['entry']),[[]])
    def test_preview_only_selects_stair_for_explicit_transition(self):
        preview=object.__new__(MultiFloorPreview);preview.state=SimpleNamespace(root=ROOT);preview.scene='old'
        preview.maps={'old':{'upper':(dict(origin=[-10,-10,0],resolution=.05,width=1000,height=1000),None)}}
        preview.snapshot=lambda:dict(pose=dict(x=0,y=0))
        flat=dict(ok=True,goal=self.c['entry'],start=dict(x=0,y=0),floor='lower',points=[[0,0]],_path=[self.c['entry']])
        with patch('multifloor_preview.scene_bundle',return_value=self.scene),patch('preview_planner.PreviewPlanner.plan',return_value=copy.deepcopy(flat)):
            ordinary=preview.plan(dict(self.c['entry'],scene='old',floor='lower'))
            self.assertNotIn('mission',ordinary)
        with patch('multifloor_preview.scene_bundle',return_value=self.scene),patch('preview_planner.PreviewPlanner.plan',return_value=copy.deepcopy(flat)) as plan:
            cross=preview.plan(dict(x=1,y=2,yaw=0,scene='old',floor='upper'))
            self.assertEqual(cross['mission']['target_floor'],'floor2')
            self.assertEqual(cross['goal']['x'],1)
            self.assertEqual(plan.call_args.args[0]['floor'],'lower')
            self.assertEqual(cross['mission']['connections'][0]['exit']['floor_id'],'floor2')
    def test_state_does_not_leak_between_maps(self):
        with tempfile.TemporaryDirectory() as d:
            write_state(d,self.registry,actual_floor=None,phase='STAIR')
            self.assertIsNone(read_state(d,self.registry)['actual_floor'])
            other=copy.deepcopy(self.registry);other['map_id']='other'
            self.assertEqual(read_state(d,other)['actual_floor'],'floor1')
    def sample(self,t,x=0,z=0,**extra):
        return dict(stamp=t,x=x,y=0,z=z,yaw=0,roll=0,pitch=0,connection_id=self.c['id'],**extra)
    def climb_and_level(self,detector,**extra):
        detector.update(self.sample(0))
        detector.update(self.sample(1,x=1,z=.5,**extra))
        result=False
        for i in range(1,31):result=detector.update(self.sample(1+i*.1,x=1+i*.04,z=.5,**extra))
        return result
    def test_initial_flat_body_never_completes(self):
        d=LandingDetector(self.c)
        for i in range(60):self.assertFalse(d.update(self.sample(i*.1,x=i*.04)))
    def test_unmarked_exit_is_not_a_coordinate_requirement(self):
        d=LandingDetector(self.c)
        self.assertTrue(self.climb_and_level(d))
        self.assertFalse(d.evidence['coordinate_enabled']);self.assertIsNone(d.evidence['coordinate_pass'])
    def test_preview_exit_is_not_a_completion_coordinate(self):
        self.c['preview_exit']=dict(floor_id='floor2',x=100,y=100,yaw=0)
        d=LandingDetector(self.c)
        self.assertTrue(self.climb_and_level(d));self.assertFalse(d.evidence['coordinate_enabled'])
    def test_arrival_line_requires_target_floor_and_stable_posture(self):
        self.c['arrival_line']=dict(x=2,y=0,yaw=0,floor_id='floor2',half_width_m=.6)
        for fid,expected in [('floor1',False),('floor2',True)]:
            d=LandingDetector(self.c);d.line=dict(self.c['arrival_line'],connection_id=self.c['id'])
            self.assertEqual(self.climb_and_level(d,candidate_floor=fid),expected)
        d=LandingDetector(self.c)
        self.assertFalse(self.climb_and_level(d,candidate_floor='floor2'),'Configured but unbound line must fail closed')
    def test_exit_marker_never_becomes_stop_target(self):
        self.c['exit']=dict(x=100,y=100,yaw=0,floor_id='floor2')
        d=LandingDetector(self.c);self.assertTrue(self.climb_and_level(d))
        self.assertFalse(d.evidence['coordinate_enabled'])
    def test_wrong_connection_never_completes(self):
        d=LandingDetector(self.c);self.climb_and_level(d)
        s=self.sample(5,x=2,z=.5);s['connection_id']='ordinary_navigation'
        self.assertFalse(d.update(s))
    def test_stale_samples_cannot_accumulate_stability(self):
        d=LandingDetector(self.c);d.update(self.sample(0));d.update(self.sample(1,x=1,z=.5))
        for i in range(5):self.assertFalse(d.update(self.sample(2+i,x=2+i*.1,z=.5)))
    def test_manual_alignment_leaves_raw_odometry_and_tilt_intact(self):
        O=np.eye(4);O[:3,:3]=Rotation.from_euler('xyz',[.02,.03,.8]).as_matrix();O[:3,3]=[1,2,3]
        before=O.copy();request=dict(map_id='m',x=8,y=4,z=1.7,yaw=1.7518)
        T=alignment(request,O,'m');M=T@O
        np.testing.assert_allclose(O,before);np.testing.assert_allclose(M[:3,3],[8,4,1.7])
        self.assertAlmostEqual(math.atan2(M[1,0],M[0,0]),1.7518)
        np.testing.assert_allclose(M[:3,:3][2],navigation_pose(O)[:3,:3][2],atol=1e-9)
        f=FusionPolicy();f.initialize_manual(T,100)
        self.assertEqual(f.epoch,1);self.assertEqual(f.alignment_source,'manual');self.assertIsNone(f.last_visual)
        self.assertFalse(f.update(M,O,99))
        for bad in (float('nan'),True):
            with self.assertRaises(ValueError):alignment(dict(request,x=bad),O,'m')

if __name__=='__main__':unittest.main()
