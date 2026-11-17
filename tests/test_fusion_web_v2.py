import sys, unittest, tempfile, shutil, json, time, threading
from pathlib import Path
from urllib.request import urlopen, Request
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from fusion_policy import FusionPolicy
from site_identity import verify,verify_entry
from scene_registry import scene_bundle
from web_view import ViewerState,map_assets,handler
from visual_localizer import matching_settings,active_matching_profile
def pose(x=0,yaw=0):
    t=np.eye(4);t[0,3]=x;t[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();return t
def initialized():
    c=FusionPolicy()
    for t in (1.,1.4,1.8):c.update(pose(),pose(),t)
    return c
class FusionTests(unittest.TestCase):
    def test_relaxed_visual_profile_only_relaxes_pnp_candidate_gates(self):
        strict=matching_settings('strict');relaxed=matching_settings('relaxed')
        self.assertEqual(relaxed['min_matches'],12)
        self.assertEqual(relaxed['min_inliers'],12)
        self.assertLess(relaxed['min_inlier_ratio'],strict['min_inlier_ratio'])
        self.assertEqual(strict['min_matches'],16)
        self.assertEqual(strict['max_reprojection_error_px'],2.5)

    def test_relaxed_profile_is_limited_to_initial_localization(self):
        self.assertEqual(active_matching_profile(False,'relaxed'),'relaxed')
        self.assertEqual(active_matching_profile(True,'relaxed'),'strict')
        with self.assertRaises(ValueError):matching_settings('bootstrap')
    def test_three_distinct_frames_and_time_window(self):
        c=FusionPolicy();self.assertFalse(c.update(pose(),pose(),1))
        self.assertFalse(c.update(pose(),pose(),1));self.assertFalse(c.update(pose(),pose(),20))
        self.assertFalse(c.update(pose(),pose(),20.4));self.assertTrue(c.update(pose(),pose(),20.8))
    def test_moving_robot_compensated(self):
        c=FusionPolicy();C=pose(4,.2)
        for t in (1.,1.4,1.8):
            O=pose(t,.1*t);c.update(C@O,O,t)
        self.assertTrue(np.allclose(c.T,C))
    def test_inconsistent_initial_candidates(self):
        c=FusionPolicy()
        for i in range(10):self.assertFalse(c.update(pose(i%2),pose(),1+i*.3))
        self.assertIsNone(c.T)
    def test_small_update_smoothed_at_robot(self):
        c=initialized();O=pose(100)
        c.advance(O,2.2)
        self.assertTrue(c.update(pose(100.1),O,2.2))
        self.assertAlmostEqual((c.T@O)[0,3],100.)
        c.advance(O,2.3)
        self.assertAlmostEqual((c.T@O)[0,3],100.003);self.assertEqual(c.epoch,0)
    def test_large_correction_checks_consistency_without_locking(self):
        c=initialized();before=c.T.copy();self.assertFalse(c.update(pose(.39),pose(),2.2))
        self.assertFalse(c.required);self.assertEqual(c.epoch,0);self.assertTrue(np.allclose(c.T,before))
        self.assertFalse(c.update(pose(.39),pose(),2.6))
        self.assertTrue(c.update(pose(.39),pose(),3.0))
        self.assertAlmostEqual(c.T[0,3],0.);self.assertFalse(c.required)
        c.advance(pose(),3.0);c.advance(pose(),3.1)
        self.assertAlmostEqual(c.T[0,3],.003)
        self.assertTrue(c.update(pose(.39),pose(),3.4))
        self.assertLessEqual(c.T[0,3],.003001)
    def test_conflicting_large_matches_do_not_change_alignment(self):
        c=initialized()
        for i in range(10):self.assertFalse(c.update(pose(1 if i%2 else -1),pose(),2+i*.3))
        self.assertAlmostEqual(c.T[0,3],0.);self.assertFalse(c.required)
    def test_large_yaw_correction_is_bounded(self):
        c=initialized()
        for t in (2.2,2.6,3.0):c.update(pose(0,1.),pose(),t)
        self.assertLessEqual(Rotation.from_matrix(c.T[:3,:3]).magnitude(),np.deg2rad(1.5)+1e-8)
    def test_operator_recovery_three_frames(self):
        c=initialized();c.invalidate('jump');c.request()
        for t in (3.,3.4,3.8):c.update(pose(2),pose(),t)
        self.assertFalse(c.required);self.assertFalse(c.recovering);self.assertEqual(c.epoch,1)
        self.assertAlmostEqual(c.T[0,3],2)
    def test_visual_dropout_keeps_alignment_and_recovers_without_request(self):
        c=initialized();before=c.T.copy();epoch=c.epoch;c.expire(100.,5.)
        self.assertTrue(c.visual_stale);self.assertFalse(c.required)
        self.assertEqual(c.epoch,epoch);self.assertTrue(np.allclose(c.T,before))
        self.assertTrue(c.update(pose(.05),pose(),101.))
        self.assertFalse(c.visual_stale);self.assertFalse(c.required)
        self.assertEqual(c.epoch,epoch)
    def test_failed_manual_recovery_does_not_latch(self):
        c=initialized();c.request();c.invalidate('robot moved')
        self.assertFalse(c.required);self.assertFalse(c.recovering)
        self.assertTrue(c.update(pose(),pose(),3.))
    def test_bad_candidate_does_not_refresh_visual_age(self):
        c=initialized();last=c.last_visual;c.update(pose(1),pose(),2.2)
        self.assertEqual(c.last_visual,last)
class ViewerTests(unittest.TestCase):
    def test_identity_bound(self):
        import yaml
        self.assertEqual(verify(ROOT),scene_bundle(ROOT,'old')['map_id'])
        s=yaml.safe_load((ROOT/'config/site.yaml').read_text());verify_entry(ROOT,s)
        s.update(entry=dict(x=1.,y=2.,yaw=0.),entry_map_id=verify(ROOT));verify_entry(ROOT,s)
        s['entry_map_id']='wrong'
        with self.assertRaises(ValueError):verify_entry(ROOT,s)
    def test_current_vs_stale_and_odom_only(self):
        s=ViewerState(ROOT,'A');s.put('localization',dict(map_id='A',tracking_valid=True,ready=False))
        s.put('pose',dict(x=0,y=0),100)
        a=s.snapshot(time.monotonic(),100);self.assertTrue(a['pose_current']);self.assertFalse(a['navigation_ready'])
        self.assertFalse(s.snapshot(time.monotonic()+2,102)['pose_current'])
        self.assertFalse(s.snapshot(time.monotonic(),110)['pose_current'])
    def test_http_exposes_scene_entry_and_rejects_state_write(self):
        mid=verify(ROOT);s=ViewerState(ROOT,mid);bundle=scene_bundle(ROOT,'old');maps=map_assets(ROOT,bundle)
        bundle['entry']=dict(x=1.25,y=.5,yaw=.2)
        h=ThreadingHTTPServer(('127.0.0.1',0),handler(s,{'old':maps},{},{'old':[]},lambda:100,{'old':bundle},'old'))
        t=threading.Thread(target=h.serve_forever,daemon=True);t.start()
        base='http://127.0.0.1:'+str(h.server_port)
        try:
            data=json.load(urlopen(base+'/state'));self.assertEqual(data['entry'],bundle['entry'])
            self.assertFalse(data['navigation_ready']);self.assertEqual(set(data['maps']),set(bundle['maps']))
            self.assertEqual(urlopen(base+'/map.png?floor=lower').headers['Content-Type'],'image/png')
            with self.assertRaises(HTTPError) as e:urlopen(Request(base+'/state',data=b'{}',method='POST'))
            self.assertEqual(e.exception.code,404)
            with self.assertRaises(HTTPError):urlopen(base+'/reference.jpg?bank=-1')
        finally:h.shutdown();h.server_close();t.join()
if __name__=='__main__':unittest.main(verbosity=2)
