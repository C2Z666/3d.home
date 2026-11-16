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
        self.assertTrue(c.update(pose(100.1),O,2.2))
        self.assertAlmostEqual((c.T@O)[0,3],100.025);self.assertEqual(c.epoch,0)
    def test_large_update_does_not_enter_navigation(self):
        c=initialized();self.assertFalse(c.update(pose(.39),pose(),2.2))
        self.assertTrue(c.required);self.assertEqual(c.epoch,1);self.assertAlmostEqual(c.T[0,3],0)
        for t in (3.,3.4,3.8):self.assertFalse(c.update(pose(.39),pose(),t))
    def test_operator_recovery_three_frames(self):
        c=initialized();c.invalidate('jump');c.request()
        for t in (3.,3.4,3.8):c.update(pose(2),pose(),t)
        self.assertFalse(c.required);self.assertFalse(c.recovering);self.assertEqual(c.epoch,2)
        self.assertAlmostEqual(c.T[0,3],2)
    def test_expiration_latches_until_request(self):
        c=initialized();c.expire(7.,5.)
        self.assertTrue(c.required);self.assertFalse(c.update(pose(),pose(),8.))
    def test_bad_candidate_does_not_refresh_visual_age(self):
        c=initialized();last=c.last_visual;c.update(pose(1),pose(),2.2)
        self.assertEqual(c.last_visual,last)
class ViewerTests(unittest.TestCase):
    def test_identity_bound(self):
        import yaml
        self.assertTrue(verify(ROOT).startswith('retained_'))
        s=yaml.safe_load((ROOT/'config/site.yaml').read_text());verify_entry(ROOT,s)
        s['entry_map_id']='wrong'
        with self.assertRaises(ValueError):verify_entry(ROOT,s)
    def test_current_vs_stale_and_odom_only(self):
        s=ViewerState(ROOT,'A');s.put('localization',dict(map_id='A',tracking_valid=True,ready=False))
        s.put('pose',dict(x=0,y=0),100)
        a=s.snapshot(time.monotonic(),100);self.assertTrue(a['pose_current']);self.assertFalse(a['navigation_ready'])
        self.assertFalse(s.snapshot(time.monotonic()+2,102)['pose_current'])
        self.assertFalse(s.snapshot(time.monotonic(),110)['pose_current'])
    def test_http_is_read_only_and_entry_is_persisted(self):
        mid=verify(ROOT);s=ViewerState(ROOT,mid);bundle=scene_bundle(ROOT,'old');maps=map_assets(ROOT,bundle)
        h=ThreadingHTTPServer(('127.0.0.1',0),handler(s,{'old':maps},{},{'old':[]},lambda:100,{'old':bundle},'old'))
        t=threading.Thread(target=h.serve_forever,daemon=True);t.start()
        base='http://127.0.0.1:'+str(h.server_port)
        try:
            data=json.load(urlopen(base+'/state'));self.assertAlmostEqual(data['entry']['x'],1.9245870099355233)
            self.assertFalse(data['navigation_ready']);self.assertIn('upper',data['maps'])
            self.assertEqual(urlopen(base+'/map.png?floor=lower').headers['Content-Type'],'image/png')
            with self.assertRaises(HTTPError) as e:urlopen(Request(base+'/state',data=b'{}',method='POST'))
            self.assertEqual(e.exception.code,501)
            with self.assertRaises(HTTPError):urlopen(base+'/reference.jpg?bank=-1')
        finally:h.shutdown();h.server_close();t.join()
if __name__=='__main__':unittest.main(verbosity=2)
