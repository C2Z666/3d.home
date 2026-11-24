"""Batch A regressions: source age, recovery evidence, scheduling and legacy ROS."""
import ast,copy,math,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_policy import source_fresh,load_policy,configure_costmaps,FreshWindow,follow_path_failure
from visual_schedule import VisualSchedule
from cloud_pairing import CloudQueue
from route_validation import ClearWindow,near_obstacle_action,timed_check_route,check_route
from mission_runtime import RecoveryBudget
from fusion_policy import FusionPolicy


def nested(name,filename,env):
    tree=ast.parse((ROOT/'scripts'/filename).read_text())
    fun=copy.deepcopy(next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name))
    fun.body=[ast.Global(n.names) if isinstance(n,ast.Nonlocal) else n for n in fun.body]
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fun],type_ignores=[])),filename,'exec'),env)
    return env[name]


class SourceAge(unittest.TestCase):
    def test_same_source_clock_before_and_after_processing(self):
        self.assertTrue(source_fresh(10.55,10.))
        self.assertFalse(source_fresh(10.601,10.))
        self.assertFalse(source_fresh(9.94,10.))
        self.assertFalse(source_fresh(10.,None))
        self.assertFalse(source_fresh(10.,float('nan')))

    def test_queue_pairs_latest_without_replaying_duplicates(self):
        q=CloudQueue();q.add(10.,'a');q.add(10.1,'b');q.add(10.7,'unpaired')
        self.assertEqual(q.take(10.55,9.9,10.2),'b')
        q.add(10.,'old');q.add(10.1,'duplicate')
        self.assertIsNone(q.take(10.55,9.9,10.2))
        self.assertIsNone(q.take(11.4,9.9,11.4))

    def test_production_scan_callback_accepts_550ms_and_rejects_old_order(self):
        clock=S(nanoseconds=10_550_000_000)
        env=dict(policy=load_policy(),source_fresh=source_fresh,n=S(get_clock=lambda:S(now=lambda:clock)),
                 time=time,scan_at=-math.inf,scan_stamp=None,scan_counts={},scan_callbacks={},scan_diag={},match_scan=lambda _:None)
        receive=nested('on_scan','navigate_to_goal.py',env)
        scan=S(header=S(frame_id='nav_base',stamp=S(sec=10,nanosec=0)),ranges=[1.]*720)
        receive(scan);self.assertEqual(env['scan_counts'],{'accepted':1})
        accepted=env['scan_at'];receive(scan)
        self.assertEqual(env['scan_at'],accepted);self.assertEqual(env['scan_counts']['out_of_order'],1)
        clock.nanoseconds=10_701_000_000;scan.header.stamp.nanosec=100_000_000
        receive(scan);self.assertEqual(env['scan_counts']['source_stale'],1)

    def test_supervisor_rechecks_source_age_not_receipt_age(self):
        clock=S(nanoseconds=10_550_000_000)
        env=dict(policy=load_policy(),source_fresh=source_fresh,n=S(get_clock=lambda:S(now=lambda:clock)),
                 map_id='map',loc=dict(map_id='map',ready=True),loc_at=100.,pose_at=100.,
                 scan_at=100.,scan_stamp=10.,odom_at=100.,health_at=100.,pose=object(),
                 scanhealth={'ready':True},scan_input_state=lambda h:(True,True,''))
        safe=nested('localization_safe','navigate_to_goal.py',env)
        self.assertTrue(safe(100.01))
        clock.nanoseconds=10_601_000_000
        self.assertFalse(safe(100.02),'recent receipt must not extend an expired source')

    def test_launch_policy_is_identical_for_both_maps(self):
        cfg=configure_costmaps(yaml.safe_load((ROOT/'config/nav2.yaml').read_text()))
        for name in ('global_costmap','local_costmap'):
            layer=cfg[name][name]['ros__parameters']['obstacle_layer']
            self.assertEqual(layer['source_max_age_sec'],.6)
            self.assertEqual(layer['obstacles']['expected_update_rate'],.6)
            self.assertEqual(layer['obstacles']['observation_persistence'],2.,'batch C is not applied')


class Recovery(unittest.TestCase):
    def test_pending_clear_never_replans_even_after_wait_deadline(self):
        window=ClearWindow()
        state=window.classify(10.,100,100,True)
        self.assertEqual(state,'CLEAR_PENDING');self.assertEqual(near_obstacle_action(20,5,state),'wait')
        self.assertEqual(window.classify(10.1,100,101,True),'CLEAR_PENDING')
        state=window.classify(10.21,101,102,True)
        self.assertEqual(state,'CLEAR_CONFIRMED');self.assertEqual(near_obstacle_action(20,5,state),'resume')
        state=window.classify(10.3,102,103,False)
        self.assertEqual(near_obstacle_action(20,5,state),'replan')
        state=window.classify(10.4,None,104,None)
        self.assertEqual(state,'DATA_UNAVAILABLE');self.assertEqual(near_obstacle_action(20,5,state),'wait')

    def test_regressions_and_gaps_restart_confirmation(self):
        w=FreshWindow(.2)
        self.assertFalse(w.observe(0,(10,10)))
        self.assertFalse(w.observe(.2,(9,11)))
        self.assertFalse(w.observe(.8,(12,12)))
        self.assertTrue(w.observe(1.,(13,13)))
        self.assertFalse(w.observe(1.1,(14,14),False))

    def test_wait_phase_partition_does_not_reset_budget(self):
        b=RecoveryBudget();b.begin_wait(0);b.set_phase(1,'settling');b.set_phase(3,'action_release');b.end_wait(4)
        snap=b.snapshot(10)
        self.assertEqual(snap['wait_total_sec'],4)
        self.assertEqual(snap['wait_phase_sec'],dict(input_unavailable=1.,settling=2.,action_release=1.))
        b.begin_wait(10);self.assertIn('DATA_WAIT_TIMEOUT',b.wait_error(40))

    def test_legacy_empty_requires_geometry_but_modern_fault_is_not_hidden(self):
        self.assertEqual(follow_path_failure(S()),'REVIEW')
        self.assertEqual(follow_path_failure(S(),'BLOCKED'),'BLOCKED')
        self.assertEqual(follow_path_failure(S(),'CLEAR_CONFIRMED'),'FAULT')
        self.assertEqual(follow_path_failure(S(error_code=104)),'BLOCKED')
        self.assertEqual(follow_path_failure(S(error_code=101),'BLOCKED'),'FAULT')

    def test_timing_wrapper_preserves_collision_and_reports_stages(self):
        data=np.zeros((80,80),dtype=np.uint8);data[44,55]=254
        grid=dict(width=80,height=80,resolution=.05,origin=[-2.,-2.,0.],data=data.ravel())
        args=([[0,0],[1,0]],[0,0,0],grid,.5,.275,None)
        ok,detail=timed_check_route(*args,time.monotonic())
        self.assertEqual(ok,check_route(*args)[0]);self.assertFalse(ok)
        self.assertTrue(all(t>=0 for t in detail['timing'].values()))


class Visual(unittest.TestCase):
    def test_steady_rate_and_bounded_burst(self):
        s=VisualSchedule();self.assertTrue(s.ready(0));s.started(0)
        self.assertFalse(s.ready(1.9));self.assertTrue(s.ready(2))
        s.request(2);s.started(2);self.assertTrue(s.ready(2.5));s.started(2.5)
        s.request(6.9);self.assertEqual(s.burst_until,7.,'candidate cannot perpetually extend burst')
        s.started(6.8);self.assertFalse(s.ready(7.5));self.assertTrue(s.ready(8.8))
        s.confirmed();s.request(9);self.assertEqual(s.burst_until,14)

    def test_three_frames_can_confirm_over_two_seconds(self):
        f=FusionPolicy();pose=np.eye(4)
        self.assertFalse(f.update(pose,pose,10.))
        self.assertFalse(f.update(pose,pose,11.1))
        self.assertTrue(f.update(pose,pose,12.2))
        # A large correction still requires a consistent burst, never one image.
        moved=pose.copy();moved[0,3]=1
        self.assertFalse(f.update(moved,pose,14.))
        self.assertFalse(f.update(moved,pose,15.))
        self.assertTrue(f.update(moved,pose,16.))


if __name__=='__main__':unittest.main()
