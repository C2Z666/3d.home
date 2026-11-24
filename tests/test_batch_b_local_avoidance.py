"""Behavioral coverage for local motion, event-driven search and the real loop."""
import ast,copy,math,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from local_motion import LocalGrid,stopping_sweep
from local_avoidance import LocalProgress,SearchGate
from navigation_policy import FreshWindow
from test_batch_a_policy import nested


def grid():
    m=S(info=S(resolution=.05,width=120,height=120,
        origin=S(position=S(x=-3.,y=-3.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=np.zeros(14400,np.int8))
    return LocalGrid(m)


class Motion(unittest.TestCase):
    def sweep(self,g,cmd=(.3,0.),measured=(0.,0.,0.),age=.3):
        return stopping_sweep(g,(0.,0.,0.),cmd,measured,.50,.275,age)[0]

    def test_reference_block_ahead_does_not_forbid_legal_short_motion(self):
        g=grid();g.data[60,84]=100  # 1.2m ahead, outside current stopping arc
        self.assertEqual(self.sweep(g),'CLEAR')

    def test_body_and_braking_are_checked_not_only_centre(self):
        g=grid();g.data[64,73]=100  # 0.65m forward, off centreline
        self.assertEqual(self.sweep(g),'BLOCKED')
        self.assertEqual(self.sweep(g,(0.,0.)),'CLEAR')
        self.assertEqual(self.sweep(g,(0.,0.),(.3,0.,0.)),'BLOCKED')

    def test_soft_inflation_is_not_a_second_body_padding(self):
        g=grid();g.data[64,68]=99
        self.assertEqual(self.sweep(g),'CLEAR')
        g.data[60,60]=99;self.assertEqual(self.sweep(g),'BLOCKED')

    def test_unknown_is_data_unavailable_not_a_replan_trigger(self):
        g=grid();g.data[64,68]=-1
        self.assertEqual(self.sweep(g),'DATA_UNAVAILABLE')
        self.assertEqual(stopping_sweep(g,(-2.8,0,0),(0,0),(0,0,0),.5,.275,.1)[0],'DATA_UNAVAILABLE')

    def test_source_age_extends_stopping_arc(self):
        g=grid();g.data[60,75]=100
        self.assertEqual(self.sweep(g,age=0.),'CLEAR')
        self.assertEqual(self.sweep(g,age=.59),'BLOCKED')

    def test_curved_motion_and_rotated_map_origin(self):
        g=grid();g.data[68,73]=100
        self.assertEqual(self.sweep(g),'CLEAR')
        self.assertEqual(self.sweep(g,(.3,.4),age=.5),'BLOCKED')
        g=grid();g.origin=(3.,-3.,math.pi/2)
        self.assertEqual(self.sweep(g),'CLEAR')

    def test_corner_touch_is_a_collision(self):
        g=grid();g.data[65,70]=100
        self.assertEqual(g.rectangle((0.,0.,0.),.5,.275,1e-6)[0],'BLOCKED')


class Progress(unittest.TestCase):
    def test_two_seconds_zero_local_command(self):
        p=LocalProgress();self.assertEqual(p.update(0,(0,0,0),(0,0)),'WAIT_LOCAL_MOTION')
        self.assertEqual(p.update(1.99,(0,0,0),(0,0)),'WAIT_LOCAL_MOTION')
        self.assertEqual(p.update(2,(0,0,0),(0,0)),'NO_LOCAL_MOTION')

    def test_progress_can_go_away_from_goal_and_around_bend(self):
        p=LocalProgress()
        for i in range(40):
            self.assertEqual(p.update(i*.2,(-i*.025,i*.03,0),(.2,.1)),'ADVANCE')

    def test_small_shuffling_and_missing_motion_stop_after_three_seconds(self):
        for positions in ([(0,0,0)]*32,[(.03*math.sin(i),0,0) for i in range(32)]):
            p=LocalProgress()
            for i,pos in enumerate(positions):result=p.update(i*.1,pos,(.2,0))
            self.assertEqual(result,'NO_PROGRESS')

    def test_deliberate_turn_is_allowed_but_repeated_spin_is_bounded(self):
        p=LocalProgress()
        for i in range(41):self.assertEqual(p.update(i*.1,(0,0,i*.03),(0,.3)),'ADVANCE')
        decisions=[p.update(i*.1,(0,0,i*.03),(0,.3)) for i in range(41,150)]
        self.assertIn('NO_PROGRESS',decisions)

    def test_data_loss_resets_confirmation_without_geometry_replan(self):
        p=LocalProgress();p.update(0,(0,0,0),(0,0))
        self.assertEqual(p.update(5,(0,0,0),(0,0),valid=False),'DATA_UNAVAILABLE')
        self.assertEqual(p.update(10,(0,0,0),(0,0)),'WAIT_LOCAL_MOTION')

    def test_confirmed_input_evidence_survives_async_map_completion(self):
        w=FreshWindow(.2);w.observe(0,(1,1));w.observe(.25,(2,2))
        self.assertTrue(w.confirmed(.35));self.assertFalse(w.confirmed(.8))


class Searches(unittest.TestCase):
    def setUp(self):
        self.g=dict(width=160,height=160,resolution=.05,origin=[-4,-4,0],data=np.zeros(25600,np.uint8))
        self.gate=SearchGate()
    def evidence(self,start=(0,0,0)):
        return self.gate.evidence(self.g,start,(2,0,0))
    def test_no_duplicate_worker_or_retry_on_new_timestamp(self):
        e=self.evidence();self.assertTrue(self.gate.begin(e));self.assertFalse(self.gate.begin(e))
        self.gate.finish();self.g['stamp_ns']=123456
        self.assertFalse(self.gate.begin(self.evidence()))
    def test_soft_noise_and_distant_changes_do_not_restart_search(self):
        self.gate.begin(self.evidence());self.gate.finish()
        self.g['data'][80*160+82:80*160+88]=253;self.g['data'][0:10]=254
        self.assertFalse(self.gate.begin(self.evidence()))
    def test_changed_local_geometry_or_start_can_retry(self):
        self.gate.begin(self.evidence());self.gate.finish()
        self.g['data'][80*160+82:80*160+86]=254
        self.assertTrue(self.gate.begin(self.evidence()));self.gate.finish()
        self.assertFalse(self.gate.begin(self.evidence()))
        self.assertTrue(self.gate.begin(self.evidence((.11,0,0))))
    def test_running_search_handle_is_kept_when_a_check_is_requeued(self):
        future=Mock();env=dict(body_future=future,body_repaired=True,body_force=False,check_future=None,
            approved=True,route_future=None,route_last_valid=False,execution_deadline=1000.,time=S(monotonic=lambda:1.),
            transition=Mock(),request_route_check=Mock())
        nested('queue_route_check','navigate_to_goal.py',env)()
        self.assertIs(env['body_future'],future);future.cancel.assert_not_called()
        env['request_route_check'].assert_not_called()


class RunningSupervisor(unittest.TestCase):
    def loop(self,**extra):
        tree=ast.parse((ROOT/'scripts/navigate_to_goal.py').read_text())
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="state in ('NAVIGATE', 'ACCEPT')")
        class OneTick(ast.NodeTransformer):
            def visit_Continue(self,node):return ast.copy_location(ast.Break(),node)
        code=ast.fix_missing_locations(OneTick().visit(ast.Module(body=[ast.While(test=ast.Constant(True),body=copy.deepcopy(branch.body)+[ast.Break()],orelse=[])],type_ignores=[])))
        twist=S(linear=S(x=.2,y=0,z=0),angular=S(x=0,y=0,z=0))
        env=dict(state='NAVIGATE',sdk=dict(motion='Walk',motion_pending=False),current_pose=lambda:(0.,0.,0.),
                 current_odom=lambda:(0.,0.,0.),target=dict(x=4.,y=0.,yaw=0.),turn_replan_route=False,
                 now=1.,result=None,goal=None,raw=twist,raw_controller=twist,raw_at=1.,controller_at=1.,
                 MAX_LINEAR_SPEED=.3,MAX_ANGULAR_SPEED=.4,math=math,output=Mock(),zero=Mock(),fault=Mock(),
                 current_motion_check=lambda c:('CLEAR',{}),pause_inputs=Mock(),obstacle_hold=Mock(),
                 local_progress=LocalProgress(),progress_distance=0.,replan_cfg={'progress_distance_m':.1},
                 replan_policy=Mock(),request_route_check=Mock(),poll_route_check=Mock(),first_command_seen=True,
                 command_missing_at=None,route_details={'clear_prefix_m':.5})
        env.update(extra);exec(compile(code,'production_nav_loop','exec'),env);return env
    def test_blocked_reference_does_not_cancel_dwb_or_fetch_global_map(self):
        e=self.loop();e['output'].assert_called_once_with(.2,0.,1.)
        e['obstacle_hold'].assert_not_called();e['request_route_check'].assert_not_called();e['poll_route_check'].assert_not_called()
    def test_actual_unsafe_command_stops_before_output(self):
        e=self.loop(current_motion_check=lambda c:('BLOCKED',{'reason':'obstacle'}))
        e['output'].assert_not_called();e['obstacle_hold'].assert_called_once()
    def test_unavailable_local_data_never_triggers_obstacle_search(self):
        e=self.loop(current_motion_check=lambda c:('DATA_UNAVAILABLE',{}))
        e['output'].assert_not_called();e['obstacle_hold'].assert_not_called();e['pause_inputs'].assert_called_once()
    def test_dwb_zero_does_not_forward_nonzero_smoother_tail(self):
        zero=S(linear=S(x=0.,y=0.,z=0.),angular=S(x=0.,y=0.,z=0.))
        e=self.loop(raw_controller=zero);e['output'].assert_called_once_with(0.,0.,1.)
    def test_invalid_raw_dwb_cannot_release_a_valid_old_smoothed_command(self):
        invalid=S(linear=S(x=float('nan'),y=0.,z=0.),angular=S(x=0.,y=0.,z=0.))
        e=self.loop(raw_controller=invalid);e['output'].assert_not_called();e['fault'].assert_called_once()


if __name__=='__main__':unittest.main()
