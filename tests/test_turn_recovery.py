import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from turn_recovery import TurnWait
from terminal_feedback import TerminalApproach
from body_planner import BodyGrid, certify
from navigation_control import swept_rotation_clearance
from types import SimpleNamespace as S
import numpy as np

class TurnRecoveryTests(unittest.TestCase):
    def test_planner_and_turn_agree_on_body_inflation_and_real_obstacles(self):
        for cost, point, allowed in [(253,(.4,0),True),(254,(.4,0),False),
                                     (253,(0,0),False),(255,(.4,0),False)]:
            a=np.zeros((100,100),dtype=np.uint8)
            col,row=[int((v+2.5)/.05) for v in point];a[row,col]=cost
            raw=dict(width=100,height=100,resolution=.05,origin=[-2.5,-2.5,0],data=a.ravel())
            body=BodyGrid(raw)
            self.assertEqual(body.turn_clear(body.local([0,0,0]),.1),allowed)
            occupancy=np.zeros_like(a,dtype=np.int16)
            occupancy[row,col]={253:99,254:100,255:-1}[cost]
            grid=S(info=S(width=100,height=100,resolution=.05,
                          origin=S(position=S(x=-2.5,y=-2.5),orientation=S(x=0,y=0,z=0,w=1))),
                   data=occupancy.ravel().tolist())
            self.assertEqual(not bool(swept_rotation_clearance(grid,[0,0,0],.1,.5,.275)),allowed)

    def test_temporary_person_waits_then_resumes_only_after_stable_clear(self):
        w=TurnWait('ALIGN_PATH',[0,0,1],0)
        self.assertEqual(w.update(1,True,True),'WAIT')
        self.assertEqual(w.update(2,False,True),'WAIT')
        self.assertEqual(w.update(2.5,False,True),'RESUME')

    def test_persistent_obstacle_requests_replan_instead_of_turn(self):
        w=TurnWait('ALIGN_YAW',[1,2,3],0)
        self.assertEqual(w.update(3,True,True),'REPLAN')

    def test_stale_map_or_motion_cannot_resume_or_replan(self):
        w=TurnWait('ALIGN_YAW',[1,2,3],0)
        self.assertEqual(w.update(4,False,True,False),'WAIT')
        self.assertEqual(w.update(5,True,False),'WAIT')
        self.assertEqual(w.update(6,False,True),'WAIT')
        self.assertEqual(w.update(6.3,True,True),'REPLAN')

    def test_sensor_pause_does_not_consume_progress_timeout(self):
        a=TerminalApproach(0,[0,0,0])
        self.assertEqual(a.command([0,0,0],[.27,0],0,1,0,.1)[1],'ADVANCE')
        a.resume_after_pause(3.44)
        self.assertEqual(a.command([-.04,0,0],[.27,0],0,2,4.1,.1)[1],'ADVANCE')
        self.assertEqual(a.command([-.04,0,0],[.27,0],0,3,7.5,.1)[1],'POSITION_NO_PROGRESS')

if __name__=='__main__':unittest.main()
