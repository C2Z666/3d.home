import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from landing_line import bind_arrival_line,crossed
from walk_reset import reset_floor_state


class LandingLineResetTests(unittest.TestCase):
    def test_plane_ignores_lateral_distance_but_requires_floor_and_connection(self):
        line=dict(x=5,y=0,yaw=0,floor_id='floor2',connection_id='A')
        for y in (-.3,0,.3):
            s=dict(x=5.1,y=y,candidate_floor='floor2',connection_id='A')
            self.assertTrue(crossed(line,s,'floor2','A')[0])
            s['x']=4.9;self.assertFalse(crossed(line,s,'floor2','A')[0])
            s.update(x=5.1,candidate_floor='floor1');self.assertFalse(crossed(line,s,'floor2','A')[0])
            s.update(candidate_floor='floor2',connection_id='B');self.assertFalse(crossed(line,s,'floor2','A')[0])

    def test_binding_transform_is_frozen(self):
        c=dict(id='A',to_floor='floor2',arrival_line=dict(x=15,y=20,yaw=0,floor_id='floor2'))
        mp=dict(x=10,y=20,yaw=0,stamp=1.);raw=dict(x=0,y=0,yaw=math.pi/2,stamp=1.)
        line=bind_arrival_line(c,mp,raw,raw)
        self.assertAlmostEqual(line['x'],0);self.assertAlmostEqual(line['y'],5)
        self.assertTrue(crossed(line,dict(x=.2,y=5.1,connection_id='A',candidate_floor='floor2'),'floor2','A')[0])
        mp['x']=100  # Later localization correction must not move the bound plane.
        self.assertAlmostEqual(line['y'],5)
        raw['stamp']=2
        with self.assertRaises(ValueError):bind_arrival_line(c,mp,raw,raw)

    def test_reset_never_invents_a_floor(self):
        for actual,active,phase in [('floor1','floor1','ON_FLOOR'),(None,'floor1','NEEDS_LOCALIZATION'),('floor2',None,'NEEDS_LOCALIZATION')]:
            state=dict(actual_floor=actual,active_nav_floor=active,phase='STAIR',connection_id='A',target_floor='floor2')
            changes=reset_floor_state(state)
            self.assertEqual(changes['phase'],phase)
            self.assertIsNone(changes['connection_id']);self.assertIsNone(changes['target_floor'])
            self.assertNotIn('actual_floor',changes);self.assertNotIn('active_nav_floor',changes)


if __name__=='__main__':unittest.main()
