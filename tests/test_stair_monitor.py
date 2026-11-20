import copy
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stair_monitor import StairMonitor, StairMonitorFault, monitor_config


def connection():
    return dict(control=dict(heading_correction=True,cruise_speed=.4,max_heading_error_rad=.35,
        max_lateral_m=.4,max_roll_rad=.45,max_pitch_rad=.85))


def sample(t=0.,yaw=0.,x=0.,y=0.):
    return dict(stamp=t,x=x,y=y,z=x*.5,yaw=yaw,roll=0.,pitch=.2)


class StairMonitorTests(unittest.TestCase):
    def test_closed_loop_and_limits(self):
        m=StairMonitor(connection(),sample(yaw=.14),0.,now=0.)
        yaw=.14;last=0.;outputs=[]
        for i in range(1,101):
            v,w,d=m.update(sample(i*.04,yaw,x=i*.008),.4,now=i*.04)
            self.assertLessEqual(abs(w),.05+1e-9)
            self.assertLessEqual(abs(w-last),.004+1e-9)
            self.assertLessEqual(w*yaw,1e-9)
            yaw+=w*.04;last=w;outputs.append((v,w))
        self.assertLess(yaw,.07)
        self.assertEqual(outputs[0][0],.4)

    def test_wraparound_and_no_integral(self):
        m=StairMonitor(connection(),sample(yaw=-math.pi+.04),math.pi-.04,now=0)
        v,w,d=m.update(sample(.04,-math.pi+.04,x=.005),.4,now=.04)
        self.assertAlmostEqual(d['heading_error_rad'],.08)
        self.assertLess(w,0)

    def test_deadband_noise_does_not_steer(self):
        m=StairMonitor(connection(),sample(),0.,now=0)
        for i in range(1,21):
            _,w,_=m.update(sample(.04*i,.02*(-1)**i,x=i*.008),.4,now=.04*i)
            self.assertEqual(w,0.)

    def test_duplicates_then_stall(self):
        m=StairMonitor(connection(),sample(),0.,now=0)
        m.update(sample(),.4,now=.1)
        with self.assertRaises(StairMonitorFault):m.update(sample(),.4,now=.26)

    def test_jump_gap_reverse_and_invalid(self):
        cases=[sample(.04,x=1),sample(.5),sample(-.1),sample(.04,y=.5),sample(.04,yaw=.4),sample(.04,yaw=float('nan'))]
        for s in cases:
            with self.subTest(sample=s):
                m=StairMonitor(connection(),sample(),0.,now=0)
                with self.assertRaises(StairMonitorFault):m.update(s,.4,now=.04)

    def test_lateral_warning_preserves_climbing_speed(self):
        m=StairMonitor(connection(),sample(),0.,now=0)
        v,w,d=m.update(sample(.1,x=.03,y=.13),.4,now=.1)
        self.assertEqual(v,.4);self.assertEqual(w,0.);self.assertTrue(d['lateral_warning'])

    def test_no_progress_and_diverging_response(self):
        m=StairMonitor(connection(),sample(),0.,now=0)
        with self.assertRaisesRegex(StairMonitorFault,'进展'):
            for i in range(1,85):m.update(sample(i*.1),.4,now=i*.1)
        m=StairMonitor(connection(),sample(yaw=.1),0.,now=0)
        with self.assertRaisesRegex(StairMonitorFault,'恶化'):
            for i in range(1,60):m.update(sample(i*.04,.1+i*.0032,x=i*.008),.4,now=i*.04)

    def test_height_gain_counts_as_climbing_progress(self):
        m=StairMonitor(connection(),sample(),0.,now=0)
        for i in range(1,130):
            v=sample(i*.1);v['z']=i*.005
            m.update(v,.4,now=i*.1)

    def test_climb_never_drops_below_point_four(self):
        m=StairMonitor(connection(),sample(yaw=.25),0.,now=0)
        v,w,d=m.update(sample(.1,.25,x=.03),.4,now=.1)
        self.assertEqual(v,.4)
        m=StairMonitor(connection(),sample(),0.,now=0)
        v,w,d=m.update(sample(.1,x=.03),.15,now=.1)
        self.assertEqual(v,.15,'Only the caller-confirmed platform phase may use low speed')

    def test_disabled_and_parameter_bounds(self):
        c=connection();c['control']['heading_correction']=False
        m=StairMonitor(c,sample(yaw=.1),0.,now=0)
        self.assertEqual(m.update(sample(.04,.1,x=.01),.4,now=.04)[1],0.)
        c['monitor']={'max_yaw_rate':.5}
        with self.assertRaises(ValueError):monitor_config(c)


if __name__=='__main__':unittest.main()
