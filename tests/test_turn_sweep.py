import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import rotation_clearance,swept_rotation_clearance


class TurnSweep(unittest.TestCase):
    def grid(self):
        return S(info=S(resolution=.02,width=200,height=200,origin=S(position=S(x=-2.,y=-2.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=[0]*40000)

    def put(self,g,x,y,cost=100):g.data[int((y+2)/.02)*200+int((x+2)/.02)]=cost
    def check(self,g,delta,pose=(0.,0.,0.),**kw):return swept_rotation_clearance(g,pose,delta,.5,.275,**kw)

    def test_outside_small_turn_but_inside_full_circle(self):
        g=self.grid();self.put(g,0.,.52)
        self.assertTrue(rotation_clearance(g,(0,0),math.hypot(.5,.275)))
        self.assertEqual(self.check(g,math.radians(10)),'')

    def test_intermediate_collision_with_clear_endpoints(self):
        g=self.grid();self.put(g,.35,.35)
        self.assertEqual(self.check(g,0.),'')
        self.assertEqual(self.check(g,0.,pose=(0,0,math.pi/2)),'')
        self.assertIn('实体障碍',self.check(g,math.pi/2))
        self.assertEqual(self.check(g,-math.pi/3),'')

    def test_unknown_cell_corners_and_inflation(self):
        g=self.grid();self.put(g,.49,.27,-1)
        self.assertIn('未知',self.check(g,0.))
        self.put(g,.49,.27,99);self.assertEqual(self.check(g,0.),'')
        self.put(g,0.,0.,99);self.assertIn('中心',self.check(g,0.))

    def test_measured_coast_and_rotated_map(self):
        g=self.grid();self.put(g,.35,.35)
        self.assertTrue(self.check(g,-.1,coast_delta=1.))
        g.info.origin.position=S(x=2.,y=-2.)
        g.info.origin.orientation=S(x=0.,y=0.,z=math.sin(math.pi/4),w=math.cos(math.pi/4))
        self.assertTrue(self.check(g,math.pi/2,pose=(0.,0.,math.pi/2)))
        self.assertEqual(self.check(g,0.,pose=(0.,0.,math.pi/2)),'')

    def test_map_boundary_is_not_free(self):
        self.assertIn('超出',self.check(self.grid(),.2,pose=(-1.8,0.,0.)))

if __name__=='__main__':unittest.main()
