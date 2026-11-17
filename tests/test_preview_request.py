import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from preview_planner import validate_request, PreviewError


class PreviewRequestTests(unittest.TestCase):
    def setUp(self):
        self.maps={'old':{'lower':({'width':10,'height':20,'resolution':1.,'origin':[0.,0.,math.pi/2]},b'')}}
        self.state={'pose_current':True,'localization':{'ready':True,'session':'s','epoch':2,'map_id':'m'},'pose':{'x':-2.,'y':2.,'yaw':.3}}
        self.goal={'scene':'old','floor':'lower','x':-5.,'y':5.,'yaw':math.pi/2}
    def test_heading_and_rotated_map(self):
        goal,start,identity=validate_request(self.goal,'old',self.maps,self.state)
        self.assertAlmostEqual(goal['yaw'],math.pi/2)
        self.assertEqual(start['yaw'],.3)
        self.assertEqual(identity,('s',2,'m'))
    def test_invalid_targets(self):
        for patch in ({'yaw':float('nan')},{'x':True},{'x':3.},{'floor':'upper'},{'scene':'front_A'}):
            with self.subTest(patch=patch),self.assertRaises(PreviewError):
                validate_request(dict(self.goal,**patch),'old',self.maps,self.state)
    def test_stale_pose(self):
        self.state['pose_current']=False
        with self.assertRaises(PreviewError):validate_request(self.goal,'old',self.maps,self.state)
    def test_unready_localization(self):
        self.state['localization']['ready']=False
        with self.assertRaises(PreviewError):validate_request(self.goal,'old',self.maps,self.state)

if __name__=='__main__':unittest.main()
