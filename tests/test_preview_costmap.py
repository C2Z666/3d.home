import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from preview_planner import planner_cell, diagnose


class CostmapTests(unittest.TestCase):
    def grid(self, data, width=3):
        return dict(width=width,height=len(data)//width,resolution=1.,origin=[0.,0.,0.],data=data)

    def test_navfn_rounding_and_boundary(self):
        g=self.grid([0,253,0,0,0,0])
        self.assertEqual(planner_cell(g,dict(x=.5,y=0))['cost'],253)
        self.assertEqual(planner_cell(g,dict(x=.49,y=0))['cost'],0)
        self.assertIsNone(planner_cell(g,dict(x=-.01,y=0))['cost'])
        self.assertIsNone(planner_cell(g,dict(x=2.5,y=0))['cost'])

    def test_each_blocked_goal_reason(self):
        for cost,code in ((253,'goal_inflated'),(254,'goal_obstacle'),(255,'goal_unknown')):
            with self.subTest(cost=cost):
                g=self.grid([0,cost,0])
                result=diagnose(g,dict(x=0,y=0),dict(x=1,y=0))
                self.assertEqual(result[0],code)
                self.assertEqual(result[2]['goal_cell']['cost'],cost)

    def test_disconnected_and_high_cost_passable(self):
        for barrier,code in ((253,'disconnected'),(255,'disconnected'),(252,'planner_failed')):
            self.assertEqual(diagnose(self.grid([0,barrier,0]),dict(x=0,y=0),dict(x=2,y=0))[0],code)

    def test_navfn_clears_blocked_start(self):
        self.assertEqual(diagnose(self.grid([254,0,0]),dict(x=0,y=0),dict(x=2,y=0))[0],'planner_failed')

    def test_recorded_goal_rounds_into_inflation(self):
        g=dict(width=286,height=187,resolution=.05000000074505806,origin=[-7.98281,-5.42383,0],data=[0]*(286*187))
        g['data'][95*286+138]=231;g['data'][95*286+139]=253
        self.assertEqual(planner_cell(g,dict(x=-1.045,y=-.653)),dict(x=139,y=95,cost=253))

if __name__=='__main__':unittest.main()
