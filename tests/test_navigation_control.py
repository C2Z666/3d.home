import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import HeadingAlignment,MotionGuard,rotation_clearance,initial_path_heading


class NavigationControlTests(unittest.TestCase):
 def test_initial_heading_follows_path_not_goal_yaw(self):
  route=[dict(x=0,y=0,yaw=0),dict(x=-.3,y=0,yaw=1.)]
  self.assertAlmostEqual(initial_path_heading(route,[0,0,0]),math.pi)
 def test_heading_can_escape_measured_speed_deadband(self):
  align=HeadingAlignment([0,0,0],[0,0,1.],0)
  self.assertEqual(align.command([0,0,0],1),(.3,''))
  self.assertEqual(align.command([0,0,.95],2),(0.,''))
 def test_wrap_and_negative_turn(self):
  align=HeadingAlignment([0,0,3.1],[0,0,-2.8],0)
  self.assertGreater(align.command([0,0,3.1],1)[0],0)
  align=HeadingAlignment([0,0,0],[0,0,-1.],0)
  self.assertEqual(align.command([0,0,0],1)[0],-.3)
 def test_real_rotation_is_progress_even_after_15_seconds(self):
  align=HeadingAlignment([0,0,0],[0,0,math.pi],0)
  for t in range(21):self.assertEqual(align.command([0,0,t*.1],t)[1],'')
 def test_stall_and_position_drift_do_not_report_success(self):
  align=HeadingAlignment([0,0,0],[0,0,1.],0)
  self.assertIn('ALIGN_NO_PROGRESS',align.command([0,0,0],12)[1])
  self.assertIn('ALIGN_POSITION_DRIFT',align.command([-.13,0,0],1)[1])
 def test_no_response_reverse_and_wrong_turn_guards(self):
  for command,measured,key,t in [((.1,0),(0,0,0),'NO_MOTION_RESPONSE',3.1),((0,.3),(0,0,0),'NO_TURN_RESPONSE',3.1),((0,.3),(-.1,0,.3),'UNEXPECTED_REVERSE',.8),((0,.3),(0,0,-.2),'WRONG_TURN_DIRECTION',1.1)]:
   guard=MotionGuard();self.assertEqual(guard.check(*command,measured,0),'')
   self.assertIn(key,guard.check(*command,measured,t))
 def test_brief_gait_oscillation_does_not_trip(self):
  guard=MotionGuard()
  for i in range(40):
   measured=(-.1 if i%2 else .1,0,.3)
   self.assertEqual(guard.check(0,.3,measured,i*.1),'')
 def test_costmap_obstacle_unknown_border_and_rotated_origin(self):
  grid=S(info=S(resolution=.05,width=80,height=80,origin=S(position=S(x=-2,y=-2),orientation=S(x=0,y=0,z=0,w=1))),data=[0]*6400)
  self.assertEqual(rotation_clearance(grid,(0,0),.58),'')
  for cost in (99,100,-1):
   grid.data[40*80+50]=cost
   self.assertNotEqual(rotation_clearance(grid,(0,0),.58),'')
  grid.data[40*80+50]=0
  self.assertIn('超出',rotation_clearance(grid,(-1.9,0),.58))
  grid.info.origin.position=S(x=2,y=-2)
  grid.info.origin.orientation=S(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4))
  self.assertEqual(rotation_clearance(grid,(0,0),.58),'')
  grid.data[40*80+50]=100
  self.assertNotEqual(rotation_clearance(grid,(0,0),.58),'')

if __name__=='__main__':unittest.main()
