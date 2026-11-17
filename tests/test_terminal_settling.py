import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_control import HeadingAlignment,settled_goal_action,RECOVERABLE_INPUT_PHASES

class TerminalSettlingTests(unittest.TestCase):
 def test_yaw_only_overshoot_does_not_replan(self):
  self.assertEqual(settled_goal_action([.18,0,.2],[0,0,0]),'ALIGN_YAW')
  self.assertEqual(settled_goal_action([.18,0,.09],[0,0,0]),'DONE')
  self.assertEqual(settled_goal_action([.201,0,.09],[0,0,0]),'REPLAN')
 def test_brake_early_when_already_turning_toward_goal(self):
  a=HeadingAlignment([0,0,0],[0,0,1],0,[0,0,0])
  self.assertEqual(a.command([0,0,.86],1,[0,0,.86],.3),(0.,''))
  self.assertGreater(a.command([0,0,.86],2,[0,0,.86],0)[0],0)
 def test_slowdown_near_target(self):
  a=HeadingAlignment([0,0,0],[0,0,1],0,[0,0,0])
  self.assertAlmostEqual(a.command([0,0,.8],1,[0,0,.8],0)[0],.16)
 def test_delayed_turn_plant_settles_without_position_replan(self):
  for target in (-2.5,1.,2.8):
   yaw=0.;rate=0.;state='TURN';now=0.
   a=HeadingAlignment([.17,0,0],[0,0,target],0,[0,0,0])
   for i in range(900):
    now=i*.05
    if state=='TURN':
     cmd,reason=a.command([.17,0,yaw],now,[0,0,yaw],rate)
     self.assertFalse(reason,reason)
     if cmd==0:state='STOP';halt=now
    else:
     cmd=0.
     if now-halt>=.7 and abs(rate)<.03:
      decision=settled_goal_action([.17,0,yaw],[0,0,target])
      self.assertNotEqual(decision,'REPLAN')
      if decision=='DONE':break
      state='TURN'
    rate+=(cmd-rate)*.05/.4;yaw+=rate*.05
   else:self.fail('did not settle in 45 seconds')
 def test_yaw_retry_keeps_original_timeout(self):
  a=HeadingAlignment([0,0,0],[0,0,1],0,[0,0,0])
  self.assertIn('ALIGN_TIMEOUT',a.command([0,0,.8],45,[0,0,.8])[1])
 def test_stale_scan_can_pause_during_recovery_and_planning(self):
  self.assertTrue({'REACQUIRE_STOP','REPLAN'}<=RECOVERABLE_INPUT_PHASES)

if __name__=='__main__':unittest.main()
