"""Shared swept body rules and bounded recovery of one stalled episode."""
import math
from body_planner import certify

def check_route(points,current,grid,half_length=.50,half_width=.275,goal_yaw=None):
    return certify(points,current,grid,half_length,half_width,goal_yaw=goal_yaw)

class ReplanPolicy:
    def __init__(self,wait=3.,cooldown=3.,max_attempts=4,stall_limit=60.):
        self.wait=wait;self.cooldown=cooldown;self.max_attempts=max_attempts
        self.since=None;self.last=-math.inf;self.attempts=0
        self.stalled_at=None;self.stall_limit=stall_limit
        self.progress_at=None;self.progress_distance=None
    def blocked(self,now):
        if self.since is None:self.since=now
        if self.stalled_at is None:self.stalled_at=now
    def cleared(self):
        self.since=None
    def ready(self,now):
        return self.since is not None and now-self.since>=self.wait and now-self.last>=self.cooldown
    def request(self,now):
        if self.attempts>=self.max_attempts:return False
        self.attempts+=1;self.last=now;self.since=None;return True
    def timeout(self,now,charged=True):
        # A compute timeout does not prove the physical route is blocked.
        if charged:self.attempts=max(0,self.attempts-1)
        self.last=now+2.
    def progress(self,now,distance,valid=True):
        if not valid:
            self.progress_at=None;self.progress_distance=None;return False
        if self.progress_at is None:
            self.progress_at=now;self.progress_distance=distance;return False
        if now-self.progress_at>=3. and self.progress_distance-distance>=.5:
            reset=self.attempts>0 or self.stalled_at is not None
            self.attempts=0;self.stalled_at=None;self.since=None
            self.progress_at=now;self.progress_distance=distance
            return reset
        return False
    def stall_error(self,now):
        if self.stalled_at is not None and now-self.stalled_at>=self.stall_limit:
            return '[OBSTACLE_STALL_TIMEOUT] 本次受阻60秒仍未取得有效前进，已停车'
        return ''
