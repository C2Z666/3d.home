"""Shared swept body rules for preview, replan and execution."""
import math
from body_planner import certify


def check_route(points,current,grid,half_length=.50,half_width=.275,goal_yaw=None):
    return certify(points,current,grid,half_length,half_width,goal_yaw=goal_yaw)


class ReplanPolicy:
    def __init__(self,wait=1.5,cooldown=3.,max_attempts=4):
        self.wait=wait;self.cooldown=cooldown;self.max_attempts=max_attempts
        self.since=None;self.last=-math.inf;self.attempts=0
    def blocked(self,now):
        if self.since is None:self.since=now
    def ready(self,now):return self.since is not None and now-self.since>=self.wait and now-self.last>=self.cooldown
    def request(self,now):
        if self.attempts>=self.max_attempts:return False
        self.attempts+=1;self.last=now;self.since=None;return True
