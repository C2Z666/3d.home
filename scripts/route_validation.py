"""Shared swept body rules and bounded recovery of one stalled episode."""
import math
from body_planner import certify

def check_route(points,current,grid,half_length=.50,half_width=.275,goal_yaw=None):
    return certify(points,current,grid,half_length,half_width,goal_yaw=goal_yaw)

def near_obstacle_action(elapsed, limit, route_valid):
    if route_valid is True:
        return 'resume'
    if route_valid is False and elapsed >= limit:
        return 'replan'
    return 'wait'

class ReplanPolicy:
    def __init__(self,wait=3.,cooldown=3.,max_attempts=8,stall_limit=60.):
        if type(max_attempts) is not int or max_attempts<1:
            raise ValueError('max_replan_attempts 必须为正整数')
        self.wait=wait;self.cooldown=cooldown;self.max_attempts=max_attempts
        self.since=None;self.last=-math.inf;self.attempts=0
        self.stalled_at=None;self.stall_limit=stall_limit;self.pause_at=None
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
    def set_waiting(self,now,waiting):
        if waiting and self.pause_at is None:
            self.pause_at=now
        elif not waiting and self.pause_at is not None:
            if self.stalled_at is not None:
                self.stalled_at+=max(0.,now-max(self.pause_at,self.stalled_at))
            self.pause_at=None
    def stall_error(self,now):
        end=now if self.pause_at is None else min(now,self.pause_at)
        if self.stalled_at is not None and end-self.stalled_at>=self.stall_limit:
            return '[OBSTACLE_STALL_TIMEOUT] 扣除停车等待后，累计60秒仍未取得有效前进，已停车'
        return ''
