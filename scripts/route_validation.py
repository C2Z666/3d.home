"""Shared swept body rules and bounded recovery of one stalled episode."""
import math
from body_planner import certify

def bounded_route(points, current, distance_limit=3., point_limit=80):
    """Keep the contiguous route prefix without skipping intermediate bends."""
    if distance_limit <= 0 or point_limit < 1:
        raise ValueError('invalid route check limits')
    result = []
    previous = (float(current[0]), float(current[1]))
    distance = 0.
    for point in points:
        x, y = float(point[0]), float(point[1])
        segment = math.hypot(x-previous[0], y-previous[1])
        if segment < 1e-9:
            continue
        remaining = distance_limit-distance
        if segment > remaining:
            fraction = remaining/segment
            result.append([previous[0]+fraction*(x-previous[0]), previous[1]+fraction*(y-previous[1])])
            return result, False
        result.append([x,y])
        distance += segment
        previous = (x,y)
        if len(result) >= point_limit or distance >= distance_limit:
            return result, False
    return result, True


def check_route(points,current,grid,half_length=.50,half_width=.275,goal_yaw=None):
    return certify(points,current,grid,half_length,half_width,goal_yaw=goal_yaw)

def near_obstacle_action(elapsed, limit, route_valid):
    if route_valid is True:
        return 'resume'
    if route_valid is False and elapsed >= limit:
        return 'replan'
    return 'wait'

class ClearWindow:
    """Require successive clear checks backed by advancing scan and map stamps."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.first=None
        self.previous=None

    def observe(self, now, map_stamp, scan_stamp, clear):
        if not clear or not map_stamp or not scan_stamp:
            self.reset()
            return False
        if self.previous:
            last, old_map, old_scan=self.previous
            if now-last>.5 or map_stamp<old_map or scan_stamp<old_scan:
                self.reset()
            elif map_stamp==old_map or scan_stamp==old_scan:
                return False
        if self.first is None:
            self.first=now
        self.previous=(now,map_stamp,scan_stamp)
        return now-self.first>=.5


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
