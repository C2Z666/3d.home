"""Bounded fresh-cloud queue; waiting for odometry must not starve older pairs."""
from collections import deque
from navigation_policy import load_policy, source_fresh


class CloudQueue:
    def __init__(self,limit=8,policy=None):
        self.items=deque(maxlen=limit);self.policy=policy or load_policy();self.last_taken=None
    def add(self,stamp,message):
        if self.last_taken is None or stamp>self.last_taken:self.items.append((stamp,message))
    def clear(self):self.items.clear()
    def take(self,now,oldest_odom,newest_odom):
        self.items=deque(((t,m) for t,m in self.items if source_fresh(now,t,self.policy) and t>=oldest_odom),maxlen=self.items.maxlen)
        ready=[(t,m) for t,m in self.items if t<=newest_odom]
        if not ready:return None
        t,message=max(ready,key=lambda item:item[0])
        self.last_taken=t
        self.items=deque(((s,m) for s,m in self.items if s>t),maxlen=self.items.maxlen)
        return message
