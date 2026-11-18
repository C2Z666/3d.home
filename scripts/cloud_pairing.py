"""Bounded fresh-cloud queue; waiting for odometry must not starve older pairs."""
from collections import deque


class CloudQueue:
    def __init__(self,limit=8):self.items=deque(maxlen=limit)
    def add(self,stamp,message):self.items.append((stamp,message))
    def clear(self):self.items.clear()
    def take(self,now,oldest_odom,newest_odom):
        self.items=deque(((t,m) for t,m in self.items if -.1<=now-t<=.5 and t>=oldest_odom),maxlen=self.items.maxlen)
        ready=[(t,m) for t,m in self.items if t<=newest_odom]
        if not ready:return None
        t,message=max(ready,key=lambda item:item[0])
        self.items=deque(((s,m) for s,m in self.items if s>t),maxlen=self.items.maxlen)
        return message
