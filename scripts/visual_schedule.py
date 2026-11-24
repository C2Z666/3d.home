"""One worker, latest image, bounded confirmation bursts; monotonic scheduling."""
import math
import threading
from navigation_policy import load_policy


class VisualSchedule:
    def __init__(self, policy=None):
        self.policy = policy or load_policy()
        self.lock = threading.Lock()
        self.last_start = -math.inf
        self.burst_until = -math.inf
        self.burst_armed = True

    def request(self, now, explicit=False):
        with self.lock:
            # A failed episode cannot extend itself on every result. A later
            # episode may retry after a full quiet interval at the steady rate.
            if explicit or self.burst_armed or now >= self.burst_until+self.policy['visual_burst_sec']:
                self.burst_until = now+self.policy['visual_burst_sec']
                self.burst_armed = False

    def confirmed(self):
        with self.lock:
            self.burst_until = -math.inf
            self.burst_armed = True

    def ready(self, now):
        with self.lock:
            hz = self.policy['visual_burst_hz'] if now < self.burst_until else self.policy['visual_steady_hz']
            return now-self.last_start >= 1./hz-1e-9

    def started(self, now):
        with self.lock:
            self.last_start = now
