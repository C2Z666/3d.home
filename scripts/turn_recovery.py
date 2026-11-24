"""Bounded stationary waiting for a fresh, clear rectangular turn sweep."""
class TurnWait:
    def __init__(self, phase, target, now, timeout=3.,stable_sec=.5):
        self.phase, self.target = phase, list(target)
        self.started, self.timeout = now, timeout
        self.stable_sec = stable_sec
        self.clear_since = None
        self.next_check = now

    def update(self, now, blocked, stationary, fresh=True):
        if not stationary or not fresh:
            self.clear_since = None
            return 'WAIT'
        if not blocked:
            if self.clear_since is None:
                self.clear_since = now
            if now-self.clear_since >= self.stable_sec:
                return 'RESUME'
        else:
            self.clear_since = None
            if now-self.started >= self.timeout:
                return 'REPLAN'
        return 'WAIT'
