"""Bounded terminal feedback; physical units, no ROS or SDK side effects."""
import math

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class FreshFeedback:
    """Recompute from distinct source stamps, refresh cached commands <= 0.2s."""
    def __init__(self):
        self.stamp = None
        self.at = -math.inf
        self.value = (0., 'WAIT_SAMPLE')

    def update(self, stamp, now, compute):
        if not math.isfinite(stamp) or not math.isfinite(now):
            return 0., 'INVALID_SAMPLE'
        if self.stamp is not None and stamp < self.stamp:
            return 0., 'STAMP_REGRESSION'
        if stamp != self.stamp:
            self.stamp, self.at = stamp, now
            self.value = compute()
        return self.value if now-self.at <= .2 else (0., 'WAIT_SAMPLE')

class TerminalApproach:
    """One optional straight correction, never turns away from final heading later."""
    def __init__(self, now, pose):
        self.started = self.progress_at = now
        self.best = math.inf
        self.anchor = pose[:2]
        self.feedback = FreshFeedback()

    def command(self, pose, target, speed, stamp, now, source_age):
        if now-self.started >= 6.:
            return 0., 'POSITION_BUDGET'
        if not 0 <= source_age <= .35:
            return 0., 'WAIT_SAMPLE'
        def compute():
            dx, dy = target[0]-pose[0], target[1]-pose[1]
            distance = math.hypot(dx, dy)
            if distance < self.best-.01:
                self.best, self.progress_at = distance, now
            # Bound delay prediction and leave 2cm for observed stop response.
            brake = .10 + .02 + max(0., min(.15, speed))*(source_age+.15)
            if distance <= brake:
                return 0., 'POSITION_BRAKE'
            if distance > .35 or math.hypot(pose[0]-self.anchor[0],pose[1]-self.anchor[1]) > .20:
                return 0., 'POSITION_ENVELOPE'
            if abs(wrap(math.atan2(dy, dx)-pose[2])) > .18:
                return 0., 'POSITION_NOT_FORWARD'
            if now-self.progress_at >= 2.:
                return 0., 'POSITION_NO_PROGRESS'
            return .10, 'ADVANCE'
        return self.feedback.update(stamp, now, compute)

class TerminalYaw:
    """Predict braking with measured yaw rate; avoid ineffective tiny requests."""
    def __init__(self):
        self.feedback = FreshFeedback()

    def command(self, error, rate, stamp, now, source_age):
        if not 0 <= source_age <= .35:
            return 0., 'WAIT_SAMPLE'
        def compute():
            if abs(error) <= .07:
                return 0., 'YAW_BRAKE'
            remaining = error-max(-.5,min(.5,rate))*(source_age+.15)
            if error*rate > 0 and (remaining*error <= 0 or abs(remaining) <= .07):
                return 0., 'YAW_PREDICTED_BRAKE'
            return math.copysign(max(.12,min(.25,.8*abs(error))),error), 'TURN'
        return self.feedback.update(stamp, now, compute)
