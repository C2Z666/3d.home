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
    """One bounded curved approach, then irrevocably hand off to final yaw."""
    def __init__(self, now, pose, budget=12.):
        self.started = self.progress_at = self.adjusted_at = now
        self.budget = min(12., max(0., budget))
        self.best = math.inf
        self.anchor = pose[:2]
        self.feedback = FreshFeedback()
        self.request = .12
        self.wz = 0.
        self.samples = []
        self.diagnostics = {}

    def command(self, pose, target, speed, stamp, now, source_age):
        if not all(math.isfinite(v) for v in (*pose, *target, speed, stamp, now, source_age)):
            self.wz=0.; return 0., 'INVALID_SAMPLE'
        if now-self.started >= self.budget:
            self.wz=0.; return 0., 'POSITION_BUDGET'
        if not 0 <= source_age <= .35:
            self.wz=0.; return 0., 'WAIT_SAMPLE'
        def compute():
            self.wz=0.
            dx, dy = target[0]-pose[0], target[1]-pose[1]
            distance = math.hypot(dx, dy)
            bearing = wrap(math.atan2(dy, dx)-pose[2])
            self.samples.append((now,speed))
            self.samples=[v for v in self.samples if now-v[0]<=.8]
            response=sum(v[1] for v in self.samples)/len(self.samples)
            # Measured response and a bounded acceleration braking estimate.
            v=max(0., min(.3,max(speed,response)))
            brake=.09+.02+v*(source_age+.2)+v*v/(2*.3)
            desired=min(.06,max(.025,.5*(distance-brake)))
            if distance < self.best-.01:
                self.best, self.progress_at = distance, now
            self.diagnostics=dict(distance=distance,bearing_error=bearing,
                measured_mean=response,desired_response=desired,brake_distance=brake,
                requested_speed=self.request,budget_remaining=max(0.,self.budget-(now-self.started)))
            if distance <= brake:return 0., 'POSITION_BRAKE'
            if distance > .35 or math.hypot(pose[0]-self.anchor[0],pose[1]-self.anchor[1]) > .30:
                return 0., 'POSITION_ENVELOPE'
            if abs(bearing) > .6:return 0., 'POSITION_NOT_FORWARD'
            if now-self.progress_at >= 4.:return 0., 'POSITION_NO_PROGRESS'
            # Never boost indefinitely against a blocked/no-response plant.
            # Adapt only on new samples and after sustained observed response.
            if now-self.adjusted_at >= .5 and now-self.started>=.6:
                if response < desired-.008:self.request=min(.15,self.request+.01)
                elif response > desired+.008:self.request=max(.10,self.request-.01)
                self.adjusted_at=now
            self.wz=max(-.18,min(.18,1.2*bearing)) if abs(bearing)>.04 else 0.
            self.diagnostics['requested_speed']=self.request
            return self.request, 'ADVANCE'
        value=self.feedback.update(stamp, now, compute)
        if value[1]!='ADVANCE':self.wz=0.
        return value

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
