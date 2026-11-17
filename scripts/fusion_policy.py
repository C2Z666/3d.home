"""Continuous FAST-LIO tracking with bounded, verified visual corrections."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from geometry import rigid


class FusionPolicy:
    def __init__(self):
        self.T = None
        self.last_visual = None
        self.epoch = 0
        self.required = False  # Only an explicitly requested stationary reset.
        self.recovering = False
        self.visual_stale = True
        self.candidates = []
        self.last_query = -math.inf
        self.reason = 'waiting for three consistent visual observations'
        self.delta_m = self.delta_deg = None
        self.last_rejection_reason = ''
        # Observations select a target alignment; only odometry callbacks move
        # the applied alignment. Rates therefore do not depend on camera FPS.
        self.target = None
        self.target_stamp = None
        self.advance_stamp = None
        self.translation_rate = .03  # m/s at the CURRENT robot, not odom origin
        self.rotation_rate = math.radians(1.5)  # rad/s, full SO(3)
        self.time_constant = 1.0
        self.target_ttl = 1.5  # capture time, never result arrival time
        self.correction = dict(active=False,step_m=0.,step_deg=0.,remaining_m=0.,remaining_deg=0.)

    def invalidate(self, reason):
        # A failed manual recovery must not permanently disable an existing
        # map alignment. Actual odometry faults are gated by localization_live.
        self.required = self.recovering = False
        self.candidates.clear()
        self.target = None
        self.reason = self.last_rejection_reason = reason

    def request(self):
        self.epoch += 1
        self.required = self.recovering = True
        self.candidates.clear()
        self.target = None
        self.delta_m = self.delta_deg = None
        self.reason = 'stationary relocalization: waiting for three consistent images'

    def expire(self, t, ttl):
        # Visual absence is a change of information source, not loss of odometry.
        self.visual_stale = self.last_visual is None or t-self.last_visual >= ttl

    def consistent(self, V, O, C, t):
        self.candidates = [(a, B) for a, B in self.candidates if 0 < t-a <= 2.]
        if self.candidates:
            d = np.linalg.inv(self.candidates[0][1] @ O) @ V
            if np.linalg.norm(d[:3,3]) > .25 or Rotation.from_matrix(d[:3,:3]).magnitude() > np.deg2rad(10):
                self.candidates.clear()
        self.candidates.append((t, C))
        return len(self.candidates) >= 3 and t-self.candidates[0][0] >= .3

    def update(self, V, O, t):
        if not math.isfinite(t) or t <= self.last_query:
            self.reason = 'duplicate or reversed visual timestamp'
            return False
        self.last_query = t
        if not all(A.shape == (4,4) and np.isfinite(A).all() for A in (V,O)):
            self.reason = 'invalid visual or odometry transform'
            return False
        C = rigid(V @ np.linalg.inv(O))
        if self.T is not None:
            delta = np.linalg.inv(self.T @ O) @ V
            self.delta_m = float(np.linalg.norm(delta[:3,3]))
            self.delta_deg = float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude()))
        if self.T is None or self.recovering:
            if not self.consistent(V, O, C, t):
                self.reason = 'consistent visual candidates %d/3 within 2s' % len(self.candidates)
                return False
            self.T = C
            self.target = None
            self.advance_stamp = None
            self.required = self.recovering = False
            self.candidates.clear()
        else:
            if self.delta_m > .15 or self.delta_deg > 5.:
                if not self.consistent(V, O, C, t):
                    self.reason = 'visual correction pending %d/3 consistent frames; FAST-LIO continues' % len(self.candidates)
                    self.last_rejection_reason = self.reason
                    return False
            else:
                self.candidates.clear()
            self.target = C
            self.target_stamp = t
        self.last_visual = t
        self.visual_stale = False
        self.reason = 'accepted visual alignment; correction applied smoothly at odometry rate'
        return True

    def advance(self, O, t):
        """Slew map alignment, keeping raw FAST-LIO pose/velocity untouched.

        Evaluate both transforms at this same odometry pose: rotation about a
        distant map/odom origin must not create an unbounded position change.
        Pauses never accumulate correction credit. An old accepted match is
        not extrapolated indefinitely after the camera loses matching.
        """
        self.correction = dict(active=False,step_m=0.,step_deg=0.,remaining_m=0.,remaining_deg=0.)
        if not math.isfinite(t) or (self.advance_stamp is not None and t <= self.advance_stamp):
            return
        dt = 0. if self.advance_stamp is None else min(.1,t-self.advance_stamp)
        self.advance_stamp = t
        if self.T is None or self.target is None or self.recovering:
            return
        age = t-self.target_stamp
        if age > self.target_ttl:
            self.target = None
            return
        if age < 0 or dt <= 0:
            return
        predicted, desired = self.T @ O, self.target @ O
        distance = float(np.linalg.norm(desired[:3,3]-predicted[:3,3]))
        angle = float(Rotation.from_matrix(predicted[:3,:3].T @ desired[:3,:3]).magnitude())
        # Independent gains: a yaw discrepancy must not suppress a valid
        # translation correction (and vice versa).
        gain = -math.expm1(-dt/self.time_constant)
        a = min(gain,self.translation_rate*dt/max(distance,1e-12))
        b = min(gain,self.rotation_rate*dt/max(angle,1e-12))
        corrected = np.eye(4)
        corrected[:3,3] = predicted[:3,3]+a*(desired[:3,3]-predicted[:3,3])
        corrected[:3,:3] = Slerp([0.,1.],Rotation.from_matrix([predicted[:3,:3],desired[:3,:3]]))([b]).as_matrix()[0]
        self.T = rigid(corrected @ np.linalg.inv(O))
        self.correction = dict(active=distance>1e-5 or angle>1e-5,
                               step_m=a*distance,step_deg=math.degrees(b*angle),
                               remaining_m=(1-a)*distance,remaining_deg=math.degrees((1-b)*angle))
