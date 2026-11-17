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

    def invalidate(self, reason):
        # A failed manual recovery must not permanently disable an existing
        # map alignment. Actual odometry faults are gated by localization_live.
        self.required = self.recovering = False
        self.candidates.clear()
        self.reason = self.last_rejection_reason = reason

    def request(self):
        self.epoch += 1
        self.required = self.recovering = True
        self.candidates.clear()
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
            # Bound each accepted correction at the robot, independent of the
            # arbitrary odometry origin. Never jump to a single far-away match.
            alpha = min(.25, .03/max(self.delta_m, 1e-12), 1.5/max(self.delta_deg, 1e-12))
            predicted = self.T @ O
            target = np.eye(4)
            target[:3,3] = (1-alpha)*predicted[:3,3]+alpha*V[:3,3]
            target[:3,:3] = Slerp([0.,1.], Rotation.from_matrix([predicted[:3,:3],V[:3,:3]]))([alpha]).as_matrix()[0]
            self.T = rigid(target @ np.linalg.inv(O))
        self.last_visual = t
        self.visual_stale = False
        self.reason = 'accepted verified visual correction'
        return True
