"""Time-windowed visual correction policy; no ROS or movement commands."""
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from geometry import rigid

class FusionPolicy:
    def __init__(self):
        self.T = None
        self.last_visual = None
        self.epoch = 0
        self.required = False
        self.recovering = False
        self.candidates = []
        self.last_query = -math.inf
        self.reason = 'waiting for three consistent visual observations'
        self.delta_m = self.delta_deg = None

    def invalidate(self, reason):
        if not self.required:
            self.epoch += 1
        self.required = True
        self.recovering = False
        self.candidates.clear()
        self.reason = reason

    def request(self):
        self.epoch += 1
        self.required = True
        self.recovering = True
        self.candidates.clear()
        self.reason = 'stationary relocalization: waiting for three consistent images'

    def expire(self, t, ttl):
        if self.T is not None and self.last_visual is not None and t-self.last_visual >= ttl and not self.required:
            self.invalidate('visual correction expired; stop and request relocalization')

    def update(self, V, O, t):
        if not math.isfinite(t) or t <= self.last_query:
            self.reason = 'duplicate or reversed visual timestamp'
            return False
        self.last_query = t
        C = rigid(V @ np.linalg.inv(O))
        if self.required and not self.recovering:
            self.reason = 'relocalization required; old alignment is display-only'
            return False
        if self.T is not None and not self.recovering:
            delta = np.linalg.inv(self.T @ O) @ V
            self.delta_m = float(np.linalg.norm(delta[:3, 3]))
            self.delta_deg = float(np.rad2deg(Rotation.from_matrix(delta[:3, :3]).magnitude()))
            if self.delta_m > .15 or self.delta_deg > 5.:
                self.invalidate('visual correction exceeds 0.15m/5deg; stop and relocalize')
                return False
            # Smooth the correction's effect at the observation reference point.
            # Interpolating the transform translation itself depends on odom origin.
            predicted = self.T @ O
            target = np.eye(4)
            target[:3, 3] = .75*predicted[:3, 3] + .25*V[:3, 3]
            target[:3, :3] = Slerp([0., 1.], Rotation.from_matrix([predicted[:3,:3], V[:3,:3]]))([.25]).as_matrix()[0]
            self.T = rigid(target @ np.linalg.inv(O))
        else:
            self.candidates = [(a, B) for a, B in self.candidates if 0 < t-a <= 2.]
            if self.candidates:
                d = np.linalg.inv(self.candidates[0][1] @ O) @ V
                if np.linalg.norm(d[:3,3]) > .25 or Rotation.from_matrix(d[:3,:3]).magnitude() > np.deg2rad(10):
                    self.candidates.clear()
            self.candidates.append((t, C))
            if len(self.candidates) < 3 or t-self.candidates[0][0] < .3:
                self.reason = 'consistent visual candidates %d/3 within 2s' % len(self.candidates)
                return False
            self.T = C
            self.required = self.recovering = False
            self.candidates.clear()
        self.last_visual = t
        self.reason = 'accepted verified visual correction'
        return True
