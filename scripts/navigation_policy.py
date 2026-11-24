"""Shared input policy and pure recovery decisions (no ROS imports)."""
import math
from functools import lru_cache
from pathlib import Path
import yaml


@lru_cache(maxsize=4)
def load_policy(root=None):
    root = Path(root) if root else Path(__file__).resolve().parents[1]
    values = yaml.safe_load((root/'config/navigation_policy.yaml').read_text(encoding='utf-8'))
    bounds = dict(observation_source_max_age_sec=(.1, 1.), future_tolerance_sec=(.001, .1),
                  recovery_stable_sec=(.05, 1.), visual_steady_hz=(.1, 2.),
                  visual_burst_hz=(.5, 5.), visual_burst_sec=(1., 10.),
                  visual_consistency_sec=(.5, 5.))
    for key, (lo, hi) in bounds.items():
        value = values.get(key)
        if type(value) not in (float, int) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError('invalid navigation policy: '+key)
    return values


def source_fresh(now, stamp, policy=None):
    p = policy or load_policy()
    return (stamp is not None and math.isfinite(stamp) and math.isfinite(now) and
            -p['future_tolerance_sec'] < now-stamp < p['observation_source_max_age_sec'])


def configure_costmaps(config, policy=None):
    """Inject the same source-age policy into runtime AND preview costmaps."""
    p = policy or load_policy()
    for name in ('global_costmap', 'local_costmap'):
        layer = config[name][name]['ros__parameters']['obstacle_layer']
        layer['source_max_age_sec'] = p['observation_source_max_age_sec']
        layer['future_tolerance_sec'] = p['future_tolerance_sec']
        # Nav2's observation expected_update_rate is an interval in seconds.
        layer['obstacles']['expected_update_rate'] = p['observation_source_max_age_sec']
    return config


class FreshWindow:
    """Two advancing source samples over one shared recovery interval."""
    def __init__(self, duration=None):
        self.duration = load_policy()['recovery_stable_sec'] if duration is None else duration
        self.reset()

    def reset(self):
        self.first = None
        self.previous = None

    def observe(self, now, stamps, valid=True):
        if not valid or any(s is None or not math.isfinite(s) for s in stamps):
            self.reset()
            return False
        if self.previous:
            last, old = self.previous
            if now-last > .5 or any(s < o for s, o in zip(stamps, old)):
                self.reset()
            elif any(s == o for s, o in zip(stamps, old)):
                return False
        if self.first is None:
            self.first = now
        self.previous = (now, tuple(stamps))
        return now-self.first >= self.duration-1e-9

    def confirmed(self, now):
        # A consumer may finish its asynchronous check between source updates.
        # Reusing this evidence must not require another expensive geometry check.
        return (self.previous is not None and self.first is not None and
                now-self.previous[0]<=.5 and self.previous[0]-self.first>=self.duration-1e-9)


def follow_path_failure(detail, route_state='DATA_UNAVAILABLE'):
    """Legacy Empty results require fresh geometry; never retry every abort."""
    code = getattr(detail, 'error_code', None)
    if code is not None:
        return 'BLOCKED' if code in (104, 105, 106) else 'FAULT'
    if route_state == 'BLOCKED':
        return 'BLOCKED'
    if route_state in ('CLEAR_PENDING', 'CLEAR_CONFIRMED'):
        return 'FAULT'
    return 'REVIEW'
