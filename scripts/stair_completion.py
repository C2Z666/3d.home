"""Pure landing detector: ascent evidence precedes every completion condition."""
import collections, math
from landing_line import crossed


class LandingDetector:
    def __init__(self, connection):
        self.cfg=connection['completion'];self.line_config=connection.get('arrival_line');self.line=None;self.reference_yaw=None
        self.connection_id=connection['id'];self.target_floor=connection['to_floor']
        self.origin=None;self.last_stamp=None;self.rows=collections.deque()
        self.ascended=False;self.level_start=None;self.level_origin=None
        self.evidence={}

    def update(self, sample):
        t=sample['stamp']
        if self.last_stamp is not None and t<=self.last_stamp:return False
        if self.last_stamp is not None and t-self.last_stamp>.4:
            self.rows.clear();self.level_start=None;self.level_origin=None
        self.last_stamp=t
        if self.origin is None:self.origin=dict(sample)
        o=self.origin;c=self.cfg
        dx,dy=sample['x']-o['x'],sample['y']-o['y']
        heading=o['yaw'] if self.reference_yaw is None else self.reference_yaw
        forward=dx*math.cos(heading)+dy*math.sin(heading);rise=sample['z']-o['z']
        if rise>=c['min_rise_m'] and forward>=c['min_forward_m']:self.ascended=True
        level=abs(sample['roll'])<=c['level_roll_rad'] and abs(sample['pitch'])<=c['level_pitch_rad']
        if not self.ascended or not level:
            self.level_start=None;self.level_origin=None;self.rows.clear()
        else:
            if self.level_start is None:self.level_start=t;self.level_origin=forward
            self.rows.append((t,sample['z']))
            while len(self.rows)>1 and t-self.rows[1][0]>=c['stable_sec']:self.rows.popleft()
        span=max((z for _,z in self.rows),default=0)-min((z for _,z in self.rows),default=0)
        stable=bool(self.rows and t-self.rows[0][0]>=c['stable_sec'] and span<=c['height_span_m'])
        level_travel=0. if self.level_origin is None else forward-self.level_origin
        posture=stable and level_travel>=c['level_travel_m']
        line_pass,line_distance=crossed(self.line,sample,self.target_floor,self.connection_id)
        enabled=self.line_config is not None
        if enabled and self.line is None:line_pass=False
        complete=(sample.get('connection_id')==self.connection_id and self.ascended and posture and
                  (not enabled or line_pass is True))
        self.evidence=dict(ascended=self.ascended,rise_m=rise,forward_m=forward,level=level,
            level_travel_m=level_travel,height_span_m=span,posture_pass=posture,
            coordinate_enabled=False,coordinate_pass=None,line_enabled=enabled,line_pass=line_pass,
            line_signed_distance_m=line_distance,line_remaining_m=None if line_distance is None else max(0.,-line_distance),complete=complete)
        return complete
