"""DWB terminal control adapted from d1nav_safe v6.9.8+; no ROS authority.

Position tracking belongs to DWB. Final heading uses an error-based command,
not a measured-speed-plus-one-cycle clamp. The SDK bridge ramps that command.
"""
import math

POSITION_TOLERANCE = .15
POSITION_RELEASE = .20
YAW_TOLERANCE = .10
ALIGN_GAIN = 2.0
ALIGN_TURN_CAP = .30
ALIGN_TIMEOUT = 45.0
ALIGN_PROGRESS_TIMEOUT = 12.0


RECOVERABLE_INPUT_PHASES = frozenset(('ACCEPT','NAVIGATE','ALIGN_PATH','ALIGN_YAW',
    'STOPPING','INPUT_PAUSE','REACQUIRE_STOP','REPLAN'))


def settled_goal_action(pose, target):
    if math.hypot(pose[0]-target[0],pose[1]-target[1]) > POSITION_RELEASE:
        return 'REPLAN'
    return 'ALIGN_YAW' if abs(wrap(target[2]-pose[2])) >= .13 else 'DONE'


def replan_errors(start, end, current, target, endpoint_tolerance):
    start_error=math.hypot(start[0]-current[0],start[1]-current[1])
    end_error=math.hypot(end[0]-target[0],end[1]-target[1])
    reason=('REPLAN_START_CHANGED' if start_error>POSITION_RELEASE else
            'REPLAN_ENDPOINT_MISMATCH' if end_error>endpoint_tolerance+1e-6 else '')
    return reason,dict(start_error_m=start_error,end_error_m=end_error,
                      start_limit_m=POSITION_RELEASE,end_limit_m=endpoint_tolerance)


def scan_input_state(health):
    """Return (usable, recoverable, explanation) for the cloud-scan producer.

    Its 0.4s ready deadline can expire before the supervisor's 0.5s reception
    deadline. That is a reason to hold zero and wait, not a permanent fault.
    Explicit malformed clouds/frame errors must never use the stale fallback.
    """
    reason=str(health.get('reason') or '').strip()
    transient=('stale cloud','no odometry bracket; no extrapolation',
               'odometry gap','waiting for calibrated cloud and odometry')
    if reason in transient:
        return False,True,reason
    if reason:
        return False,False,reason
    if health.get('ready') is True:
        return True,True,'ready'
    return False,True,'扫描暂时无新鲜输出，停车等待恢复'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def initial_path_heading(points, pose):
    """Use route geometry, never the user's final orientation, for departure."""
    for point in points:
        dx, dy = point['x']-pose[0], point['y']-pose[1]
        if math.hypot(dx, dy) >= .20:
            return math.atan2(dy, dx)
    dx, dy = points[-1]['x']-pose[0], points[-1]['y']-pose[1]
    return math.atan2(dy, dx) if math.hypot(dx, dy) > POSITION_TOLERANCE else pose[2]


class HeadingAlignment:
    def __init__(self, pose, target, now, odom_pose, check_goal_position=True):
        self.target = target
        self.started = self.progress_at = now
        self.best_angle = abs(wrap(target[2]-pose[2]))
        self.anchor = pose[:]
        self.odom_anchor = odom_pose[:]
        self.check_goal_position = check_goal_position

    def diagnostics(self, pose, odom_pose):
        dx,dy = odom_pose[0]-self.odom_anchor[0],odom_pose[1]-self.odom_anchor[1]
        angle = self.anchor[2]-self.odom_anchor[2]
        px = self.anchor[0]+math.cos(angle)*dx-math.sin(angle)*dy
        py = self.anchor[1]+math.sin(angle)*dx+math.cos(angle)*dy
        return dict(odom_drift_m=math.hypot(dx,dy),
                    map_alignment_shift_m=math.hypot(pose[0]-px,pose[1]-py),
                    goal_distance_m=math.hypot(pose[0]-self.target[0],pose[1]-self.target[1]))

    def command(self, pose, now, odom_pose, yaw_rate=0.):
        stats = self.diagnostics(pose,odom_pose)
        error = wrap(self.target[2]-pose[2])
        details = ' odom_drift=%.3fm map_alignment_shift=%.3fm goal_distance=%.3fm' % tuple(stats.values())
        if stats['odom_drift_m'] > .12:
            return 0., '[ALIGN_ODOM_DRIFT] 转向期间FAST-LIO机身位移超过0.12m，停车复核并重新规划；'+details
        if self.check_goal_position and stats['goal_distance_m'] > POSITION_RELEASE:
            return 0., '[ALIGN_GOAL_SHIFT] 地图中终点距离超过%.2fm，需要重新接近目标；' % POSITION_RELEASE+details
        if now-self.started >= ALIGN_TIMEOUT:
            return 0., '[ALIGN_TIMEOUT] 终点朝向调整超过45秒'
        if abs(error) <= self.best_angle-.05:
            self.best_angle, self.progress_at = abs(error), now
        if abs(error) <= YAW_TOLERANCE:
            return 0., ''
        if now-self.progress_at >= ALIGN_PROGRESS_TIMEOUT:
            return 0., '[ALIGN_NO_PROGRESS] 终点朝向误差连续12秒没有改善0.05rad'
        if self.check_goal_position:
            # Brake before the tolerance boundary; do not reverse while coasting.
            predicted = error - .25*yaw_rate
            if error*yaw_rate > 0 and (error*predicted <= 0 or abs(predicted) <= YAW_TOLERANCE):
                return 0., ''
            return max(-ALIGN_TURN_CAP,min(ALIGN_TURN_CAP,.8*error)), ''
        return max(-ALIGN_TURN_CAP, min(ALIGN_TURN_CAP, ALIGN_GAIN*error)), ''


def rotation_clearance(grid, position, radius, details=None):
    """Require a full padded-body turn disk to be known and collision free.

    OccupancyGrid from Nav2: 99 is ALREADY inflated inscribed cost, 100
    is an obstacle source, -1 unknown. Test the ROBOT CENTRE against 99;
    test the full padded turn disk against 100/unknown. Applying 99 to
    the full disk would add the robot's inscribed radius a second time.
    Intersect cells as squares (nearest point), not just cell centres. Unlike
    the reference's fail-open fallback, missing/stale maps never permit motion.
    Freshness/frame checks are the caller's responsibility.
    """
    info = grid.info
    if details is None:details={}
    details.update(radius_m=radius,ignored_inflated_cells=0,centre_cost=None,blocked_cell=None)
    res = info.resolution
    if (not math.isfinite(res) or res <= 0 or info.width <= 0 or info.height <= 0
            or len(grid.data) != info.width*info.height
            or not all(math.isfinite(v) for v in (*position, radius)) or radius <= 0):
        return '局部代价地图尺寸或转向几何无效'
    angle = quaternion_yaw(info.origin.orientation)
    dx, dy = position[0]-info.origin.position.x, position[1]-info.origin.position.y
    x, y = math.cos(angle)*dx+math.sin(angle)*dy, -math.sin(angle)*dx+math.cos(angle)*dy
    centre_col,centre_row=math.floor(x/res),math.floor(y/res)
    if not (0<=centre_col<info.width and 0<=centre_row<info.height):
        return '机器人中心超出局部代价地图'
    centre_cost=int(grid.data[centre_row*info.width+centre_col]);details['centre_cost']=centre_cost
    if centre_cost<0 or centre_cost>=99:
        details['blocked_cell']=[centre_col,centre_row,centre_cost]
        return '机器人中心处于%s，格点(%d,%d)，代价%d' % ('未知区域' if centre_cost<0 else '不可通行区域',centre_col,centre_row,centre_cost)
    for row in range(math.floor((y-radius)/res), math.floor((y+radius)/res)+1):
        for col in range(math.floor((x-radius)/res), math.floor((x+radius)/res)+1):
            nearest_x = max(0., abs((col+.5)*res-x)-res/2)
            nearest_y = max(0., abs((row+.5)*res-y)-res/2)
            if math.hypot(nearest_x, nearest_y) > radius:
                continue
            if not (0 <= col < info.width and 0 <= row < info.height):
                return '转向包络超出局部代价地图'
            cost = int(grid.data[row*info.width+col])
            if cost==99:details['ignored_inflated_cells']+=1
            if cost < 0 or cost >= 100:
                details['blocked_cell']=[col,row,cost]
                return '半径%.3fm的完整转向包络内%s，格点(%d,%d)，代价%d；膨胀代价99不重复叠加机身尺寸' % (radius,'存在未知区域' if cost<0 else '存在真实障碍栅格',col,row,cost)
    return ''


class MotionGuard:
    """Reference's sustained-effort guard, plus the observed reverse failure.

    Brief gait oscillations are allowed; persistent wrong-direction motion or
    no response is stopped with a reason. This never manufactures commands.
    """
    def __init__(self):
        self.since = {}
        self.progress_anchor=None
        self.progress_at=None

    def reset(self):
        self.since.clear();self.progress_anchor=None;self.progress_at=None

    def check(self, vx, wz, measured, now, position):
        mx, my, mw = measured
        if not all(math.isfinite(v) for v in (vx, wz, mx, my, mw,*position)):
            return '[INVALID_MOTION] 指令或实测速度不是有限数值'
        if vx>=.06:
            if self.progress_anchor is None:
                self.progress_anchor=tuple(position);self.progress_at=now
            displacement=math.hypot(position[0]-self.progress_anchor[0],position[1]-self.progress_anchor[1])
            if displacement>=.02:
                self.progress_anchor=tuple(position);self.progress_at=now
            elif now-self.progress_at>=5.:
                return '[NO_MOTION_RESPONSE] 持续请求前进5秒但FAST-LIO净位移不足0.02m；command=(%.3f,%.3f) measured=(%.3f,%.3f,%.3f) displacement=%.4fm' % (vx,wz,mx,my,mw,displacement)
        else:
            self.progress_anchor=None;self.progress_at=None
        conditions = {
            'UNEXPECTED_REVERSE': (vx >= 0 and mx < -.05, .7, '未请求后退但实测持续后退'),
            'WRONG_TURN_DIRECTION': (abs(wz) >= .10 and wz*mw < -.005, 1., '实测转向持续与指令相反'),
            'NO_TURN_RESPONSE': (abs(wz) >= .10 and abs(mw) < .025, 3., '持续请求转向但机身没有转动'),
        }
        for key, (condition, duration, reason) in conditions.items():
            if condition:
                self.since.setdefault(key, now)
                if now-self.since[key] >= duration:
                    return '[%s] %s；command=(%.3f,%.3f) measured=(%.3f,%.3f,%.3f)' % (key, reason, vx, wz, mx, my, mw)
            else:
                self.since.pop(key, None)
        return ''
