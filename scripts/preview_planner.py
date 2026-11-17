"""Read-only Nav2 ComputePathToPose adapter. No navigation or velocity client."""
import math
import threading
import time
from collections import deque


class PreviewError(ValueError):
    def __init__(self, message, code='preview_error', details=None):
        super().__init__(message)
        self.code, self.details = code, details


def planner_cell(grid, pose):
    """Match Humble Navfn's worldToMap (std::round, not costmap floor)."""
    u = (pose['x']-grid['origin'][0])/grid['resolution']
    v = (pose['y']-grid['origin'][1])/grid['resolution']
    x, y = math.floor(u+.5), math.floor(v+.5)
    inside = u >= 0 and v >= 0 and 0 <= x < grid['width'] and 0 <= y < grid['height']
    return dict(x=x, y=y, cost=int(grid['data'][y*grid['width']+x]) if inside else None)


def diagnose(grid, start, goal):
    s, g = planner_cell(grid, start), planner_cell(grid, goal)
    details = dict(start_cell=s, goal_cell=g, resolution=grid['resolution'],
                   origin=grid['origin'], sampling='Navfn std::round', allow_unknown=False)
    label = f"目标栅格 ({g['x']}, {g['y']})"
    if g['cost'] is None:
        return 'goal_outside', label+' 超出规划器代价地图边界。请向地图内部移动目标。', details
    if g['cost'] >= 253:
        names = {253:'膨胀阻塞区（机器人中心不可通过）',254:'障碍物',255:'未知区域（当前禁止穿越）'}
        return 'goal_'+{253:'inflated',254:'obstacle',255:'unknown'}[g['cost']], f"{label} 的代价为 {g['cost']}，属于{names[g['cost']]}。请将目标移到可通行区域；只改变朝向不能解除此栅格限制。", details
    if s['cost'] is None:
        return 'start_outside', '当前机器人位置超出规划器代价地图，请核对定位和地图。', details
    # Navfn clears its start cell before searching; reflect that here.
    w, h, data = grid['width'], grid['height'], grid['data']
    first, target = s['y']*w+s['x'], g['y']*w+g['x']
    seen = bytearray(w*h); seen[first] = 1; queue = deque([first])
    while queue:
        i = queue.popleft()
        if i == target:
            return 'planner_failed', '起点与目标栅格可连通，但 Navfn 未生成有效路径；需结合规划器返回状态和日志检查搜索或路径提取失败。', details
        x, y = i%w, i//w
        for j in ((i-1 if x else -1), (i+1 if x+1<w else -1), (i-w if y else -1), (i+w if y+1<h else -1)):
            if j >= 0 and not seen[j] and data[j] < 253:
                seen[j] = 1; queue.append(j)
    return 'disconnected', '目标栅格可通行，但与起点之间没有连续的可通行栅格；障碍、膨胀阻塞区或未知区域截断了通道。', details


def validate_request(data, scene, maps, snapshot):
    if not isinstance(data, dict):
        raise PreviewError('请求必须为 JSON 对象')
    if data.get('scene') != scene or data.get('floor') != 'lower':
        raise PreviewError('预览服务只加载当前启动场景的 lower 地图')
    goal = {}
    for key in ('x', 'y', 'yaw'):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise PreviewError('目标 x、y、yaw 必须为有限数值')
        goal[key] = float(value)
    goal['yaw'] = math.atan2(math.sin(goal['yaw']), math.cos(goal['yaw']))
    loc = snapshot.get('localization', {})
    if not snapshot.get('pose_current') or not loc.get('ready'):
        raise PreviewError('当前融合定位无效或已过期，请先恢复定位')
    pose = snapshot.get('pose', {})
    if not all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k]) for k in ('x', 'y', 'yaw')):
        raise PreviewError('缺少有效起点位姿')
    meta = maps[scene]['lower'][0]
    dx, dy = goal['x'] - meta['origin'][0], goal['y'] - meta['origin'][1]
    a = meta['origin'][2]
    u = (math.cos(a)*dx + math.sin(a)*dy)/meta['resolution']
    v = (-math.sin(a)*dx + math.cos(a)*dy)/meta['resolution']
    if not (0 <= u < meta['width'] and 0 <= v < meta['height']):
        raise PreviewError('目标超出地图范围')
    return goal, dict(pose), (loc.get('session'), loc.get('epoch'), loc.get('map_id'))


class PreviewPlanner:
    def __init__(self, node, state, maps, scene):
        from rclpy.action import ActionClient
        from nav2_msgs.action import ComputePathToPose
        self.node, self.state, self.maps, self.scene = node, state, maps, scene
        self.action_type = ComputePathToPose
        self.client = ActionClient(node, ComputePathToPose, '/d1/preview/compute_path_to_pose')
        self.lock = threading.Lock()
        from nav2_msgs.srv import GetCostmap
        self.costmap_type = GetCostmap
        self.costmap_client = node.create_client(GetCostmap, '/d1/preview/global_costmap/get_costmap')
        self.costmap_lock = threading.Lock()
        self.costmap_cache = None

    def costmap(self, fresh=False):
        with self.costmap_lock:
            if not fresh and self.costmap_cache and time.monotonic()-self.costmap_cache['received'] < 1:
                return self.costmap_cache
            if not self.costmap_client.service_is_ready():
                raise PreviewError('规划器代价地图服务未就绪', 'costmap_unavailable')
            future = self.costmap_client.call_async(self.costmap_type.Request())
            try:
                m = self.wait(future, 3).map
            except Exception:
                self.costmap_client.remove_pending_request(future)
                raise PreviewError('读取规划器代价地图超时或失败', 'costmap_unavailable')
            meta = m.metadata
            q = meta.origin.orientation
            angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if m.header.frame_id != 'map' or abs(angle) > 1e-6 or meta.resolution <= 0 or not meta.size_x or not meta.size_y or len(m.data) != meta.size_x*meta.size_y:
                raise PreviewError('规划器代价地图坐标系或尺寸异常，无法可靠叠加', 'costmap_invalid')
            self.costmap_cache = dict(width=meta.size_x, height=meta.size_y, resolution=meta.resolution,
                origin=[meta.origin.position.x,meta.origin.position.y,angle], data=list(m.data),
                frame_id=m.header.frame_id, received=time.monotonic())
            return self.costmap_cache

    def costmap_view(self, scene, floor):
        if scene != self.scene or floor != 'lower':
            raise PreviewError('当前场景或楼层未加载预览规划器', 'costmap_scene')
        grid = self.costmap()
        return dict(ok=True, scene=self.scene, floor='lower',
                    **{k:v for k,v in grid.items() if k != 'received'},
                    age_sec=time.monotonic()-grid['received'])

    def snapshot(self):
        return self.state.snapshot(time.monotonic(), self.node.get_clock().now().nanoseconds*1e-9)

    @staticmethod
    def wait(future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise PreviewError('规划服务超时，请稍后重试')
        return future.result()

    def plan(self, data):
        from geometry_msgs.msg import PoseStamped
        from action_msgs.msg import GoalStatus
        if not self.lock.acquire(blocking=False):
            raise PreviewError('正在计算另一条路线，请稍后重试')
        handle = None
        try:
            goal, start, identity = validate_request(data, self.scene, self.maps, self.snapshot())
            if not self.client.server_is_ready():
                raise PreviewError('Nav2 预览规划服务未就绪')
            def pose(value):
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.header.stamp = self.node.get_clock().now().to_msg()
                p.pose.position.x, p.pose.position.y = value['x'], value['y']
                p.pose.orientation.z = math.sin(value['yaw']/2)
                p.pose.orientation.w = math.cos(value['yaw']/2)
                return p
            request = self.action_type.Goal()
            request.start, request.goal = pose(start), pose(goal)
            request.use_start = True
            request.planner_id = 'GridBased'
            begin = time.monotonic()
            pending = self.client.send_goal_async(request)
            try:
                handle = self.wait(pending, 4)
            except PreviewError:
                def cancel_late(future):
                    try:
                        h = future.result()
                        if h and h.accepted: h.cancel_goal_async()
                    except Exception: pass
                pending.add_done_callback(cancel_late)
                raise
            if not handle.accepted:
                raise PreviewError('Nav2 拒绝了规划请求')
            result = self.wait(handle.get_result_async(), 12)
            if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:
                try:
                    code, reason, details = diagnose(self.costmap(fresh=True), start, goal)
                except PreviewError as e:
                    raise PreviewError('规划失败；'+str(e)+'，暂时无法判断栅格原因', 'diagnosis_unavailable') from e
                details['action_status'] = result.status
                details['planner_error_code'] = getattr(result.result, 'error_code', None)
                raise PreviewError(reason, code, details)
            _, _, current_identity = validate_request(data, self.scene, self.maps, self.snapshot())
            if current_identity != identity:
                raise PreviewError('规划期间定位发生重置，请重新规划')
            path = result.result.path
            if path.header.frame_id != 'map':
                raise PreviewError('规划路径坐标系不匹配')
            points = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
            if not all(math.isfinite(x) and math.isfinite(y) for x, y in points):
                raise PreviewError('规划返回了无效坐标')
            return dict(ok=True, preview_only=True, scene=self.scene, floor='lower',
                        map_id=identity[2], localization_session=identity[0], localization_epoch=identity[1],
                        start=start, goal=goal, points=points,
                        length_m=sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points, points[1:])),
                        compute_sec=time.monotonic()-begin)
        except Exception:
            if handle is not None and handle.accepted: handle.cancel_goal_async()
            raise
        finally:
            self.lock.release()
