"""Mission-scoped recovery budgets and bounded asynchronous diagnostics (no ROS)."""
import collections,json,math,queue,threading,time
from pathlib import Path

class RecoveryBudget:
    def __init__(self):
        self.counts=dict(data_wait=0,position=0,motion=0)
        self.limits=dict(position=3,motion=3)
        self.wait_started=None;self.wait_total=0.;self.wait_category=None
        self.phase=None;self.phase_at=None;self.phase_totals=collections.defaultdict(float)
    def request(self,category):
        if category not in self.limits:raise ValueError(category)
        if self.counts[category]>=self.limits[category]:
            return '[%s_RECOVERY_LIMIT] %s恢复已达到%d次' % (category.upper(),category,self.limits[category])
        self.counts[category]+=1;return ''
    def begin_wait(self,now,category='data'):
        if self.wait_started is None:
            self.wait_started=now;self.wait_category=category
            if category=='data':self.counts['data_wait']+=1
            self.set_phase(now,'input_unavailable')
    def set_phase(self,now,phase):
        if self.wait_started is None:return
        if self.phase_at is not None:self.phase_totals[self.phase]+=max(0.,now-self.phase_at)
        self.phase=phase;self.phase_at=now
    def wait_used(self,now):
        return self.wait_total+(max(0.,now-self.wait_started) if self.wait_started is not None else 0.)
    def wait_error(self,now):
        if self.wait_started is not None and now-self.wait_started>=30:return '[DATA_WAIT_TIMEOUT] 本次零速等待超过30秒'
        if self.wait_used(now)>=60:return '[DATA_WAIT_BUDGET] 累计零速等待超过60秒'
        return ''
    def end_wait(self,now):
        if self.wait_started is not None:self.set_phase(now,'complete')
        if self.wait_started is not None:self.wait_total+=max(0.,now-self.wait_started)
        self.wait_started=None;self.wait_category=None
        self.phase=None;self.phase_at=None
    def snapshot(self,now):
        phases=dict(self.phase_totals)
        if self.phase_at is not None:phases[self.phase]=phases.get(self.phase,0.)+max(0.,now-self.phase_at)
        return dict(counts=dict(self.counts),limits=dict(self.limits),wait_total_sec=round(self.wait_used(now),2),wait_limit_sec=60,
                    wait_phase=self.phase,wait_phase_sec={k:round(v,3) for k,v in phases.items() if k!='complete'})

class ZeroCommandWindow:
    """Reports evidence; never subtracts a bias or changes motion commands."""
    def __init__(self):self.rows=collections.deque(maxlen=32)
    def update(self,now,command,sdk,odom_pose,odom_age):
        feedback=[sdk.get(k) for k in ('reported_forward_speed','reported_lateral_speed','reported_yaw_rate')]
        valid=odom_pose is not None and odom_age is not None and odom_age<.4 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in feedback+list(odom_pose))
        if not valid or sdk.get('command_zero') is not True or any(abs(v)>1e-6 for v in command):
            self.rows.clear();return None
        if self.rows and now-self.rows[-1][0]>.5:self.rows.clear()
        self.rows.append((now,list(odom_pose),feedback))
        while self.rows and now-self.rows[0][0]>5.:self.rows.popleft()
        first=self.rows[0];count=len(self.rows)
        return dict(duration_sec=round(now-first[0],2),samples=count,
                    odom_net_m=math.hypot(odom_pose[0]-first[1][0],odom_pose[1]-first[1][1]),
                    sdk_mean=[sum(r[2][i] for r in self.rows)/count for i in range(3)],
                    sdk_min=[min(r[2][i] for r in self.rows) for i in range(3)],
                    sdk_max=[max(r[2][i] for r in self.rows) for i in range(3)],
                    note='零指令下的反馈与净位移；不等同于外部实测静止，不自动补偿')

class MissionTrace:
    """Sampling never waits for disk. Two 4MiB log segments, latest UI snapshot."""
    def __init__(self,directory,task_id):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.task_id=task_id;self.route=None;self.route_revision=0;self.trajectory=collections.deque(maxlen=600)
        self.zero_window=ZeroCommandWindow();self.queue=queue.Queue(maxsize=256);self.dropped=0;self.error=None;self.closed=False
        self.thread=threading.Thread(target=self._writer,daemon=True);self.thread.start()
    def emit(self,kind,**data):
        item=dict(kind=kind,task_id=self.task_id,wall_time=time.time(),monotonic=time.monotonic(),**data)
        try:self.queue.put_nowait(item)
        except queue.Full:self.dropped+=1
    def set_route(self,points,map_id,scene):
        self.route_revision+=1
        self.route=dict(task_id=self.task_id,route_id='%s:%d'%(self.task_id,self.route_revision),frame_id='map',map_id=map_id,scene=scene,points=points,status='submitted')
        self.emit('route',route=self.route)
    def route_status(self,status):
        if self.route:
            self.route=dict(self.route,status=status);self.emit('route',route=self.route)
    def sample(self,**data):
        data['zero_command_window']=self.zero_window.update(time.monotonic(),data.get('commands',{}).get('supervised',[1,1]),data.get('sdk',{}),data.get('odom_pose'),data.get('receive_age',{}).get('odom'))
        pose=data.get('map_pose')
        if pose:self.trajectory.append(pose[:2])
        self.emit('sample',route_id=self.route.get('route_id') if self.route else None,**data)
    def _writer(self):
        log=self.directory/(self.task_id+'.telemetry.jsonl');ui=self.directory/(self.task_id+'.telemetry.json')
        route=None;trajectory=collections.deque(maxlen=600);last_ui=0.;latest={}
        while not self.closed or not self.queue.empty():
            try:item=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                def finite(v):
                    if isinstance(v,float) and not math.isfinite(v):return None
                    if isinstance(v,dict):return {k:finite(x) for k,x in v.items()}
                    if isinstance(v,(list,tuple)):return [finite(x) for x in v]
                    return v
                item=finite(item)
                encoded=(json.dumps(item,ensure_ascii=False,allow_nan=False)+'\n').encode()
                if log.exists() and log.stat().st_size+len(encoded)>4*1024*1024:log.replace(log.with_suffix('.jsonl.1'))
                with log.open('ab') as f:f.write(encoded)
                if item['kind']=='route':route=item['route']
                if item['kind']=='sample':
                    latest=item
                    if item.get('map_pose'):trajectory.append(item['map_pose'][:2])
                if item['kind']!='sample' or time.monotonic()-last_ui>=.5:
                    route=self.route
                    temp=ui.with_suffix('.json.tmp')
                    temp.write_text(json.dumps(dict(task_id=self.task_id,updated_at=time.time(),route=route,trajectory=list(trajectory),latest=latest,dropped=self.dropped,error=self.error),ensure_ascii=False,allow_nan=False))
                    temp.replace(ui);last_ui=time.monotonic()
            except Exception as e:self.error=str(e)
    def close(self):
        self.closed=True;self.thread.join(timeout=1.)
