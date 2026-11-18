"""Confirmed, single-task execution of server-owned preview paths."""
import collections,copy,json,math,os,signal,subprocess,threading,time,uuid
from pathlib import Path

class NavigationManager:
    def __init__(self,root,preview):
        self.root=Path(root);self.preview=preview;self.lock=threading.RLock()
        self.plans={};self.process=None;self.task=None;self.lines=collections.deque(maxlen=150);self.started_mono=None;self.finished_mono=None
        self.directory=self.root/'work/navigation';self.directory.mkdir(parents=True,exist_ok=True)

    def register(self,result):
        path=result.pop('_path')
        with self.lock:
            now=time.monotonic()
            self.plans={k:v for k,v in self.plans.items() if now-v['created']<60}
            while len(self.plans)>=8:self.plans.pop(next(iter(self.plans)))
            token=uuid.uuid4().hex
            self.plans[token]={'created':now,'plan':copy.deepcopy(result),'path':path}
            result.update(plan_id=token,execute_valid_for_sec=60)
        return result

    def start(self,data):
        with self.lock:
            from map_versions import busy
            if busy(self.root):raise ValueError('地图正在切换')
            if data.get('confirmed') is not True:raise ValueError('需要明确确认后才能执行导航')
            if self.task and self.task['state'] not in ('SUCCEEDED','CANCELED','FAILED'):raise ValueError('已有导航任务，请先停止或等待结束')
            runtime=data.get('max_runtime',10.)
            if isinstance(runtime,bool) or not isinstance(runtime,(int,float)) or not math.isfinite(runtime) or runtime<=0:raise ValueError('最大运行时间必须是大于 0 的有限秒数')
            entry=self.plans.get(data.get('plan_id'))
            if not entry or time.monotonic()-entry['created']>=60:raise ValueError('预览已过期或已执行，请重新规划')
            plan=entry['plan'];snap=self.preview.snapshot();loc=snap.get('localization',{});pose=snap.get('pose',{});sdk=snap.get('sdk',{})
            if not snap.get('navigation_ready'):raise ValueError('当前定位未就绪，不能开始导航')
            if (loc.get('map_id'),loc.get('session'),loc.get('epoch'))!=(plan['map_id'],plan['localization_session'],plan['localization_epoch']):raise ValueError('定位身份已变化，请重新规划')
            if math.hypot(pose['x']-plan['start']['x'],pose['y']-plan['start']['y'])>.15 or abs(math.atan2(math.sin(pose['yaw']-plan['start']['yaw']),math.cos(pose['yaw']-plan['start']['yaw'])))>.25:raise ValueError('机器人起点已变化，请重新规划')
            if snap.get('sdk_age',99)>.4 or not all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh')) or any(sdk.get(k) is not False for k in ('soft_stop','hardware_stop')):raise ValueError('SDK bridge 未就绪或急停未解除；请查看桥接日志')
            if self.preview.node.get_publishers_info_by_topic('/d1/sdk/supervised_cmd_vel'):raise ValueError('已有其他速度发布者，拒绝并行控制')
            import yaml
            site=yaml.safe_load((self.root/'config/site.yaml').read_text())
            if not site.get('maps_validated') or not site.get('scan_source_validated'):raise ValueError('地图/扫描核对记录尚未完成')
            from site_identity import verify
            if verify(self.root)!=plan['map_id']:raise ValueError('地图身份与规划不符')
            task_id=uuid.uuid4().hex
            route=self.directory/(task_id+'.path.json')
            route.write_text(json.dumps({'plan':plan,'poses':entry['path']},allow_nan=False))
            args=['bash',str(self.root/'scripts/run_mission.sh'),'--live','--scene',plan['scene'],'--x',str(plan['goal']['x']),'--y',str(plan['goal']['y']),'--yaw',str(plan['goal']['yaw']),'--z',str(pose['z']),'--max-runtime',str(runtime),'--path-file',str(route),'--owner-pid',str(os.getpid())]
            self.process=subprocess.Popen(args,cwd=self.root,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1,start_new_session=True)
            self.plans.pop(data['plan_id'])
            self.lines.clear();self.started_mono=None;self.finished_mono=None
            self.task=dict(task_id=task_id,state='PREPARING',goal=plan['goal'],max_runtime=runtime,created_at=time.time(),started_at=None,reason='已确认，正在检查执行条件',log_file=str(self.directory/(task_id+'.log')))
            thread=threading.Thread(target=self._monitor,args=(self.process,task_id),daemon=True);thread.start()
            print('[NAV_CONFIRM] '+json.dumps(self.task,ensure_ascii=False),flush=True)
            return self.snapshot()

    def _monitor(self,process,task_id):
        with (self.directory/(task_id+'.log')).open('w',buffering=1) as log:
            for line in process.stdout:
                line=line.rstrip();formatted=time.strftime('%Y-%m-%d %H:%M:%S')+' '+line
                log.write(formatted+'\n');print('[NAV '+task_id[:8]+'] '+line,flush=True)
                with self.lock:
                    self.lines.append(formatted)
                    if line.startswith('[NAV_EVENT] '):
                        try:
                            event=json.loads(line[len('[NAV_EVENT] '):])
                            if event.get('task_id')==task_id:
                                phase=event['state'];display={'WAIT':'PREPARING','ACCEPT':'RUNNING','NAVIGATE':'RUNNING','ALIGN_PATH':'ALIGNING_PATH','ALIGN_YAW':'ALIGNING','REACQUIRE_STOP':'REACQUIRING','REPLAN':'REPLANNING','DONE':'STOPPING','FAULT':'STOPPING'}.get(phase,phase)
                                self.task.update(state=display,phase=phase,recovery=event.get('recovery'))
                        except (ValueError,KeyError):pass
                    if line.startswith('[START]'):
                        self.started_mono=time.monotonic();self.task.update(state='RUNNING',started_at=time.time(),reason=line)
                    elif line.startswith('[ALIGN_YAW]'):self.task.update(state='ALIGNING',reason=line)
                    elif line.startswith('[ALIGN_PATH]'):self.task.update(state='ALIGNING_PATH',reason=line)
                    elif line.startswith('[TRACK_PATH]'):self.task.update(state='RUNNING',reason=line)
                    elif line.startswith('[CONTROL]'):self.task.update(reason=line)
                    elif line.startswith(('[CLOSED_LOOP_POSITION]','[CLOSED_LOOP_YAW]','[POSITION_FINISHED]','[POSITION_ATTEMPT]')):self.task.update(reason=line)
                    elif line.startswith('[FINAL_ERROR]'):
                        try:
                            self.task['final_position_error_m']=float(line.split('position_error=')[1].split('m')[0])
                            self.task['final_yaw_error_rad']=float(line.split('yaw_error=')[1].split('rad')[0])
                        except (ValueError,IndexError):pass
                    elif line.startswith('[POSITION_APPROXIMATE]'):self.task.update(position_approximate=True)
                    elif line.startswith('[STOPPING]'):self.task.update(state='STOPPING',reason=line)
                    elif line.startswith('[REACQUIRE_STOP]'):self.task.update(state='REACQUIRING',reason=line)
                    elif line.startswith('[REPLAN]'):self.task.update(state='REPLANNING',reason=line)
                    elif line.startswith('[REPLAN_READY]'):self.task.update(reason=line)
                    elif line.startswith('[INPUT_PAUSE]'):self.task.update(state='INPUT_PAUSE',reason=line)
                    elif line.startswith('[INPUT_RECOVERED]'):self.task.update(state='RECOVERING',reason=line)
                    elif line.startswith('FAULT:'):self.task.update(reason=line)
                    elif line.startswith('[WAIT]'):self.task.update(reason=line)
            code=process.wait()
            with self.lock:
                self.finished_mono=time.monotonic()
                self.task.update(state='SUCCEEDED' if code==0 else 'CANCELED' if code==130 or self.task['state']=='CANCELING' else 'FAILED',exit_code=code,finished_at=time.time())
                if code==0:
                    position=self.task.get('final_position_error_m');angle=self.task.get('final_yaw_error_rad')
                    self.task['reason']=('朝向完成并确认停稳；位置误差 %.3fm，朝向误差 %.4frad%s' % (position,angle,'（位置近似）' if self.task.get('position_approximate') else '')) if position is not None and angle is not None else '执行完成并确认停稳；详见阶段日志'
                elif self.task['state']=='CANCELED':self.task['reason']='操作者已取消导航'
                elif 'FAULT:' not in self.task['reason']:self.task['reason']='执行进程退出，退出码 '+str(code)+'；请查看阶段日志'
                (self.directory/(task_id+'.result.json')).write_text(json.dumps(self.task,ensure_ascii=False,indent=2))
                print('[NAV_FINISHED] '+json.dumps(self.task,ensure_ascii=False),flush=True)

    def stop(self,data):
        with self.lock:
            if self.task and data.get('task_id')!=self.task['task_id']:raise ValueError('任务编号不匹配，请刷新状态')
            if self.process is not None and self.process.poll() is None:
                self.task.update(state='CANCELING',reason='已请求取消，等待监督器停止并退出')
                self.process.send_signal(signal.SIGINT)
                print('[NAV_CANCEL] '+self.task['task_id'],flush=True)
            return self.snapshot()

    def snapshot(self, include_logs=True):
        with self.lock:
            task=dict(self.task) if self.task else None
            if task and self.started_mono is not None:
                end=self.finished_mono if self.finished_mono is not None else time.monotonic();task['remaining_sec']=max(0.,task['max_runtime']-(end-self.started_mono))
            telemetry=None
            if task:
                try:
                    p=self.directory/(task['task_id']+'.telemetry.json')
                    if p.stat().st_size<=2*1024*1024:
                        value=json.loads(p.read_text())
                        if value.get('task_id')==task['task_id']:
                            telemetry=value;telemetry['fresh']=0<=time.time()-value.get('updated_at',0)<2
                except (OSError,ValueError):pass
            return dict(ok=True,task=task,telemetry=telemetry,logs=list(self.lines) if include_logs else [],active=self.process is not None and self.process.poll() is None)

    def close(self):
        with self.lock:
            process=self.process
            if process is not None and process.poll() is None:process.send_signal(signal.SIGINT)
        if process is not None:
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:process.terminate()
