"""Confirmed, single-task execution of server-owned preview paths."""
import collections,copy,json,math,os,signal,subprocess,threading,time,uuid
from pathlib import Path

class NavigationManager:
    def __init__(self,root,preview):
        self.root=Path(root);self.preview=preview;self.lock=threading.RLock()
        self.plans={};self.process=None;self.task=None;self.lines=collections.deque(maxlen=150);self.started_mono=None;self.finished_mono=None
        self.resetting=False;self.monitor_thread=None
        self.directory=self.root/'work/navigation';self.directory.mkdir(parents=True,exist_ok=True)

    def register(self,result):
        path=result.pop('_path')
        with self.lock:
            if self.resetting or (self.task and self.task.get('kind')=='walk_reset' and self.process and self.process.poll() is None):raise ValueError('走路重置期间不能登记路线')
            now=time.monotonic()
            self.plans={k:v for k,v in self.plans.items() if now-v['created']<60}
            while len(self.plans)>=8:self.plans.pop(next(iter(self.plans)))
            token=uuid.uuid4().hex
            self.plans[token]={'created':now,'plan':copy.deepcopy(result),'path':path}
            result.update(plan_id=token,execute_valid_for_sec=60)
        return result

    def start(self,data):
        with self.lock:
            if self.resetting:raise ValueError('正在停止旧任务并恢复走路步态')
            from map_versions import busy
            if busy(self.root):raise ValueError('地图正在切换')
            if data.get('confirmed') is not True:raise ValueError('需要明确确认后才能执行导航')
            if self.process is not None and self.process.poll() is None:raise ValueError('已有导航进程，请等待停止完成')
            if self.task and self.task['state'] not in ('SUCCEEDED','CANCELED','FAILED'):raise ValueError('已有导航任务，请先停止或等待结束')
            runtime=data.get('max_runtime',10.)
            if isinstance(runtime,bool) or not isinstance(runtime,(int,float)) or not math.isfinite(runtime) or runtime<=0:raise ValueError('最大运行时间必须是大于 0 的有限秒数')
            entry=self.plans.get(data.get('plan_id'))
            if not entry or time.monotonic()-entry['created']>=60:raise ValueError('预览已过期或已执行，请重新规划')
            plan=entry['plan'];snap=self.preview.snapshot();loc=snap.get('localization',{});pose=snap.get('pose',{});sdk=snap.get('sdk',{})
            if not snap.get('navigation_ready'):raise ValueError('当前定位未就绪，不能开始导航')
            fs=snap.get('floor_state',{})
            if fs and (not fs.get('actual_floor') or fs.get('active_nav_floor')!=fs['actual_floor']):raise ValueError('实际楼层与导航地图未就绪')
            if fs and fs.get('floors',{}).get(fs['actual_floor'],{}).get('map_key')!=plan['floor']:
                raise ValueError('预览路径所属楼层已变化，请重新规划')
            if 'mission' in plan:
                from floor_routes import load_registry
                from scene_registry import scene_bundle
                registry=load_registry(self.root,scene_bundle(self.root,plan['scene']))
                if registry['revision']!=plan['mission']['registry_revision'] or fs.get('actual_floor')!=plan['mission']['source_floor']:
                    raise ValueError('楼层连接或出发楼层已变化，请重新规划')
                if any(sdk.get('stair_speed_cap',0)<c['control']['cruise_speed'] for c in plan['mission']['connections']):
                    raise ValueError('SDK 桥接未启用所需的 Stair 速度上限')
            if (loc.get('map_id'),loc.get('session'),loc.get('epoch'))!=(plan['map_id'],plan['localization_session'],plan['localization_epoch']):raise ValueError('定位身份已变化，请重新规划')
            if math.hypot(pose['x']-plan['start']['x'],pose['y']-plan['start']['y'])>.15 or abs(math.atan2(math.sin(pose['yaw']-plan['start']['yaw']),math.cos(pose['yaw']-plan['start']['yaw'])))>.25:raise ValueError('机器人起点已变化，请重新规划')
            if snap.get('sdk_age',99)>.4 or not all(sdk.get(k) is True for k in ('sdk_connected','control_held','state_fresh')) or any(sdk.get(k) is not False for k in ('soft_stop','hardware_stop')):raise ValueError('SDK bridge 未就绪或急停未解除；请查看桥接日志')
            if self.preview.node.get_publishers_info_by_topic('/d1/sdk/supervised_cmd_vel'):raise ValueError('已有其他速度发布者，拒绝并行控制')
            if self.preview.node.get_publishers_info_by_topic('/d1/sdk/stair_cmd_vel'):raise ValueError('已有楼梯速度发布者，拒绝并行控制')
            import yaml
            site=yaml.safe_load((self.root/'config/site.yaml').read_text())
            if not site.get('maps_validated') or not site.get('scan_source_validated'):raise ValueError('地图/扫描核对记录尚未完成')
            from site_identity import verify
            if verify(self.root)!=plan['map_id']:raise ValueError('地图身份与规划不符')
            task_id=uuid.uuid4().hex
            route=self.directory/(task_id+'.path.json')
            route.write_text(json.dumps({'plan':plan,'poses':entry['path']},allow_nan=False))
            args=['bash',str(self.root/'scripts/run_mission.sh'),'--live','--scene',plan['scene'],'--x',str(plan['goal']['x']),'--y',str(plan['goal']['y']),'--yaw',str(plan['goal']['yaw']),'--z',str(pose['z']),'--max-runtime',str(runtime),'--path-file',str(route),'--owner-pid',str(os.getpid())]
            if 'mission' in plan:
                args=['bash',str(self.root/'scripts/run_multifloor.sh'),'--live','--max-runtime',str(runtime),'--path-file',str(route),'--owner-pid',str(os.getpid())]
            elif fs:args+=['--floor',fs['actual_floor']]
            self.process=subprocess.Popen(args,cwd=self.root,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1,start_new_session=True)
            self.plans.pop(data['plan_id'])
            self.lines.clear();self.started_mono=None;self.finished_mono=None
            self.task=dict(task_id=task_id,state='PREPARING',goal=plan['goal'],max_runtime=runtime,created_at=time.time(),started_at=None,reason='已确认，正在检查执行条件',log_file=str(self.directory/(task_id+'.log')))
            self.task['route_connections']=plan.get('mission',{}).get('connections',[])
            self.task.update(multifloor='mission' in plan,target_floor=plan.get('target_floor',fs.get('actual_floor')),source_floor=fs.get('actual_floor'))
            thread=threading.Thread(target=self._monitor,args=(self.process,task_id),daemon=True);self.monitor_thread=thread;thread.start()
            print('[NAV_CONFIRM] '+json.dumps(self.task,ensure_ascii=False),flush=True)
            return self.snapshot()

    def _monitor(self,process,task_id):
        with (self.directory/(task_id+'.log')).open('w',buffering=1) as log:
            for line in process.stdout:
                line=line.rstrip();formatted=time.strftime('%Y-%m-%d %H:%M:%S')+' '+line
                log.write(formatted+'\n');print('[NAV '+task_id[:8]+'] '+line,flush=True)
                with self.lock:
                    self.lines.append(formatted)
                    if line.startswith('[MULTIFLOOR_EVENT] '):
                        try:
                            event=json.loads(line.split(' ',1)[1])
                            if event.get('task_id')==task_id:
                                self.task.update(state=event['state'],phase=event['state'],reason=event.get('reason',''),stair_evidence=event.get('evidence'))
                                if 'heading_monitor' in event:self.task['heading_monitor']=event['heading_monitor']
                        except (ValueError,KeyError):pass
                    if line.startswith('[NAV_EVENT] ') and not self.task.get('multifloor'):
                        try:
                            event=json.loads(line[len('[NAV_EVENT] '):])
                            if event.get('task_id')==task_id:
                                phase=event['state'];display={'WAIT':'PREPARING','ACCEPT':'RUNNING','NAVIGATE':'RUNNING','ALIGN_PATH':'ALIGNING_PATH','ALIGN_YAW':'ALIGNING','REACQUIRE_STOP':'REACQUIRING','REPLAN':'REPLANNING','DONE':'STOPPING','FAULT':'STOPPING'}.get(phase,phase)
                                self.task.update(state=display,phase=phase,recovery=event.get('recovery'))
                        except (ValueError,KeyError):pass
                    if line.startswith('[START]'):
                        if self.started_mono is None:self.started_mono=time.monotonic();self.task.update(started_at=time.time())
                        self.task.update(state='RUNNING',reason=line)
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
                    elif line.startswith('[WALK_RESET]'):self.task.update(reason=line)
                    elif line.startswith('FAULT:'):self.task.update(reason=line)
                    elif line.startswith('[WAIT]'):self.task.update(reason=line)
            code=process.wait()
            with self.lock:
                self.finished_mono=time.monotonic()
                self.task.update(state='SUCCEEDED' if code==0 else 'CANCELED' if code==130 or self.task['state']=='CANCELING' else 'FAILED',exit_code=code,finished_at=time.time())
                if code==0 and self.task.get('kind')=='walk_reset':pass
                elif code==0:
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
            return dict(ok=True,task=task,telemetry=telemetry,logs=list(self.lines) if include_logs else [],active=self.resetting or (self.process is not None and self.process.poll() is None),resetting=self.resetting)

    def close(self):
        with self.lock:
            process=self.process
            if process is not None and process.poll() is None:process.send_signal(signal.SIGINT)
        if process is not None:
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:process.terminate()

    def reset_walk(self,data):
        from map_versions import busy
        with self.lock:
            if data.get('confirmed') is not True:raise ValueError('需要明确确认切换走路步态')
            if self.resetting or (self.task and self.task.get('kind')=='walk_reset' and self.process and self.process.poll() is None):
                raise ValueError('走路重置正在进行')
            if busy(self.root):raise ValueError('地图切换中，暂不能重置')
            self.resetting=True;self.plans.clear()
            process=self.process;monitor=self.monitor_thread
            if process and process.poll() is None:
                if os.getpgid(process.pid)!=process.pid:
                    self.resetting=False;raise ValueError('无法确认旧任务进程组')
                self.task.update(state='CANCELING',reason='切换走路：先停止原任务')
                os.killpg(process.pid,signal.SIGINT)
        try:
            # Do not hold the manager lock while draining the old monitor thread.
            if process:
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=2)
            if monitor:
                monitor.join(timeout=3)
                if monitor.is_alive():raise ValueError('旧任务日志尚未退出，未切换步态')
            with self.lock:
                task_id=uuid.uuid4().hex
                args=['bash',str(self.root/'scripts/run_walk_reset.sh'),'--scene',self.preview.scene,'--owner-pid',str(os.getpid())]
                self.process=subprocess.Popen(args,cwd=self.root,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1,start_new_session=True)
                self.lines.clear();self.started_mono=time.monotonic();self.finished_mono=None
                self.task=dict(task_id=task_id,kind='walk_reset',state='PREPARING',goal=None,max_runtime=25,
                    created_at=time.time(),started_at=time.time(),reason='正在确认停稳并恢复Walk',
                    log_file=str(self.directory/(task_id+'.log')))
                self.monitor_thread=threading.Thread(target=self._monitor,args=(self.process,task_id),daemon=True)
                self.monitor_thread.start()
                return self.snapshot()
        finally:
            with self.lock:self.resetting=False

    def manual_initialize(self,data):
        import fcntl
        from floor_routes import load_registry,floor_id
        from scene_registry import scene_bundle
        from floor_map_control import activate_maps,wait
        from rcl_interfaces.srv import SetParametersAtomically
        from rcl_interfaces.msg import Parameter,ParameterValue
        with self.lock:
            if self.resetting:raise ValueError('正在重置步态')
            if data.get('confirmed') is not True:raise ValueError('请确认手动初值')
            if self.process is not None and self.process.poll() is None:raise ValueError('请先停止当前任务')
            scene=scene_bundle(self.root,self.preview.scene);registry=load_registry(self.root,scene)
            if data.get('scene')!=self.preview.scene:raise ValueError('只能标定当前启用场景')
            fid=floor_id(registry,data.get('floor'))
            for topic in ('/d1/sdk/supervised_cmd_vel','/d1/sdk/stair_cmd_vel'):
                if self.preview.node.get_publishers_info_by_topic(topic):raise ValueError('存在运动发布者，请先结束控制任务')
            lease=(self.root/'work/navigation_mission.lock').open('a')
            try:
                fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
                snap=self.preview.snapshot();loc=snap.get('localization',{})
                request=dict(data,map_id=scene['map_id'],floor_id=fid,session=loc.get('session'),epoch=loc.get('epoch'))
                client=self.preview.node.create_client(SetParametersAtomically,'/d1/localization/manual_initialize')
                try:
                    if not client.wait_for_service(timeout_sec=2):raise ValueError('手动定位服务未就绪')
                    req=SetParametersAtomically.Request()
                    req.parameters=[Parameter(name='initial_pose',value=ParameterValue(type=4,string_value=json.dumps(request,allow_nan=False)))]
                    result=wait(client.call_async(req)).result
                    if not result.successful:raise ValueError(result.reason)
                finally:self.preview.node.destroy_client(client)
                self.plans.clear()
                try:activate_maps(self.preview.node,self.root,scene,registry,fid)
                except RuntimeError as e:raise ValueError('手动初值已应用，但楼层导航地图尚未就绪：'+str(e)) from e
                self.preview.costmap_cache=None
                return dict(ok=True,message='手动初值已应用，FAST-LIO保持运行；旧规划已失效',floor_id=fid)
            except BlockingIOError:raise ValueError('当前有其他导航或标定任务')
            finally:lease.close()
