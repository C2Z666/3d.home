#!/usr/bin/env python3
"""Restart only map-dependent processes; no velocity/posture publication."""
import fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path
from map_versions import catalog,validate,install,atomic
ROOT=Path(__file__).resolve().parents[1]
SERVICES={
 'localization':('localization_live.py','run_localization.sh',[]),
 'cloud_scan':('cloud_scan.py','run_cloud_scan.sh',[]),
 'planner_preview':('planner_preview.launch.py','run_planner_preview.sh',['scene:=old']),
 'navigation_nav2':('nav2_bringup.launch.py','run_nav2.sh',['floor:=lower']),
 'web_observation':('web_view.py','run_web.sh',['--host','0.0.0.0','--port','8767','--scene','old'])}

def web_pids(root, proc_root=Path('/proc')):
    expected=str(Path(root)/'scripts/web_view.py')
    found=[]
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():continue
        try:args=proc.joinpath('cmdline').read_bytes().decode().split('\0')
        except (OSError,UnicodeError):continue
        if expected in args:found.append(int(proc.name))
    return found

def stop(name):
    if name=='web_observation':
        # Include manually launched viewers absent from the managed PID file.
        for pid in web_pids(ROOT):
            try:os.kill(pid,signal.SIGTERM)
            except ProcessLookupError:continue
        for _ in range(80):
            if not web_pids(ROOT):return
            time.sleep(.1)
        for pid in web_pids(ROOT):
            try:os.kill(pid,signal.SIGKILL)
            except ProcessLookupError:pass
        return
    pidfile=ROOT/'work'/f'{name}.pid'
    if not pidfile.exists():return
    pid=int(pidfile.read_text());proc=Path('/proc')/str(pid)
    if not proc.exists():return
    cmd=proc.joinpath('cmdline').read_bytes().decode().replace('\0',' ')
    marker=SERVICES[name][0]
    if str(ROOT) not in cmd or marker not in cmd or os.getpgid(pid)!=pid:
        raise RuntimeError(f'进程身份不符，未停止 {name}: {pid} {cmd}')
    os.killpg(pid,signal.SIGTERM)
    for _ in range(80):
        time.sleep(.1)
        try:os.killpg(pid,0)
        except ProcessLookupError:break
    else:
        os.killpg(pid,signal.SIGKILL);time.sleep(.3)

def start(name):
    _,script,args=SERVICES[name]
    with (ROOT/'work'/f'{name}.log').open('ab') as log:
        p=subprocess.Popen(['bash',str(ROOT/'scripts'/script)]+args,cwd=ROOT,env={**os.environ,'D1_SCENE':'old'},stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (ROOT/'work'/f'{name}.pid').write_text(str(p.pid))
    time.sleep(1)
    if p.poll() is not None:raise RuntimeError(name+' 启动退出')

def main():
    name=sys.argv[1];previous=catalog(ROOT)['active'];changed=False;stopped=False
    leases=[]
    def status(phase,**kw):
        d=dict(phase=phase,version=name,time=time.time(),**kw)
        atomic(ROOT/'work/map_switch_status.json',json.dumps(d,ensure_ascii=False).encode());print(d,flush=True)
    try:
        for lockname in ('map_switch.lock','start_stack.lock','navigation_mission.lock'):
            f=(ROOT/'work'/lockname).open('a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(f)
        status('validating');validate(ROOT,name)
        # Allow the POST response to be delivered before restarting its server.
        time.sleep(2);status('stopping');stopped=True
        for s in SERVICES:stop(s)
        status('installing');changed=True;install(ROOT,name)
        for s in SERVICES:start(s)
        status('awaiting_localization',message='已切换；等待新地图视觉初始化，未发送运动指令')
    except Exception as e:
        status('failed',error=str(e))
        if changed:
            for s in SERVICES:stop(s)
            install(ROOT,previous)
        if stopped:
            for s in SERVICES:
                try:stop(s);start(s)
                except Exception as restart_error:print('ROLLBACK_START_FAILED',s,str(restart_error),flush=True)
        raise
    finally:
        (ROOT/'work/map_switch.pending').unlink(missing_ok=True)
        for f in leases:f.close()

if __name__=='__main__':main()
