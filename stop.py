#!/usr/bin/env python3
"""Stop this repository's runtime only; preserve SSH and vendor sensor services."""
import argparse,fcntl,json,os,signal,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SCRIPTS={
 'start_stack.py':'starter','switch_map.py':'starter',
 'web_view.py':'web','navigate_to_goal.py':'mission','mission_live.py':'mission',
 'nav2_bringup.launch.py':'navigation','planner_preview.launch.py':'planner',
 'cloud_scan.py':'scan','localization_live.py':'localization'}

def classify(argv,root=ROOT):
    for name,role in SCRIPTS.items():
        if str(root/'scripts'/name) in argv:return role
    if str(root/'config/fastlio_live.yaml') in argv:return 'fastlio'
    if str(root/'install/d1_host_control/lib/d1_host_control/d1_bridge') in argv:return 'bridge'
    for name,role in [('nav2.yaml','navigation')]:
        if str(root/'config'/name) in argv:return role
    if str(root/'work/planner_preview_params.yaml') in argv:return 'planner'
    return None

def processes():
    rows={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            stat=p.joinpath('stat').read_text().rsplit(')',1)[1].split()
            if stat[0]=='Z':continue
            argv=p.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            rows[int(p.name)]={'argv':argv,'group':os.getpgid(int(p.name)),'role':classify(argv),'start':stat[19]}
        except (OSError,UnicodeError,IndexError,ProcessLookupError):pass
    return rows

def groups():
    result={}
    for pid,p in processes().items():
        if p['role'] and p['group']!=os.getpgrp():result.setdefault(p['group'],set()).add(p['role'])
    return result

def log(text):
    line=time.strftime('%F %T')+' '+text;print(line,flush=True)
    with (ROOT/'work/stop.log').open('a') as f:f.write(line+'\n')

def terminate(group):
    # Group selection is based on actual argv, never on a stale PID file alone.
    if group not in groups():return
    members={pid:p['start'] for pid,p in processes().items() if p['group']==group}
    log('[STOP] process group '+str(group))
    for sig,timeout in [(signal.SIGINT,6),(signal.SIGTERM,3),(signal.SIGKILL,1)]:
        live=processes()
        if not any(pid in live and live[pid]['start']==stamp and live[pid]['group']==group for pid,stamp in members.items()):return
        try:os.killpg(group,sig)
        except ProcessLookupError:return
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if not any(p['group']==group for p in processes().values()):return
            time.sleep(.1)
    if any(p['group']==group for p in processes().values()):raise RuntimeError('process group did not exit: '+str(group))

def api(path,data=None):
    req=urllib.request.Request('http://127.0.0.1:8767/'+path,data=json.dumps(data).encode() if data is not None else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=2) as r:return json.load(r)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--dry-run',action='store_true');args=parser.parse_args()
    if args.dry_run:
        print(json.dumps({str(g):sorted(r) for g,r in groups().items()},indent=2));return 0
    (ROOT/'work').mkdir(exist_ok=True)
    leases=[]
    def lease(name):
        f=(ROOT/'work'/name).open('a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(f)
    try:
        lease('stop.lock');log('[BEGIN] stop project runtime; no movement/posture commands')
        # Prevent startup workers from creating replacements during shutdown.
        for g,roles in groups().items():
            if 'starter' in roles:terminate(g)
        lease('start_stack.lock');lease('map_switch.lock')
        try:
            n=api('navigation')
            if n.get('active'):
                api('navigation/stop',{'task_id':n['task']['task_id']});log('[CANCEL] navigation cancellation requested')
        except Exception as e:log('[INFO] web cancellation unavailable; terminate owned supervisor: '+str(e))
        # Close the web entry before stopping command producers, retaining bridge watchdog.
        for role in ['web','mission','navigation','planner','scan','localization','fastlio']:
            for g,roles in groups().items():
                if role in roles:terminate(g)
        # No producers remain. Allow >0.3s bridge watchdog and ramp-down before bridge shutdown.
        time.sleep(1.5)
        for g,roles in groups().items():
            if 'bridge' in roles:terminate(g)
        remaining=groups()
        if remaining:raise RuntimeError('owned processes remain: '+str(remaining))
        # Only remove bookkeeping for processes that no longer exist.
        live=processes()
        for p in (ROOT/'work').glob('*.pid'):
            try:
                if int(p.read_text().strip()) not in live:p.unlink()
            except (ValueError,OSError):pass
        (ROOT/'work/map_switch.pending').unlink(missing_ok=True)
        log('[STOPPED] all recognized project processes exited; SSH/vendor sensors preserved; not a physical standstill measurement')
        return 0
    except Exception as e:
        log('[STOP_FAILED] '+str(e));return 1
    finally:
        for f in leases:f.close()

if __name__=='__main__':raise SystemExit(main())
