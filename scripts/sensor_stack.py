#!/usr/bin/env python3
"""Manage only the three sensor/localization processes created for phase 3."""
import argparse,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
FAST=ROOT
SPECS=[
 ("fastlio",ROOT/"scripts/run_fastlio.sh",ROOT/"work",str(ROOT/"config/fastlio_live.yaml")),
 ("localization",ROOT/"scripts/run_localization.sh",ROOT/"work",str(ROOT/"scripts/localization_live.py")),
 ("cloud_scan",ROOT/"scripts/run_cloud_scan.sh",ROOT/"work",str(ROOT/"scripts/cloud_scan.py")),
]
def owned(spec):
 name,script,work,marker=spec
 try:
  pid=int((work/(name+".pid")).read_text())
  state=(Path("/proc")/str(pid)/"stat").read_text().split()[2]
  cmd=(Path("/proc")/str(pid)/"cmdline").read_bytes().replace(b"\0",b" ").decode()
  if state!="Z" and (marker in cmd or str(script) in cmd):return pid
 except (OSError,ValueError,IndexError):pass
 return None
def stop(spec):
 pid=owned(spec)
 if pid is None:return
 name=spec[0]
 if os.getpgid(pid)==pid:os.killpg(pid,signal.SIGINT)
 else:os.kill(pid,signal.SIGINT)
 end=time.monotonic()+5
 while time.monotonic()<end and owned(spec):time.sleep(.1)
 if owned(spec):raise RuntimeError(name+" did not stop; inspect before retry")
 print(name,"stopped",pid,flush=True)
def start(spec):
 name,script,work,marker=spec
 pid=owned(spec)
 if pid:print(name,"already running",pid,flush=True);return
 work.mkdir(exist_ok=True)
 logpath=work/(name+".log")
 if logpath.exists() and logpath.stat().st_size:
  backup=work/(name+"."+time.strftime("%Y%m%d_%H%M%S")+".log")
  logpath.rename(backup)
 log=logpath.open("ab",buffering=0)
 env=os.environ.copy()
 env.update(ROS_DOMAIN_ID="24",RMW_IMPLEMENTATION="rmw_zenoh_cpp",PYTHONNOUSERSITE="1",OPENBLAS_NUM_THREADS="1")
 proc=subprocess.Popen(["bash",str(script)],cwd=work,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (work/(name+".pid")).write_text(str(proc.pid)+"\n")
 time.sleep(1)
 if proc.poll() is not None:raise RuntimeError(name+" exited; see "+str(logpath))
 print(name,"started",proc.pid,flush=True)
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument("action",choices=["start","status","stop"])
 a=p.parse_args()
 if a.action=="stop":
  for spec in reversed(SPECS):stop(spec)
 elif a.action=="start":
  for spec in SPECS:start(spec)
 else:
  for spec in SPECS:print(spec[0],"RUNNING PID "+str(owned(spec)) if owned(spec) else "STOPPED")
if __name__=="__main__":main()
