#!/usr/bin/env python3
"""Record the operator's completed web/RViz map/scan review; never infers site safety."""
import argparse,datetime,math
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('--confirm-map-and-scan',action='store_true',required=True)
a=p.parse_args();r=Path(__file__).resolve().parents[1];path=r/'config/site.yaml';c=yaml.safe_load(path.read_text())
e=c.get('entry');runtime=yaml.safe_load((r/'config/runtime.yaml').read_text())
from site_identity import verify_entry
verify_entry(r,c)
if not isinstance(e,dict) or e.get('frame')!='map' or not all(isinstance(e.get(k),(int,float)) and math.isfinite(e[k]) for k in ('x','y','z','yaw')):p.error('capture the measured entry first')
if not runtime.get('cloud_frame'):p.error('record actual live sensor frames first')
c['maps_validated']=True;c['scan_source_validated']=True;c['operator_review_at']=datetime.datetime.now().astimezone().isoformat()
path.with_suffix('.yaml.bak').write_bytes(path.read_bytes());path.write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True))
print('Recorded operator review. Only the lower-platform approach is enabled; no landing inference or automatic floor switch.')
