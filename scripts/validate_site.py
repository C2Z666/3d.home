#!/usr/bin/env python3
"""Record the operator's completed web/RViz map/scan review; never infers site safety."""
import argparse,datetime
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('--confirm-map-and-scan',action='store_true',required=True)
a=p.parse_args();r=Path(__file__).resolve().parents[1];path=r/'config/site.yaml';c=yaml.safe_load(path.read_text())
runtime=yaml.safe_load((r/'config/runtime.yaml').read_text())
from site_identity import verify
verify(r)
if not runtime.get('cloud_frame'):p.error('record actual live sensor frames first')
c['maps_validated']=True;c['scan_source_validated']=True;c['operator_review_at']=datetime.datetime.now().astimezone().isoformat()
path.with_suffix('.yaml.bak').write_bytes(path.read_bytes());path.write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True))
print('Recorded operator review. Map and scan review saved for goal navigation.')
