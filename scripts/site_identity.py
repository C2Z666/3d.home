"""Bind the retained map, visual bank, calibration and saved entry."""
import hashlib, json
from pathlib import Path
FILES = ['assets/output/demo/visual_map.npz', 'config/calibration.yaml',
         'maps/lower.yaml', 'maps/lower.pgm']
def hashes(root):
    return {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in FILES}
def verify(root):
    data = json.loads((root/'config/site_identity.json').read_text(encoding='utf-8'))
    if hashes(root) != data['sha256']:
        raise ValueError('Map/calibration identity changed; review assets and recapture entry before use')
    return data['map_id']
def verify_entry(root, site):
    map_id = verify(root)
    if site.get('entry') and site.get('entry_map_id') != map_id:
        raise ValueError('Saved entry is not bound to the current map')
    return map_id
