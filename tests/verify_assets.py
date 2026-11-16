"""Read the real supplied visual bank; leave out the query frame from reference matching."""
import sys,json,hashlib,time
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from visual_localizer import load_index,Localizer
cv2.setNumThreads(2);cv2.setRNGSeed(17)
path=ROOT/'assets/output/demo/visual_map.npz';cal=ROOT/'config/calibration.yaml'
with np.load(path,allow_pickle=False) as z:
 assert str(z['calibration_sha256'])==hashlib.sha256(cal.read_bytes()).hexdigest()
bank,_,_=load_index(path);rows=[]
for i in (0,12,24,36,48,60,72,83):
 b=bank[i];loc=Localizer(bank,cal);start=time.monotonic();v=loc.locate(b['image'],b['stamp'])
 row=dict(query=i,identical_frame_excluded=True,matched=v is not None,elapsed_sec=round(time.monotonic()-start,3))
 if v:
  delta=np.linalg.inv(b['pose'])@v['pose'];row.update(bank_id=v['bank_id'],inliers=v['inliers'],reprojection_px=v['reprojection_px'],position_difference_m=float(np.linalg.norm(delta[:3,3])),rotation_difference_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())))
 rows.append(row)
report=dict(keyframes=len(bank),landmarks=sum(len(b['world']) for b in bank),calibration_matches=True,same_acquisition_only=True,independent_field_localization_verified=False,queries=rows)
(ROOT/'docs/phase3_asset_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
