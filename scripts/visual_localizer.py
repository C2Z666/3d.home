"""Adapted from export/py/visual.py: appearance retrieval + metric PnP only."""
import numpy as np, cv2
from geometry import calibration, transform
SIFT=cv2.SIFT_create(nfeatures=2200,contrastThreshold=.025)

MATCHING_PROFILES = {
    'strict': dict(ratio_test=.75, min_query_features=20, candidate_count=6,
                   min_matches=16, min_inliers=16, min_inlier_ratio=.30,
                   ransac_reprojection_px=3., max_reprojection_error_px=2.5,
                   min_spread_x_px=100., min_spread_y_px=60., min_coverage=.01,
                   positive_depth_ratio=.95),
    'relaxed': dict(ratio_test=.80, min_query_features=16, candidate_count=6,
                    min_matches=12, min_inliers=12, min_inlier_ratio=.20,
                    ransac_reprojection_px=4., max_reprojection_error_px=4.,
                    min_spread_x_px=80., min_spread_y_px=40., min_coverage=.005,
                    positive_depth_ratio=.95),
}


def matching_settings(profile='relaxed'):
    if profile not in MATCHING_PROFILES:
        raise ValueError('matching profile must be strict or relaxed')
    return dict(profile=profile, **MATCHING_PROFILES[profile])


def active_matching_profile(initialized, bootstrap_profile='relaxed'):
    return 'strict' if initialized else bootstrap_profile

def features(img):
    kp,d=SIFT.detectAndCompute(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY),None)
    return np.float32([k.pt for k in kp]).reshape(-1,2),d

def load_index(path):
    z=np.load(path);bank=[]
    for i,t in enumerate(z['stamps']):
        b={k:z[f'{k}_{i}'] for k in ['pose','uv','desc','world']}
        b.update(stamp=float(t),image_index=int(z['image_indices'][i]),image=cv2.imdecode(z[f'image_{i}'],cv2.IMREAD_COLOR));bank.append(b)
    return bank,z['odom_times'],z['odom_poses']

class Localizer:
    def __init__(self,bank,calibration_path,profile='relaxed'):
        self.bank=bank;self.K,_,_,self.B_C=calibration(calibration_path)
        self.settings=matching_settings(profile)
        self.matcher=cv2.FlannBasedMatcher(dict(algorithm=1,trees=5),dict(checks=64))
        self.matcher.add([np.float32(b['desc']) for b in bank]);self.matcher.train()
    def locate(self,img,t,loop=False,profile=None):
        p=matching_settings(profile or self.settings['profile'])
        candidates_diag=[]
        self.last_diagnostic={'profile':p['profile'],'reason':'no geometrically verified match','candidates':candidates_diag}
        uv,desc=features(img)
        if desc is None or len(desc)<p['min_query_features']:
            self.last_diagnostic.update(reason='fewer than %d query descriptors'%p['min_query_features'],query_features=0 if desc is None else len(desc));return None
        # Retrieval only uses appearance. Timestamp excludes the identical image and
        # for genuine loop candidates excludes recent/future observations.
        votes=np.zeros(len(self.bank),np.int32)
        for match in self.matcher.knnMatch(desc,k=1):
            if match:votes[match[0].imgIdx]+=1
        allowed=[i for i,b in enumerate(self.bank) if (t-b['stamp']>10 if loop else abs(t-b['stamp'])>.035)]
        candidates=sorted(allowed,key=lambda i:votes[i],reverse=True)[:p['candidate_count']]
        best=None
        best_failed=None
        for i in candidates:
            b=self.bank[i];pairs=cv2.BFMatcher().knnMatch(desc,b['desc'],k=2)
            ratio_matches=[a for pair in pairs if len(pair)==2 for a,c in [pair] if a.distance<p['ratio_test']*c.distance]
            pairs=ratio_matches
            # A map landmark may contribute only one correspondence.
            pairs=sorted(pairs,key=lambda m:m.distance);unique={}
            for m in pairs:unique.setdefault(m.trainIdx,m)
            pairs=list(unique.values())
            diagnostic=dict(bank_id=int(i),votes=int(votes[i]),ratio_matches=len(ratio_matches),matches=len(pairs),
                            inliers=0,reason='fewer than %d unique correspondences'%p['min_matches'])
            candidates_diag.append(diagnostic)
            if best_failed is None or diagnostic['matches']>best_failed['matches']:best_failed=diagnostic
            if len(pairs)<p['min_matches']:continue
            q=np.float32([uv[m.queryIdx] for m in pairs]);w=np.float64([b['world'][m.trainIdx] for m in pairs])
            ok,rv,tv,inliers=cv2.solvePnPRansac(w,q,self.K,None,iterationsCount=300,reprojectionError=p['ransac_reprojection_px'],confidence=.999,flags=cv2.SOLVEPNP_EPNP)
            diagnostic.update(pnp_ok=bool(ok),inliers=0 if inliers is None else len(inliers),
                              inlier_ratio=0. if inliers is None else len(inliers)/len(pairs),
                              reason='PnP inlier count/ratio rejected')
            if not ok or inliers is None or len(inliers)<p['min_inliers'] or len(inliers)/len(pairs)<p['min_inlier_ratio']:continue
            ix=inliers[:,0];rv,tv=cv2.solvePnPRefineLM(w[ix],q[ix],self.K,None,rv,tv)
            R=cv2.Rodrigues(rv)[0];C_M=np.eye(4);C_M[:3,:3]=R;C_M[:3,3]=tv[:,0]
            T=np.linalg.inv(C_M)@np.linalg.inv(self.B_C)
            projected=cv2.projectPoints(w[ix],rv,tv,self.K,None)[0][:,0]
            error=float(np.median(np.linalg.norm(projected-q[ix],axis=1)))
            coverage=float(np.ptp(q[ix,0])*np.ptp(q[ix,1])/(960*540))
            diagnostic.update(reprojection_px=error,spread_x_px=float(np.ptp(q[ix,0])),
                              spread_y_px=float(np.ptp(q[ix,1])),coverage=coverage,
                              positive_depth_ratio=float(np.mean(transform(C_M,w[ix])[:,2]>0)),
                              reason='geometric quality gate rejected')
            if error>p['max_reprojection_error_px']:diagnostic['reason']='reprojection error too high';continue
            if diagnostic['positive_depth_ratio']<p['positive_depth_ratio']:diagnostic['reason']='insufficient positive depth';continue
            if diagnostic['spread_x_px']<p['min_spread_x_px'] or diagnostic['spread_y_px']<p['min_spread_y_px']:
                diagnostic['reason']='inlier spread too small';continue
            if coverage<p['min_coverage']:diagnostic['reason']='inlier coverage too small';continue
            result={'pose':T,'bank_id':i,'inliers':len(ix),'matches':len(pairs),'reprojection_px':error,'coverage':coverage,'query_uv':q[ix],'reference_uv':np.float32([b['uv'][pairs[k].trainIdx] for k in ix])}
            diagnostic['reason']='PnP verified'
            if best is None or result['inliers']>best['inliers']:best=result
        if best:
            self.last_diagnostic.update({k:best[k] for k in ('bank_id','inliers','matches','reprojection_px','coverage')})
            self.last_diagnostic['reason']='PnP verified'
        elif best_failed:
            self.last_diagnostic.update({k:best_failed[k] for k in ('bank_id','votes','ratio_matches','matches','inliers','inlier_ratio','reprojection_px','coverage','reason') if k in best_failed})
        return best

