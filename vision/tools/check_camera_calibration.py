import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
import json
import math
sys.path.insert(0, str(ROOT / 'tests/.build/vision_deps'))
import cv2
import numpy as np

data=json.loads((ROOT / 'vision/camera.json').read_text())
cam=data['camera']
K=np.array([[cam['fx'],0,cam['cx']],[0,cam['fy'],cam['cy']],[0,0,1]],dtype=np.float64)
dist=np.array(cam['distortion'],dtype=np.float64)
obj=np.zeros((54,3),np.float32)
obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*20
views=[]
for name in cam['images']:
    gray=cv2.imread(str((ROOT / 'vision/captures/chessboard')/name),0)
    ok,pts=cv2.findChessboardCorners(gray,(9,6))
    assert ok
    pts=cv2.cornerSubPix(gray,pts,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
    views.append((name,np.asarray(pts,np.float32).reshape(-1,1,2)))

def evaluate(entries,matrix,coeff):
    results=[]
    for name,pts in entries:
        ok,rv,tv=cv2.solvePnP(obj,pts,matrix,coeff)
        assert ok
        proj,_=cv2.projectPoints(obj,rv,tv,matrix,coeff)
        error=np.linalg.norm(pts.reshape(-1,2)-proj.reshape(-1,2),axis=1)
        rot,_=cv2.Rodrigues(rv)
        tilt=math.degrees(math.acos(min(1,max(-1,abs(float(rot[2,2]))))))
        results.append({'image':name,'rms_px':float(np.sqrt(np.mean(error**2))),
                        'max_corner_px':float(error.max()),'tilt_deg':tilt})
    return results

full=evaluate(views,K,dist)
train=[v for i,v in enumerate(views) if i%5]
holdout=[v for i,v in enumerate(views) if not i%5]
rms,splitK,splitD,_,_=cv2.calibrateCamera([obj]*len(train),[p for _,p in train],(640,480),None,None)
held=evaluate(holdout,splitK,splitD)
allpts=np.concatenate([p.reshape(-1,2) for _,p in views])
report={'board_squares':[10,7],'inner_corners':[9,6],'square_mm':20,
        'total_images':len(list((ROOT / 'vision/captures/chessboard').glob('*.bmp'))),
        'used_images':len(views),'rms_px':cam['rms_px'],
        'median_view_rms_px':float(np.median([v['rms_px'] for v in full])),
        'max_view_rms_px':max(v['rms_px'] for v in full),
        'observed_corner_bounds_px':[allpts.min(axis=0).tolist(),allpts.max(axis=0).tolist()],
        'tilt_range_deg':[min(v['tilt_deg'] for v in full),max(v['tilt_deg'] for v in full)],
        'worst_views':sorted(full,key=lambda v:v['rms_px'],reverse=True)[:6],
        'holdout':{'training_count':len(train),'validation_count':len(holdout),
                   'training_rms_px':float(rms),'mean_validation_view_rms_px':float(np.mean([v['rms_px'] for v in held])),
                   'max_validation_view_rms_px':max(v['rms_px'] for v in held),
                   'fx_change_percent':float((splitK[0,0]/K[0,0]-1)*100),
                   'fy_change_percent':float((splitK[1,1]/K[1,1]-1)*100),
                   'note':'Every fifth successful image by filename withheld. Related capture poses may appear in both sets; not independent physical accuracy validation.'}}
(ROOT / 'vision/camera_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
