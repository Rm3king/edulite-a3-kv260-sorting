"""Review first wrist image only; never enable, move, reset or disable motors."""
import argparse
import base64
import hashlib
import json
import time
from pathlib import Path
from .live_session import Session


FETCH = '''import sys,json,base64
from pathlib import Path
p=json.load(sys.stdin);root=Path(p['directory']);out=root/'execution'
cmd=Path('/proc/'+str(p['pid'])+'/cmdline').read_bytes()
if str(root/'wrist_handeye_motion.py').encode() not in cmd:raise RuntimeError('Owner mismatch')
f=out/'pose_00_paired.json'
value={'ready':f.exists()}
if f.exists():
 value['pair']=json.loads(f.read_text())
 value['image']=base64.b64encode((out/'pose_00/frame_1.jpg').read_bytes()).decode()
print(json.dumps(value))
'''
PUBLISH = '''import json,sys
from pathlib import Path
p=json.load(sys.stdin);root=Path(p['owner']['directory']);out=root/'execution'
cmd=Path('/proc/'+str(p['owner']['pid'])+'/cmdline').read_bytes()
if str(root/'wrist_handeye_motion.py').encode() not in cmd:raise RuntimeError('Owner mismatch')
pair=json.loads((out/'pose_00_paired.json').read_text());r=p['review']
if pair['job_sha256']!=r['job_sha256'] or pair['frames'][1]['sha256']!=r['image_sha256']:raise RuntimeError('Stale image review')
f=out/'initial_capture_review.json'
if f.exists():raise RuntimeError('Review already supplied')
t=out/'initial_capture_review.tmp';t.write_text(json.dumps(r));t.replace(f)
print(json.dumps({'review_published':True,'accepted':r['full_grid_found']}))
'''


def review(folder):
    import cv2
    import numpy as np
    folder=Path(folder)
    owner=json.loads((folder/'owner.json').read_text())
    session=Session('aipc_app')
    deadline=time.monotonic()+180.
    while time.monotonic()<deadline:
        value=session.remote(FETCH,owner,timeout=3)
        if value['ready']:
            break
        time.sleep(.2)
    else:
        raise TimeoutError('No initial image; no continuation review issued')
    pair=value['pair'];raw=base64.b64decode(value['image']);sha=hashlib.sha256(raw).hexdigest()
    if sha!=pair['frames'][1]['sha256']:
        raise ValueError('Image digest mismatch')
    (folder/'first_view.jpg').write_bytes(raw)
    im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_GRAYSCALE)
    found,corners=cv2.findChessboardCornersSB(im,(17,17),flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    count=len(corners) if found else 0
    points=corners.reshape(-1,2) if found else None
    margin=float(min(points[:,0].min(),points[:,1].min(),im.shape[1]-1-points[:,0].max(),im.shape[0]-1-points[:,1].max())) if found else None
    accepted=bool(found and count==289 and margin>=15 and pair['status']=='CAPTURED')
    result=dict(full_grid_found=accepted,detector_found=bool(found),corner_count=count,
                margin_px=margin,image_sha256=sha,job_sha256=pair['job_sha256'],
                scope='Initial framing only; does not approve calibration or motor disable')
    (folder/'first_view_review.json').write_text(json.dumps(result,indent=2))
    response=session.remote(PUBLISH,dict(owner=owner,review=result),timeout=3)
    print(json.dumps(dict(result=result,response=response)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    parser.add_argument('--review-attended',action='store_true')
    args=parser.parse_args()
    if args.review_attended:
        review(args.out)
    else:
        print('DRY_RUN: no camera, board or motor access')
