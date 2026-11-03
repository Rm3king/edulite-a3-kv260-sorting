"""Offline eye-in-hand calibration. T_A_B maps B coordinates into A, metres.

Fixed board equation: G_i X C_i = Y, G=T_base_end, X=T_end_camera,
C=T_camera_board, Y=T_base_board. No hardware or execution authorization.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from .calibration import eye_to_hand
from .geometry import transform


def fit(samples, holdout):
    if len(holdout)<3:
        raise ValueError('NEED_THREE_NEW_HELDOUT_POSES')
    if {s['name'] for s in samples}&{s['name'] for s in holdout}:
        raise ValueError('FIT_HOLDOUT_OVERLAP')
    def inverted(rows):
        return [dict(T_base_end=s['T_base_end'],T_camera_board=np.linalg.inv(transform(s['T_camera_board'])).tolist()) for s in rows]
    # Existing solver fits G X = Y D. Set D=inverse(C), then rename its outputs.
    solved=eye_to_hand(inverted(samples),inverted(holdout))
    x=transform(solved['T_end_board']);y=transform(solved['T_base_camera'])
    def consistency(rows):
        values=[]
        for s in rows:
            predicted=transform(s['T_base_end'])@x@transform(s['T_camera_board'])
            dp=float(np.linalg.norm(predicted[:3,3]-y[:3,3])*1000)
            dr=float(np.degrees(np.arccos(np.clip((np.trace(y[:3,:3].T@predicted[:3,:3])-1)/2,-1,1))))
            values.append(dict(name=s['name'],board_center_error_mm=dp,rotation_error_deg=dr))
        errors=[v['board_center_error_mm'] for v in values]
        return dict(views=values,position_rmse_mm=float(np.sqrt(np.mean(np.square(errors)))),position_max_mm=max(errors),rotation_max_deg=max(v['rotation_error_deg'] for v in values))
    return dict(schema='workcell.eye_in_hand.candidate.v1',T_end_camera=x.tolist(),T_base_board=y.tolist(),
                fit=consistency(samples),holdout=consistency(holdout),fit_views=len(samples),holdout_views=len(holdout),
                rotation_excitation_singular_values=solved['rotation_excitation_singular_values'],
                scope='Fixed-board pose consistency, not independently measured physical TCP or grasp accuracy',
                verified=False,grasp_correction_verified=False,robot_execution_authorized=False)


def ordered(corners, reference):
    grid=corners.reshape(17,17,2)
    variants=[np.rot90(g,k).reshape(-1,2).copy() for g in (grid,grid.transpose(1,0,2)) for k in range(4)]
    if reference is None:
        return min(variants,key=lambda a:float(a[0].sum())+(10000 if a[16,0]<a[0,0] else 0))
    target=reference-reference.mean(0);norm=np.linalg.norm(target)
    ranking=sorted((float(np.sqrt(np.mean(((a-a.mean(0))*norm/np.linalg.norm(a-a.mean(0))-target)**2))),i) for i,a in enumerate(variants))
    if ranking[0][0]>70 or ranking[1][0]-ranking[0][0]<30:
        raise ValueError('AMBIGUOUS_SQUARE_BOARD_ORIENTATION')
    return variants[ranking[0][1]]


def extract(folder,intrinsics):
    import cv2
    folder=Path(folder);intrinsics=Path(intrinsics);cal=json.loads(intrinsics.read_text())
    K=np.array(cal['camera_matrix']);D=np.array(cal['distortion']);reference=None;rows=[]
    obj=np.zeros((289,3),np.float32);obj[:,:2]=(np.mgrid[0:17,0:17].T.reshape(-1,2)-8)*.0045
    for meta in sorted((folder/'execution').glob('*_paired.json')):
        pair=json.loads(meta.read_text());row=dict(name=pair['name'],split=pair['split'],status='REJECTED',frames=[])
        try:
            if pair['status']!='CAPTURED' or pair['drift_rad']>.002:raise ValueError('Unstable capture')
            allcorners=[]
            for i,frame in enumerate(pair['frames']):
                f=folder/'execution'/pair['name']/frame['file'];raw=f.read_bytes()
                if hashlib.sha256(raw).hexdigest()!=frame['sha256']:raise ValueError('Image SHA mismatch')
                gray=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_GRAYSCALE)
                if gray.shape!=(400,640):raise ValueError('Image mode changed')
                found,c=cv2.findChessboardCornersSB(gray,(17,17),flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
                method='SB'
                if not found:
                    found,c=cv2.findChessboardCorners(gray,(17,17),flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
                    if found:c=cv2.cornerSubPix(gray,c,(5,5),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,50,.001))
                    method='classic_subpixel'
                row['frames'].append(dict(index=i,found=bool(found),method=method,sha256=frame['sha256']))
                if not found:raise ValueError('Full board not detected in stationary burst')
                c=ordered(c,reference)
                if reference is None:reference=c.copy()
                allcorners.append(c)
            burst=np.array(allcorners);drift=float(np.linalg.norm(burst-burst.mean(0),axis=2).max())
            if drift>1.:raise ValueError('Image motion within burst >1px')
            # Preselected middle image, never choose the image with best fit residual.
            c=allcorners[1]
            ok,r,t=cv2.solvePnP(obj,c,K,D)
            if not ok:raise ValueError('PnP failed')
            r,t=cv2.solvePnPRefineLM(obj,c,K,D,r,t)
            R=cv2.Rodrigues(r)[0];depth=(obj@R.T+t.ravel())[:,2]
            if depth.min()<=0:raise ValueError('Negative board depth')
            projected=cv2.projectPoints(obj,r,t,K,D)[0].reshape(-1,2);error=np.linalg.norm(projected-c,axis=1)
            rms=float(np.sqrt(np.mean(error**2)))
            if rms>.5 or np.percentile(error,95)>1.:raise ValueError('Image reprojection outside intrinsic validation bounds')
            C=np.eye(4);C[:3,:3]=R;C[:3,3]=t.ravel()
            row.update(status='ACCEPTED',T_camera_board=C.tolist(),selected_frame=1,rms_px=rms,p95_px=float(np.percentile(error,95)),burst_max_deviation_px=drift,actual_motor_rad=((np.array(pair['before']['actual_motor_rad'])+pair['after_actual_motor_rad'])/2).tolist(),job_sha256=pair['job_sha256'])
            overlay=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);cv2.drawChessboardCorners(overlay,(17,17),c.reshape(-1,1,2),True);cv2.imwrite(str(folder/'execution'/pair['name']/'ordered_overlay.jpg'),overlay)
        except (ValueError,FileNotFoundError) as e:
            row['reason']=str(e)
        rows.append(row)
    result=dict(intrinsics_sha256=hashlib.sha256(intrinsics.read_bytes()).hexdigest(),board_origin='Centre of 17x17 inner-corner grid',square_m=.0045,observations=rows)
    (folder/'image_observations.json').write_text(json.dumps(result,indent=2))
    print(json.dumps([dict(name=r['name'],status=r['status'],rms_px=r.get('rms_px'),reason=r.get('reason')) for r in rows]))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--extract',required=True);p.add_argument('--intrinsics',required=True);a=p.parse_args();extract(a.extract,a.intrinsics)
