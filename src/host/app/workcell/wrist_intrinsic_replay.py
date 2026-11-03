"""Offline reproduction of a frozen wrist intrinsic fit and held-out checks.

Use the existing camera Python environment. No camera/robot connection.
Outputs a report only; never changes the candidate or installs a calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise ValueError('Preserve existing replay result')
    import cv2
    import numpy as np
    cv2.setNumThreads(1)
    candidate = json.loads(args.candidate.read_text())
    digest = sha(args.candidate)
    nx, ny = candidate['board']['inner_corners']
    obj = np.zeros((nx*ny, 3), np.float64)
    obj[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)*candidate['board']['square_size_m']
    images = []
    for source in candidate['fit_sources']:
        view = args.dataset/source['view']
        if sha(view/'observations.json') != source['observation_sha256']:
            raise ValueError('Fit observation changed: '+source['view'])
        index = source['selected_frame']
        if sha(view/f'frame_{index}.jpg') != source['image_sha256']:
            raise ValueError('Fit image changed: '+source['view'])
        observation = json.loads((view/'observations.json').read_text())
        images.append(np.array(observation['frames'][index]['corners_px'], np.float32).reshape(-1, 1, 2))
    rms, k, dist, _, _ = cv2.calibrateCamera(
        [obj.astype(np.float32) for _ in images], images,
        tuple(candidate['image_size']), None, None,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-9))
    original_k = np.array(candidate['camera_matrix'], np.float64)
    original_dist = np.array(candidate['distortion'], np.float64)
    # Held-out views use the ORIGINAL frozen parameters, never the replay fit.
    holdouts = []
    fit_hashes = {source['image_sha256'] for source in candidate['fit_sources']}
    for file in sorted(args.dataset.glob('*/validation.json')):
        recorded = json.loads(file.read_text())
        if recorded.get('candidate_sha256') != digest:
            continue
        view = file.parent
        observation = json.loads((view/'observations.json').read_text())
        if observation.get('dataset_role') != 'holdout':
            raise ValueError('Held-out role missing')
        index = recorded['selected_frame']
        image_hash = sha(view/f'frame_{index}.jpg')
        if image_hash != recorded['image_sha256'] or image_hash in fit_hashes:
            raise ValueError('Held-out image changed or leaked into fit')
        points = np.array(observation['frames'][index]['corners_px'], np.float64)
        ok, r, t = cv2.solvePnP(obj, points, original_k, original_dist)
        if not ok:
            raise ValueError('Held-out PnP failed')
        r, t = cv2.solvePnPRefineLM(obj, points, original_k, original_dist, r, t)
        projection = cv2.projectPoints(obj, r, t, original_k, original_dist)[0].reshape(-1, 2)
        errors = np.linalg.norm(projection-points, axis=1)
        positive = bool(np.all((obj@cv2.Rodrigues(r)[0].T+t.reshape(1, 3))[:, 2] > 0))
        vrms = float(np.sqrt(np.mean(errors**2)))
        p95 = float(np.percentile(errors, 95))
        policy = candidate['validation_plan']
        holdouts.append(dict(view=view.name, rms_px=vrms, p95_px=p95,
                             passed=positive and vrms <= policy['max_each_view_rms_px']
                             and p95 <= policy['max_each_view_p95_px']))
    result = dict(candidate_sha256=digest, fit_views=len(images), fit_rms_px=rms,
                  k_max_abs_difference=float(np.abs(k-original_k).max()),
                  distortion_max_abs_difference=float(np.abs(dist.reshape(-1)-original_dist).max()),
                  holdouts=holdouts, motor_commands=0, calibration_deployed=False,
                  scope='Offline reproducibility and image reprojection only; not metric grasp accuracy')
    result['passed'] = (result['k_max_abs_difference'] < 1e-5
                        and result['distortion_max_abs_difference'] < 1e-5
                        and len(holdouts) == candidate['validation_plan']['required_views']
                        and all(v['passed'] for v in holdouts))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as output:
        output.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
