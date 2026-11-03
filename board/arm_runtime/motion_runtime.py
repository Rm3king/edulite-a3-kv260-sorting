#!/usr/bin/env python3
"""KV260 ARM numerical runtime. Offline only: no CAN, MMIO or motor API.

Inputs are six robot joint coordinates (rad), NOT raw motor coordinates.
Poses use metres and Pinocchio RPY radians. This is a single-threaded object.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "vendor"))
from el_a3_sdk.kinematics import ELA3Kinematics
from el_a3_sdk.data_types import ArmEndPose
from el_a3_sdk.protocol import DEFAULT_JOINT_LIMITS

LIMITS = np.array([DEFAULT_JOINT_LIMITS[i] for i in range(1, 7)])


def vector6(value):
    if not isinstance(value, (list, tuple, np.ndarray)):
        raise ValueError("expected a six-element numeric sequence")
    if any(isinstance(x, (bool, np.bool_)) for x in value):
        raise ValueError("boolean is not a joint angle or pose component")
    a = np.asarray(value)
    if a.shape != (6,) or a.dtype.kind not in "if" or not np.isfinite(a).all():
        raise ValueError("expected six finite numeric values")
    return a.astype(float)


def joints(value):
    q = vector6(value)
    if np.any(q < LIMITS[:, 0]) or np.any(q > LIMITS[:, 1]):
        raise ValueError("joint position outside SDK limits")
    return q


def positive(value, name):
    if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
        raise ValueError(name + " must be positive and finite")
    return float(value)


def pose_transform(pose):
    p = vector6(pose)
    return pin.SE3(pin.rpy.rpyToMatrix(*p[3:]), p[:3])


def pose_values(transform):
    return np.r_[transform.translation, pin.rpy.matrixToRpy(transform.rotation)].tolist()


def residual(actual, target):
    return (float(np.linalg.norm(actual.translation - target.translation)),
            float(np.linalg.norm(pin.log3(actual.rotation.T @ target.rotation))))


def interpolate_pose(start, end, fraction):
    return pin.SE3(start.rotation @ pin.exp3(pin.log3(start.rotation.T @ end.rotation) * fraction),
                   start.translation + fraction * (end.translation - start.translation))


class MotionRuntime:
    def __init__(self):
        manifest_bytes = (ROOT / "model_manifest.json").read_bytes()
        for name, info in json.loads(manifest_bytes).items():
            if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != info["sha256"]:
                raise ValueError("model/SDK hash mismatch: " + name)
        self.model_id = hashlib.sha256(manifest_bytes).hexdigest()
        self.kin = ELA3Kinematics(
            urdf_path=str(ROOT / "model/el_a3_legacy.urdf"),
            inertia_config_path=str(ROOT / "model/inertia_params.yaml"))
        names = list(self.kin.model.names)[1:]
        if names != [f"L{i}_joint" for i in range(1, 8)]:
            raise ValueError("unexpected model joint ordering: " + repr(names))

    def fk(self, q):
        return self.kin.forward_kinematics_se3(joints(q).tolist())

    def ik(self, pose, seed):
        target = pose_transform(pose)
        q = self.kin.inverse_kinematics(ArmEndPose(*vector6(pose)),
                q_init=joints(seed).tolist(), max_iter=200, eps=1e-6,
                joint_limit_margin=0.0)
        if q is None:
            raise ValueError("IK failed within 200 iterations; no trajectory emitted")
        q = joints(q)
        p_err, r_err = residual(self.fk(q), target)
        if p_err > 2e-6 or r_err > 2e-6:
            raise ValueError("IK result failed independent FK residual check")
        return q

    def audit_segment(self, coefficients, start_pose, end_pose, samples=33):
        """Sample a normalized-u cubic vs Cartesian smoothstep on [0,1].

        Samples DO NOT prove a continuous error bound or collision clearance.
        Coefficients shape (6,4), ascending powers, like the HLS Horner input.
        """
        c = np.asarray(coefficients, dtype=float)
        if c.shape != (6, 4) or not np.isfinite(c).all():
            raise ValueError("expected finite 6x4 cubic coefficients")
        if type(samples) is not int or not 3 <= samples <= 4097:
            raise ValueError("samples must be integer in [3,4097]")
        # Analytic position extrema, not just sampled joint limits.
        for axis in range(6):
            points = [0., 1.]
            roots = np.roots([3*c[axis, 3], 2*c[axis, 2], c[axis, 1]])
            points += [float(r.real) for r in roots if abs(r.imag) < 1e-12 and 0 < r.real < 1]
            vals = np.polynomial.polynomial.polyval(points, c[axis])
            if np.min(vals) < LIMITS[axis, 0]-1e-12 or np.max(vals) > LIMITS[axis, 1]+1e-12:
                raise ValueError("cubic joint position extrema outside limits")
        start, end = pose_transform(start_pose), pose_transform(end_pose)
        errors = []
        for u in np.linspace(0., 1., samples):
            q = ((c[:, 3]*u+c[:, 2])*u+c[:, 1])*u+c[:, 0]
            # Remove floating point endpoint noise only; extrema checked above.
            q = np.clip(q, LIMITS[:, 0], LIMITS[:, 1])
            errors.append(residual(self.fk(q), interpolate_pose(start, end, u*u*(3-2*u))))
        return {"samples": samples, "max_position_error_m": max(x[0] for x in errors),
                "max_orientation_error_rad": max(x[1] for x in errors),
                "continuous_error_bound_verified": False}

    def plan_movel(self, q_start, target_pose, position_tolerance=0.0001,
                   orientation_tolerance=0.001, velocity_limit=0.2,
                   acceleration_limit=0.5, max_segments=64):
        """Dry-run stop-at-knot cubics. Not an executable motor command.

        Adaptive geometric subdivision, continuous-branch heuristic, analytic
        speed/acceleration bounds; finite sampling for Cartesian deviations.
        No jerk, collision, singularity margin or PL quantization guarantee yet.
        """
        q_start = joints(q_start)
        pt = positive(position_tolerance, "position tolerance")
        rt = positive(orientation_tolerance, "orientation tolerance")
        vmax = positive(velocity_limit, "velocity limit")
        amax = positive(acceleration_limit, "acceleration limit")
        if type(max_segments) is not int or not 1 <= max_segments <= 128:
            raise ValueError("max_segments must be integer in [1,128]")
        start, end = self.fk(q_start), pose_transform(target_pose)
        n = 1
        while n <= max_segments:
            segments, previous = [], q_start
            accepted = True
            for i in range(n):
                p0 = interpolate_pose(start, end, i/n)
                p1 = interpolate_pose(start, end, (i+1)/n)
                q = self.ik(pose_values(p1), previous)
                delta = q-previous
                c = np.column_stack((previous, np.zeros(6), 3*delta, -2*delta))
                audit = self.audit_segment(c, pose_values(p0), pose_values(p1))
                jump = float(np.max(np.abs(delta)))
                if jump > 0.25 or audit["max_position_error_m"] > pt or audit["max_orientation_error_rad"] > rt:
                    accepted = False
                    break
                # smoothstep max |dq/du|=1.5*|delta|, |d2q/du2|=6*|delta|.
                duration = max(0.005, 1.5*jump/vmax, np.sqrt(6*jump/amax))
                segments.append({"coefficients_rad": c.tolist(), "duration_s": float(duration),
                                 "audit": audit})
                previous = q
            if accepted:
                return {"schema": "kv260.movel.dry_run.v1", "model_id": self.model_id,
                        "executable": False, "motion_commands_sent": 0,
                        "segments": segments, "total_duration_s": sum(s["duration_s"] for s in segments),
                        "position_tolerance_m": pt, "orientation_tolerance_rad": rt,
                        "velocity_limit_rad_s": vmax, "acceleration_limit_rad_s2": amax,
                        "stops_at_every_knot": True,
                        "pending": ["continuous Cartesian error bound", "collision and singularity checks",
                                    "jerk constraints", "HLS quantization and integration", "motor execution"]}
            n *= 2
        raise ValueError("MoveL subdivision budget exhausted; no trajectory emitted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["fk", "ik", "movel"])
    parser.add_argument("--q", required=True, nargs=6, type=float, help="robot joint coordinates in radians")
    parser.add_argument("--target", nargs=6, type=float, help="x y z roll pitch yaw (m,rad)")
    args = parser.parse_args()
    runtime = MotionRuntime()
    if args.command == "fk":
        out = {"model_id": runtime.model_id, "pose_xyz_rpy": pose_values(runtime.fk(args.q)), "motion_commands_sent": 0}
    elif args.target is None:
        parser.error("--target required for ik/movel")
    elif args.command == "ik":
        out = {"q_rad": runtime.ik(args.target, args.q).tolist(), "motion_commands_sent": 0}
    else:
        out = runtime.plan_movel(args.q, args.target)
    print(json.dumps(out, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
