#!/usr/bin/env python3
"""Independent URDF FK, finite-difference Jacobian, IK and path checks.

Does not import a transport or access hardware. Timings are observations,
not worst-case real-time guarantees. Produces JSON and exits nonzero on failure.
"""
import argparse
import hashlib
import importlib.metadata
import json
import logging
import platform
import time
import xml.etree.ElementTree as ET

import numpy as np
from motion_runtime import ROOT, LIMITS, MotionRuntime, pose_values, residual, interpolate_pose


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)


def reference_chain():
    root = ET.parse(ROOT / "model/el_a3_legacy.urdf").getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain, link = [], "end_effector"
    while link in by_child:
        joint = by_child[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    return list(reversed(chain))


def reference_fk(chain, q):
    transform = np.eye(4)
    for joint in chain:
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
        local = np.eye(4)
        local[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
        local[:3, 3] = xyz
        transform = transform @ local
        if joint.get("type") == "revolute":
            i = int(joint.get("name").split("_")[0][1:])-1
            local = np.eye(4)
            local[:3, :3] = rotation(np.fromstring(joint.find("axis").get("xyz"), sep=" "), q[i])
            transform = transform @ local
        elif joint.get("type") != "fixed":
            raise AssertionError("unsupported reference joint")
    return transform


def timing(values):
    return {"samples": len(values), **{name: float(np.percentile(values, p))
            for name, p in [("p50_us", 50), ("p95_us", 95), ("p99_us", 99), ("max_us", 100)]}}


def validate():
    logging.getLogger("el_a3_sdk.kinematics").setLevel(logging.ERROR)
    runtime, chain = MotionRuntime(), reference_chain()
    rng = np.random.default_rng(260)
    samples = rng.uniform(LIMITS[:, 0], LIMITS[:, 1], size=(1000, 6))
    fk_errors, fk_times, gravity_times, jac_errors = [], [], [], []
    for q in samples[:10]:
        runtime.fk(q)
    for q in samples:
        begin = time.perf_counter_ns()
        fk = runtime.fk(q)
        fk_times.append((time.perf_counter_ns()-begin)/1000)
        ref = reference_fk(chain, q)
        fk_errors.append(float(np.max(np.abs(fk.homogeneous-ref))))
    assert max(fk_errors) < 1e-10, "Pinocchio/independent URDF FK disagreement"
    for q in samples[:20]:
        jac = runtime.kin.compute_jacobian(q.tolist())
        assert jac.shape == (6, 7) and np.max(np.abs(jac[:, 6])) < 1e-12
        for i in range(6):
            dq = np.zeros(6); dq[i] = 1e-6
            plus, minus = reference_fk(chain, q+dq), reference_fk(chain, q-dq)
            linear = (plus[:3, 3]-minus[:3, 3])/2e-6
            rdot = (plus[:3, :3]-minus[:3, :3])/2e-6
            skew = rdot @ reference_fk(chain, q)[:3, :3].T
            angular = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
            jac_errors.append(float(np.max(np.abs(jac[:, i]-np.r_[linear, angular]))))
    assert max(jac_errors) < 1e-7
    q0 = np.array([0.2, 1.2, -1.5, 0.2, 0.4, -0.2])
    ik_errors, ik_times, fixtures = [], [], []
    for _ in range(50):
        target_q = q0+rng.uniform(-0.05, 0.05, 6)
        target = runtime.fk(target_q)
        begin = time.perf_counter_ns()
        solved = runtime.ik(pose_values(target), q0)
        ik_times.append((time.perf_counter_ns()-begin)/1000)
        ik_errors.append(residual(runtime.fk(solved), target))
        fixtures.append({"q": target_q.tolist(), "fk": target.homogeneous.tolist(), "ik": solved.tolist()})
    for q in samples[:100]:
        begin = time.perf_counter_ns()
        gravity = runtime.kin.compute_gravity(q.tolist())
        gravity_times.append((time.perf_counter_ns()-begin)/1000)
        assert len(gravity) == 7 and np.isfinite(gravity).all()
    rejects = []
    bad_q = [[], [0]*7, [True]*6, [float("nan")]*6, [float("inf")]*6, [100]*6,
             [True, 1., -1., 0., 0., 0.], ["0"]*6]
    for i, q in enumerate(bad_q):
        try:
            runtime.fk(q)
        except ValueError:
            rejects.append("invalid_q_"+str(i))
        else:
            raise AssertionError("invalid q accepted")
    begin = time.perf_counter_ns()
    try:
        runtime.ik([10, 10, 10, 0, 0, 0], q0)
    except ValueError:
        rejects.append("unreachable_ik")
    else:
        raise AssertionError("unreachable IK accepted")
    impossible_us = (time.perf_counter_ns()-begin)/1000
    target = runtime.fk(q0+np.array([0.12, 0.06, -0.08, 0.04, 0.08, -0.04]))
    target_values = pose_values(target)
    q1 = runtime.ik(target_values, q0)
    delta = q1-q0
    coarse = np.column_stack((q0, np.zeros(6), 3*delta, -2*delta))
    coarse_audit = runtime.audit_segment(coarse, pose_values(runtime.fk(q0)), target_values, 257)
    assert coarse_audit["max_position_error_m"] > 0.0001, "fixture must expose endpoint-only validation"
    # Endpoint-valid polynomial whose interior violates a joint limit.
    overshoot = np.column_stack((q0, np.zeros((6, 3))))
    overshoot[0, 1:3] = [20, -20]
    try:
        runtime.audit_segment(overshoot, pose_values(runtime.fk(q0)), target_values)
    except ValueError:
        rejects.append("interior_joint_limit")
    else:
        raise AssertionError("interior limit crossing accepted")
    try:
        runtime.plan_movel(q0, target_values, max_segments=1)
    except ValueError:
        rejects.append("insufficient_subdivision_budget")
    else:
        raise AssertionError("coarse segment unexpectedly accepted")
    begin = time.perf_counter_ns()
    plan = runtime.plan_movel(q0, target_values)
    planning_us = (time.perf_counter_ns()-begin)/1000
    dense_errors, max_v, max_a = [], 0., 0.
    start = runtime.fk(q0)
    count = len(plan["segments"])
    assert count > 1
    for i, segment in enumerate(plan["segments"]):
        c, duration = np.array(segment["coefficients_rad"]), segment["duration_s"]
        for u in np.linspace(0, 1, 257):
            q = np.polynomial.polynomial.polyval(u, c.T)
            ref = reference_fk(chain, q)
            desired = interpolate_pose(start, target, (i+u*u*(3-2*u))/count)
            pos = float(np.linalg.norm(ref[:3, 3]-desired.translation))
            # Rotation chord norm gives a stable angle, independent of log3.
            angle = float(2*np.arcsin(np.clip(np.linalg.norm(ref[:3, :3]-desired.rotation)/np.sqrt(8), 0, 1)))
            dense_errors.append((pos, angle))
            max_v = max(max_v, float(np.max(np.abs(c[:, 1]+2*c[:, 2]*u+3*c[:, 3]*u*u)))/duration)
            max_a = max(max_a, float(np.max(np.abs(2*c[:, 2]+6*c[:, 3]*u)))/duration**2)
    assert max(e[0] for e in dense_errors) <= plan["position_tolerance_m"]
    assert max(e[1] for e in dense_errors) <= plan["orientation_tolerance_rad"]
    assert max_v <= 0.2+1e-12 and max_a <= 0.5+1e-12
    return {"status": "PASS", "machine": platform.machine(), "python": platform.python_version(),
            "packages": {p: importlib.metadata.version(p) for p in ["pin", "numpy", "PyYAML"]},
            "model_id": runtime.model_id, "motion_commands_sent": 0, "enable_commands_sent": 0,
            "fk": {"independent_samples": 1000, "max_matrix_error": max(fk_errors), "timing": timing(fk_times)},
            "jacobian": {"finite_difference_axes": len(jac_errors), "max_error": max(jac_errors)},
            "ik": {"reachable_nearby_cases": 50, "accepted": len(ik_errors), "timing": timing(ik_times),
                   "max_position_error_m": max(e[0] for e in ik_errors), "max_orientation_error_rad": max(e[1] for e in ik_errors),
                   "unreachable_rejected_time_us": impossible_us, "workspace_coverage_verified": False},
            "gravity": {"finite_output_cases": 100, "timing": timing(gravity_times), "physical_compensation_verified": False},
            "rejections": rejects,
            "movel": {"coarse_segment": coarse_audit, "refined_segments": count, "planning_us": planning_us,
                      "dense_samples": len(dense_errors), "max_position_error_m": max(e[0] for e in dense_errors),
                      "max_orientation_error_rad": max(e[1] for e in dense_errors),
                      "max_velocity_rad_s": max_v, "max_acceleration_rad_s2": max_a,
                      "continuous_error_bound_verified": False, "executable": False},
            "fixtures": fixtures, "dry_run_plan": plan}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = validate()
    with open(args.output, "w") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps({k: v for k, v in result.items() if k not in ("fixtures", "dry_run_plan")}, indent=2))
