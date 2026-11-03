"""Digital twin geometry for the console: forward kinematics of the arm from measured joint angles, read from the robot's URDF.

Pure numpy and xml.etree (no pinocchio): the console process only needs the joint origins and the tool point, not dynamics. The joint
angles are the same joint-side values the board's freedrive reports (motor value minus offset, divided by direction).
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rotation(axis, angle):
    a = np.asarray(axis, float);a = a / np.linalg.norm(a)
    x, y, z = a;c, s = math.cos(angle), math.sin(angle);t = 1 - c
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def find_urdf(app):
    app = Path(app).resolve()
    for candidate in (app.parents[1] / 'fpga_competition_2026/contest/arm_runtime/model/el_a3_legacy.urdf',
                      app.parent / 'el_a3_sdk/resources/urdf/el_a3_legacy.urdf'):
        if candidate.exists():
            return candidate
    raise FileNotFoundError('el_a3_legacy.urdf')


class Twin:
    def __init__(self, urdf_path, end='end_effector'):
        joints = {}
        for j in ET.parse(urdf_path).getroot().iter('joint'):
            if j.find('parent') is None or j.find('child') is None:
                continue                      # <transmission><joint .../> entries
            origin = j.find('origin');axis = j.find('axis')
            joints[j.find('child').get('link')] = dict(
                name=j.get('name'), kind=j.get('type'), parent=j.find('parent').get('link'),
                xyz=[float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()],
                rpy=[float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()],
                axis=[float(v) for v in axis.get('xyz').split()] if axis is not None else [0, 0, 1])
        chain, link = [], end
        while link in joints:
            chain.append(joints[link]);link = joints[link]['parent']
        self.chain = chain[::-1]
        self.revolute = [j['name'] for j in self.chain if j['kind'] in ('revolute', 'continuous')]

    def points(self, q):
        """Positions (m, base frame) of every joint origin along the chain and the tool point last; q lists the revolute joints in order."""
        T = np.eye(4);pts = [T[:3, 3].tolist()];index = 0
        for j in self.chain:
            local = np.eye(4);local[:3, :3] = _rpy(*j['rpy']);local[:3, 3] = j['xyz']
            if j['kind'] in ('revolute', 'continuous'):
                angle = float(q[index]) if index < len(q) else 0.;index += 1
                rot = np.eye(4);rot[:3, :3] = _axis_rotation(j['axis'], angle);local = local @ rot
            T = T @ local;pts.append(T[:3, 3].tolist())
        return pts

    def tool(self, q):
        return self.points(q)[-1]
