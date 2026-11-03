"""Read-only URDF visual transforms. No SDK, device or motor imports."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .simulation import rotation, vector


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, :3] = rotation(vector(element, 'rpy'))
        t[:3, 3] = vector(element, 'xyz')
    return t


class VisualModel:
    def __init__(self, urdf, meshes):
        self.root = ET.parse(urdf).getroot()
        self.meshes = Path(meshes)
        self.children = {}
        for j in self.root.findall('joint'):
            self.children.setdefault(j.find('parent').get('link'), []).append(j)
        materials = {m.get('name'): np.fromstring(m.find('color').get('rgba'), sep=' ')
                     for m in self.root.findall('material') if m.find('color') is not None}
        self.visuals = []
        for link in self.root.findall('link'):
            for v in link.findall('visual'):
                mesh = v.find('geometry/mesh')
                if mesh is None:
                    continue
                path = self.meshes / Path(mesh.get('filename')).name
                if not path.is_file():
                    raise FileNotFoundError(path)
                material = v.find('material')
                rgba = materials.get(material.get('name'), [.7, .7, .7, 1]) if material is not None else [.7, .7, .7, 1]
                if material is not None and material.find('color') is not None:
                    rgba = np.fromstring(material.find('color').get('rgba'), sep=' ')
                self.visuals.append(dict(link=link.get('name'), path=path,
                                         local=origin(v.find('origin')), rgba=rgba,
                                         scale=vector(mesh, 'scale', '1 1 1')))

    def poses(self, q):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('EXPECTED_SIX_FINITE_JOINT_ANGLES')
        angles = {'L%d_joint' % (i+1): a for i, a in enumerate(q)}
        poses = {'base_link': np.eye(4)}
        def visit(parent):
            for j in self.children.get(parent, []):
                t = origin(j.find('origin'))
                if j.get('type') != 'fixed':
                    a = angles.get(j.get('name'), 0.)
                    axis = np.asarray(vector(j.find('axis'), 'xyz'), float)
                    axis /= np.linalg.norm(axis)
                    x, y, z = axis
                    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    r = np.eye(4)
                    r[:3, :3] += np.sin(a)*k + (1-np.cos(a))*(k@k)
                    t = t@r
                child = j.find('child').get('link')
                poses[child] = poses[parent]@t
                visit(child)
        visit('base_link')
        return poses
