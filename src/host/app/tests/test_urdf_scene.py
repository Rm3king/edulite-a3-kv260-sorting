import tempfile
import unittest
from pathlib import Path
import numpy as np
from workcell.urdf_scene import VisualModel


class VisualTransformTests(unittest.TestCase):
    def test_visual_origin_and_joint_axis_are_composed(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'a.stl').write_bytes(b'not loaded by geometry test')
            (p/'a.urdf').write_text('''<robot name="test"><link name="base_link"/><link name="tool"><visual><origin xyz="1 0 0"/><geometry><mesh filename="package://x/a.stl"/></geometry></visual></link><joint name="L1_joint" type="revolute"><parent link="base_link"/><child link="tool"/><origin xyz="0 0 2"/><axis xyz="0 0 1"/></joint></robot>''')
            model=VisualModel(p/'a.urdf',p)
            t=model.poses([np.pi/2,0,0,0,0,0])['tool']@model.visuals[0]['local']
            np.testing.assert_allclose(t[:3,3],[0,1,2],atol=1e-12)
    def test_missing_mesh_is_explicit(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'a.urdf').write_text('<robot><link name="base_link"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>')
            with self.assertRaises(FileNotFoundError):VisualModel(p/'a.urdf',p)
    def test_bad_joint_vector_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'a.urdf').write_text('<robot><link name="base_link"/></robot>')
            with self.assertRaises(ValueError):VisualModel(p/'a.urdf',p).poses([0]*7)
