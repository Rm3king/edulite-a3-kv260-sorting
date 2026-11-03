"""Vendor mesh viewer; only accepts recorded joint snapshots."""
import math
import time
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from .urdf_scene import VisualModel
from .tool_geometry import visual_transform,grasp_transform


class MeshView(QWidget):
    def __init__(self, urdf, meshes):
        super().__init__()
        self.setMinimumSize(480, 330)
        self.model = VisualModel(urdf, meshes)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel('A3 原厂零件 mesh · legacy URDF · 鼠标旋转/缩放')
        self.label.setStyleSheet('color:#54d6c6;padding:6px')
        layout.addWidget(self.label)
        self.plot = QtInteractor(self)
        layout.addWidget(self.plot.interactor, 1)
        self.plot.set_background('#131c29')
        self.plot.enable_trackball_style()
        self.actors = []
        for v in self.model.visuals:
            mesh = pv.read(v['path'])
            mesh.scale(v['scale'], inplace=True)
            actor = self.plot.add_mesh(mesh, color=v['rgba'][:3], smooth_shading=True,
                                       ambient=.25, diffuse=.75, specular=.25, reset_camera=False)
            self.actors.append((v, actor))
        self.physics_actors={}
        self.tcp_actor=self.plot.add_mesh(pv.Sphere(radius=.003),color='#20ff78')
        self.end_actor=self.plot.add_mesh(pv.Sphere(radius=.002),color='#ff7257')
        self.plot.add_axes()
        self.plot.camera_position = [(-.70, -.75, 1.15), (-.10, 0, .12), (0, 0, 1)]
        self.last_q = None
        self.last_render = 0.
        self.set_frame(dict(q_rad=[0]*6,jaw_slide_m=dict(left=0.,right=0.)))

    def physics_geom(self,g):
        key=g['name']
        if key not in self.physics_actors:
            size=g['size'];kind=g['type']
            if kind==0:mesh=pv.Plane(i_size=2*size[0],j_size=2*size[1])
            elif kind==2:mesh=pv.Sphere(radius=size[0])
            elif kind==3:
                mesh=pv.Cylinder(radius=size[0],height=2*size[1],direction=(0,0,1))
                mesh=mesh.merge(pv.Sphere(radius=size[0],center=(0,0,size[1]))).merge(pv.Sphere(radius=size[0],center=(0,0,-size[1])))
            elif kind==5:mesh=pv.Cylinder(radius=size[0],height=2*size[1],direction=(0,0,1))
            elif kind==6:mesh=pv.Box(bounds=(-size[0],size[0],-size[1],size[1],-size[2],size[2]))
            else:raise ValueError('UNSUPPORTED_PHYSICS_DISPLAY_GEOM '+str(kind))
            opacity=.30 if key.startswith('wall_') else g['rgba'][3]
            self.physics_actors[key]=self.plot.add_mesh(mesh,color=g['rgba'][:3],opacity=opacity,reset_camera=False)
        self.physics_actors[key].user_matrix=np.asarray(g['pose'])

    def set_frame(self, frame):
        if time.monotonic()-self.last_render<.1:return
        q = frame.get('q_rad')
        if q is None:
            return
        poses = self.model.poses(q)
        slides=frame.get('jaw_slide_m',dict(left=0.,right=0.))
        for visual, actor in self.actors:
            actor.user_matrix = visual_transform(visual,poses,slides)
        for g in frame.get('environment_geoms',[])+frame.get('object_geoms',[]):self.physics_geom(g)
        tcp_pose=poses['end_effector']@grasp_transform()
        self.tcp_actor.user_matrix=tcp_pose;self.end_actor.user_matrix=poses['end_effector']
        tcp = frame.get('tcp_m')
        suffix = '' if tcp is None else ' · TCP '+', '.join('%+.1f' % (x*1000) for x in tcp)+' mm'
        width=frame.get('jaw_width_m')
        self.label.setText('绿色=抓取TCP / 橙色=SDK末端'+(' · 开度 %.1f mm'%(width*1000) if width is not None else '')+suffix)
        self.plot.render()
        self.last_render=time.monotonic()

    def closeEvent(self, event):
        self.plot.close()
        super().closeEvent(event)
