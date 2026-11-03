"""Native Qt competition workbench. Default disconnected; physical observer only.

Qt runs in the existing SDK environment, MuJoCo and ROCm in their own workers.
The UI has no direct CAN/enable/reset path. Simulation uses the task protocol.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from PyQt6.QtCore import Qt,QTimer,QProcess,QProcessEnvironment,QPointF,pyqtSignal
from PyQt6.QtGui import QColor,QPainter,QPen,QFont,QLinearGradient,QPixmap,QPolygonF
from PyQt6.QtWidgets import (QApplication,QMainWindow,QWidget,QLabel,QPushButton,QLineEdit,
    QVBoxLayout,QHBoxLayout,QGridLayout,QSplitter,QFrame,QPlainTextEdit,QComboBox,QFileDialog)

from .live_controls import LiveControls

BG='#0c111a';PANEL='#131c29';MUTED='#91a5bb';ACCENT='#ff7257';CYAN='#54d6c6'

def describe_intent(value):
    if value.get('version')=='workcell.inventory_goal.v1':
        if value['action']=='clarify':return value['question']
        return f"目标：{value['tray']}盒最终恰好{value['quantity']}颗螺丝；多退少补，螺母不动。"
    names={'screw':'螺丝','nut':'螺母'};regions={'all':'','left':'左侧','right':'右侧'}
    if value['action']=='clarify':return value['question']
    if value['action'] in ('pause','resume'):return '暂停当前任务' if value['action']=='pause' else '恢复当前任务'
    lines=[]
    for i,s in enumerate(value['steps']):
        quantity='全部' if s['quantity']=='all' else str(s['quantity'])+'颗'
        lines.append(f"{i+1:02d}  {regions[s['region']]}{quantity}{names[s['part_class']]} → {s['destination']}盒")
    if value['exclude']:lines.append('保留原处：'+'、'.join(names[x] for x in value['exclude']))
    return '\n\n'.join(lines)

class RobotView(QWidget):
    def __init__(self):super().__init__();self.frame={};self.setMinimumSize(480,330)
    def set_frame(self,f):self.frame=f;self.update()
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing)
        g=QLinearGradient(0,0,self.width(),self.height());g.setColorAt(0,QColor('#182536'));g.setColorAt(1,QColor('#0c1521'));p.fillRect(self.rect(),g)
        def project(xyz):
            x,y,z=xyz;return QPointF(self.width()*.64+(x-y)*650,self.height()*.71+(x+y)*270-z*750)
        p.setPen(QPen(QColor('#27384b'),1))
        for k in range(-5,6):
            a=k*.05;p.drawLine(project([a,-.3,0]),project([a,.3,0]));p.drawLine(project([-.3,a,0]),project([.3,a,0]))
        for name,y in [('A',.12),('B',-.12)]:
            poly=QPolygonF([project([x,yy,.002]) for x,yy in [(-.255,y-.065),(-.105,y-.065),(-.105,y+.065),(-.255,y+.065)]])
            p.setBrush(QColor('#194b59'));p.setPen(QPen(QColor(CYAN),1.5));p.drawPolygon(poly);p.drawText(project([-.18,y,.01]),name)
        for name,xyz in self.frame.get('objects',{}).items():
            p.setPen(QPen(QColor('#e4c47e' if name.startswith('screw') else '#8aa4bb'),6,Qt.PenStyle.SolidLine,Qt.PenCapStyle.RoundCap))
            if name.startswith('screw'):p.drawLine(project([xyz[0],xyz[1]-.023,xyz[2]]),project([xyz[0],xyz[1]+.023,xyz[2]]))
            else:p.drawEllipse(project(xyz),6,6)
        points=self.frame.get('arm_points_m',[])
        for i in range(len(points)-1):
            p.setPen(QPen(QColor('#b9c9d8'),12 if i<5 else 8,Qt.PenStyle.SolidLine,Qt.PenCapStyle.RoundCap));p.drawLine(project(points[i]),project(points[i+1]))
        for xyz in points:
            q=project(xyz);p.setBrush(QColor(ACCENT));p.setPen(QPen(QColor('#ffab76'),2));p.drawEllipse(q,6,6)
        p.setPen(QColor('#b9cadb'));p.setFont(QFont('Noto Sans',10));p.drawText(18,28,'A3 / URDF KINEMATICS')
        p.setPen(QColor(CYAN));p.drawText(18,49,'SIMULATION · approximate collision geometry')
        if self.frame.get('tcp_m'):
            t=self.frame['tcp_m'];p.setPen(QColor('#e5edf6'));p.drawText(18,self.height()-18,'TCP  '+ '  '.join(f'{x*1000:+.1f}' for x in t)+' mm')

class Window(LiveControls,QMainWindow):
    status_ready=pyqtSignal(dict)
    def __init__(self,a):
        super().__init__();self.a=a;self.app_root=Path(a.app_root).resolve();self.root=self.app_root/'integration'
        self.ui_evidence=self.app_root/'reports'/('workbench_ui_'+str(time.time_ns()));self.ui_evidence.mkdir()
        self.frames=[];self.index=0;self.intent=None;self.playing=False;self.model=None;self.sim=None;self.pending_instruction=None;self.scene=None
        self.setWindowTitle('Kria Workcell · AMD Edge Robotics');self.resize(1440,940)
        self.setStyleSheet('''QWidget{background:#0c111a;color:#e0e9f4;font-family:"Noto Sans CJK SC";font-size:13px;} QFrame#card{background:#131c29;border:1px solid #233449;border-radius:12px;} QLabel{background:transparent;} QPushButton{background:#223348;border:1px solid #344b64;border-radius:7px;padding:10px 16px;} QPushButton:hover{background:#304962;} QPushButton:disabled{color:#53677c;} QPushButton#primary{background:#ff7257;color:#101923;font-weight:bold;border:none;} QLineEdit,QPlainTextEdit,QComboBox{background:#0e1723;border:1px solid #31465d;border-radius:7px;padding:9px;color:#e0e9f4;} QSplitter::handle{background:#0c111a;}''')
        shell=QWidget();outer=QVBoxLayout(shell);outer.setContentsMargins(24,18,24,18);outer.setSpacing(16);self.setCentralWidget(shell)
        header=QHBoxLayout();title=QLabel('KRIA  /  WORKCELL');title.setStyleSheet('font-size:26px;font-weight:700;letter-spacing:2px;');header.addWidget(title);header.addStretch()
        self.mode=QComboBox();self.mode.addItems(['仿真 / MuJoCo','录制回放','实物只读监控','实物分拣 / 目标数量']);header.addWidget(self.mode);outer.addLayout(header)
        badges=QHBoxLayout()
        for label,value in [('AMD AI PC','Radeon · ROCm / local inference'),('KV260','ARM planning + PL execution'),('TASK LINK','v2.2 · explicit backend'),('CALIBRATION','Empirical grasp baseline / precision unverified')]:
            f=QFrame();f.setObjectName('card');v=QVBoxLayout(f);x=QLabel(label);x.setStyleSheet('color:#91a5bb;font-size:11px');v.addWidget(x);v.addWidget(QLabel(value));badges.addWidget(f)
        outer.addLayout(badges)
        content=QSplitter();outer.addWidget(content,1)
        left=QFrame();left.setObjectName('card');l=QVBoxLayout(left);l.setContentsMargins(18,18,18,18);l.setSpacing(12)
        heading=QLabel('任务 / LANGUAGE → SKILLS');heading.setStyleSheet('font-size:16px;font-weight:bold');l.addWidget(heading)
        self.input=QLineEdit('把一颗螺丝放到B盒，螺母不要动。');l.addWidget(self.input)
        self.parse=QPushButton('理解指令 · 本地 Qwen');self.parse.setObjectName('primary');l.addWidget(self.parse)
        self.decision=QLabel('等待输入\n模型调用不会直接驱动电机。');self.decision.setWordWrap(True);l.addWidget(self.decision)
        self.run=QPushButton('通过任务协议运行仿真');l.addWidget(self.run)
        self.pause=QPushButton('暂停回放');l.addWidget(self.pause)
        buttons=QHBoxLayout();self.pause_task=QPushButton('暂停任务');self.resume_task=QPushButton('恢复任务');buttons.addWidget(self.pause_task);buttons.addWidget(self.resume_task);l.addLayout(buttons)
        self.import_btn=QPushButton('导入录制任务');l.addWidget(self.import_btn)
        self.scene_btn=QPushButton('载入 RGB-D 场景');l.addWidget(self.scene_btn)
        self.monitor=QPushButton('读取 KV260 状态');l.addWidget(self.monitor)
        self.setup_live(l)
        self.metrics=QLabel('实物抓取成功率：待标定验证\n当前模式不发送实物运动命令');self.metrics.setWordWrap(True);self.metrics.setStyleSheet('color:#91a5bb');l.addWidget(self.metrics);l.addStretch()
        content.addWidget(left)
        middle=QWidget();ml=QVBoxLayout(middle);ml.setContentsMargins(10,0,10,0)
        mesh_error=None
        try:
            if os.environ.get('QT_QPA_PLATFORM')=='offscreen':raise RuntimeError('Offscreen test uses schematic renderer')
            from .mesh_view import MeshView
            self.view=MeshView(self.root/'workcell/assets/el_a3_legacy.urdf',self.app_root/'assets/a3_legacy_meshes')
        except (ImportError,FileNotFoundError,RuntimeError) as exc:
            mesh_error=str(exc);self.view=RobotView()
        ml.addWidget(self.view,3)
        self.image_caption=QLabel('录制图像 / 静态关键帧');ml.addWidget(self.image_caption)
        self.image=QLabel('RGB-D / mask / grasp-axis replay');self.image.setAlignment(Qt.AlignmentFlag.AlignCenter);self.image.setMinimumHeight(185);self.image.setStyleSheet('background:#131c29;border-radius:10px;color:#91a5bb');ml.addWidget(self.image,2);content.addWidget(middle)
        right=QFrame();right.setObjectName('card');rl=QVBoxLayout(right);rl.setContentsMargins(16,16,16,16)
        self.phase=QLabel('IDLE');self.phase.setStyleSheet('font-size:23px;color:#54d6c6;font-weight:bold');rl.addWidget(self.phase)
        self.joints=QLabel('关节反馈等待载入');self.joints.setStyleSheet('font-family:monospace;line-height:1.7');rl.addWidget(self.joints)
        rl.addWidget(QLabel('执行事件 / PROVENANCE'));self.log=QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumBlockCount(600);rl.addWidget(self.log,1);content.addWidget(right);content.setSizes([330,740,300])
        self.footer=QLabel('DISCONNECTED   ·   Simulation and replay are labelled; no physical execution enabled.');self.footer.setStyleSheet('color:#91a5bb');outer.addWidget(self.footer)
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(50)
        self.pause.clicked.connect(self.toggle);self.parse.clicked.connect(self.infer);self.run.clicked.connect(self.run_simulation);self.import_btn.clicked.connect(self.choose);self.monitor.clicked.connect(self.read_status)
        self.pause_task.clicked.connect(lambda:self.command('PAUSE'));self.resume_task.clicked.connect(lambda:self.command('RESUME'));self.status_ready.connect(self.status_received)
        self.input.textChanged.connect(self.invalidate)
        self.scene_btn.clicked.connect(self.choose_scene)
        if mesh_error:self.append('原厂mesh未加载，显示连杆示意：'+mesh_error)
        if a.replay:self.load(Path(a.replay))
        if a.scene:self.load_scene(Path(a.scene))
        self.run.setEnabled(False)
        self.mode.currentIndexChanged.connect(self.live_mode_changed)
        self.live_mode_changed()
        if getattr(a,'live',False):
            self.mode.setCurrentIndex(3);self.input.setText('让B盒里恰好有三颗螺丝，多的拿回去，少的补过来。');self.footer.setText('右盘=A · 左盘=B | 相机与模型常驻；实物动作需点击一键抓取；失能逐次确认')
        if getattr(a,'prewarm_model',False):QTimer.singleShot(100,self.start_model)
    def invalidate(self):
        self.intent=None;self.run.setEnabled(False)
        if hasattr(self,"live_command_valid"):self.invalidate_live_intent()
    def append(self,value):
        text=value if isinstance(value,str) else json.dumps(value,ensure_ascii=False)
        self.log.appendPlainText(text)
        with (self.ui_evidence/'events.jsonl').open('a') as log:log.write(json.dumps(dict(monotonic_s=time.monotonic(),value=value),ensure_ascii=False)+'\n')
    def show_frame(self,f):
        self.view.set_frame(f);self.phase.setText(f.get('phase','IDLE'));self.joints.setText('\n'.join(f'J{i+1}   {q*180/math.pi:+8.2f}°' for i,q in enumerate(f.get('q_rad',[]))))
    def load(self,path):
        if path.is_file():path=path.parent
        self.frames=json.loads((path/'frames.json').read_text())
        if not self.frames or self.frames[0].get('schema')!='workcell.physics-frame.v2':
            self.frames=[];self.playing=False;self.phase.setText('旧回放需重录');self.append('该旧回放缺少统一TCP、夹爪和场景几何，已停止播放，请运行新版仿真。');return
        self.index=0;self.playing=True;self.show_frame(self.frames[0]);self.append('载入回放：'+str(path));self.image_caption.setText('MuJoCo 初始关键帧（静态）；上方为动态回放')
        # Use the entire view for simulation. A permanently open gripper in a
        # second static rendering looked like a second, broken live view.
        self.image.hide();self.image_caption.hide()
        report=path/'report.json'
        if report.exists():
            try:
                r=json.loads(report.read_text());self.metrics.setText('仿真结果：'+r['status']+'\n实物抓取指标：WAITING');self.append(r['status'])
            except ValueError:pass
    def tick(self):
        if self.playing and self.frames:self.show_frame(self.frames[self.index]);self.index=(self.index+1)%len(self.frames)
    def toggle(self):self.playing=not self.playing;self.pause.setText('暂停回放' if self.playing else '恢复回放')
    def choose(self):
        path=QFileDialog.getExistingDirectory(self,'选择含 frames.json 的录制目录')
        if path:
            try:self.load(Path(path))
            except Exception as e:self.append(str(e))
    def choose_scene(self):
        path,_=QFileDialog.getOpenFileName(self,'选择录制场景 JSON',str(self.app_root/'reports'),'JSON (*.json)')
        if path:
            try:self.load_scene(Path(path))
            except Exception as e:self.append(str(e))
    def load_scene(self,path):
        import hashlib
        scene=json.loads(path.read_text());image=Path(scene['image_file'])
        if hashlib.sha256(image.read_bytes()).hexdigest()!=scene['image_sha256']:raise ValueError('RECORDED_IMAGE_HASH_MISMATCH')
        if scene['source_kind']!='recorded':raise ValueError('EXPECTED_RECORDED_SCENE')
        self.scene=scene;self.invalidate()
        self.image.show();self.image_caption.show()
        self.image_caption.setText('录制RGB-D场景（静态图像）')
        preview=path.with_suffix('.png');preview=preview if preview.exists() else image
        self.image.setPixmap(QPixmap(str(preview)).scaled(680,270,Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))
        self.append('RGB-D录制场景 '+scene['scene_id']+' / '+str(len(scene['objects']))+'个检测；基座标定等待。')
    def infer(self):
        from .tasks import control_intent
        control=control_intent(self.input.text())
        if control is not None and self.mode.currentIndex()==3:
            self.invalidate();self.decision.setText('当前实物接口尚未接入语言暂停/恢复；该命令没有发送。');return
        if control is not None:
            self.invalidate();self.decision.setText(describe_intent(control));self.append(dict(source='OPERATOR_CONTROL_NO_MODEL',intent=control));self.command(control['action'].upper());return
        self.invalidate_live_intent()
        self.pending_instruction=self.input.text();self.parse.setEnabled(False);self.run.setEnabled(False)
        if self.model is None:self.start_model()
        elif getattr(self,'model_ready',False):self.send_instruction()
    def start_model(self):
        if self.model is None:
            self.model_ready=False
            self.model=QProcess(self);self.model.readyReadStandardOutput.connect(self.model_output);self.model.readyReadStandardError.connect(lambda:self.append(bytes(self.model.readAllStandardError()).decode(errors='replace')[-800:]))
            self.model_buffer=b'';self.model.finished.connect(self.model_stopped)
            args=['gpu','env','PYTHONPATH='+str(self.app_root/'.task-tune-deps')+':'+str(self.root),str(self.app_root/'.venv-ai-labs/bin/python'),'-u','-m','workcell.model_server','--app-root',str(self.app_root)]
            config=self.app_root/'integration/config/workcell_model.json'
            adapter=self.app_root/json.loads(config.read_text())['adapter'] if config.exists() else self.app_root/'reports/task_lora_20261020/adapter'
            if adapter.exists():args+=['--adapter',str(adapter)]
            self.model.setWorkingDirectory(str(self.root));self.model.start(str(Path.home()/'.local/share/aipc-dev/aipc.sh'),args);self.decision.setText('正在加载本地 Qwen…')
    def send_instruction(self):
        if self.pending_instruction is None:return
        request=dict(instruction=self.pending_instruction,context={})
        if self.mode.currentIndex()==3 and self.goal_mode.currentIndex()==1:request['contract']='inventory_goal'
        if self.scene is not None and self.mode.currentIndex()==1:request.update(scene=self.scene,image_file=self.scene['image_file'])
        self.model.write((json.dumps(request,ensure_ascii=False)+'\n').encode());self.pending_instruction=None
    def model_output(self):
        self.model_buffer+=bytes(self.model.readAllStandardOutput())
        while b'\n' in self.model_buffer:
            line,self.model_buffer=self.model_buffer.split(b'\n',1)
            try:v=json.loads(line)
            except ValueError:self.append(line.decode(errors='replace'));continue
            self.append(v)
            if v.get('event')=='MODEL_READY':
                self.model_ready=True;self.decision.setText('千问已就绪，后续指令复用当前模型');self.send_instruction()
            elif v.get('event')=='MODEL_RESULT':
                if v.get('instruction')!=self.input.text():
                    self.invalidate();self.parse.setEnabled(True);self.append('输入已改变，旧命令结果已丢弃，请重新理解。');continue
                value=v['intent'];self.decision.setText(describe_intent(value));self.parse.setEnabled(True)
                if value.get('version')=='workcell.inventory_goal.v1':
                    if self.mode.currentIndex()==3 and self.goal_mode.currentIndex()==1 and value['action']=='reconcile':self.apply_goal_intent(value)
                    continue
                if value['action']=='plan':
                    self.intent=value;self.run.setEnabled(self.mode.currentIndex()<2)
                    if self.mode.currentIndex()==3:self.apply_live_intent(value)
                elif value['action'] in ('pause','resume'):self.command(value['action'].upper())
            elif v.get('event')=='MODEL_ERROR':self.decision.setText(v['error']);self.parse.setEnabled(True)
    def model_stopped(self):self.model=None;self.parse.setEnabled(True)
    def command(self,operation):
        if self.sim and self.sim.state()==QProcess.ProcessState.Running:
            self.sim.write((json.dumps(dict(command=operation))+'\n').encode());self.append('请求仿真任务 '+operation)
        else:self.append('没有正在运行的仿真任务。')
    def run_simulation(self):
        if self.mode.currentIndex()>=2:self.append('实物模式当前仅提供只读监控；视觉运动等待真实标定。');return
        if self.sim and self.sim.state()!=QProcess.ProcessState.NotRunning:return
        output=self.app_root/'reports'/('workbench_'+str(time.time_ns()));self.sim_output_dir=output;self.run.setEnabled(False);self.sim=QProcess(self)
        env=QProcessEnvironment.systemEnvironment();env.insert('MUJOCO_GL','egl');env.insert('PYOPENGL_PLATFORM','egl');self.sim.setProcessEnvironment(env);self.sim.setWorkingDirectory(str(self.root));self.sim_buffer=b''
        self.sim.readyReadStandardOutput.connect(self.sim_output);self.sim.readyReadStandardError.connect(lambda:self.append(bytes(self.sim.readAllStandardError()).decode(errors='replace')[-1000:]))
        self.sim.finished.connect(lambda:self.run.setEnabled(True));args=['sim','python','-u','-m','workcell.demo','--urdf',str(self.root/'workcell/assets/el_a3_legacy.urdf'),'--output',str(output)]
        if self.intent is not None:
            plan=self.app_root/'reports'/'workbench_task.json';plan.write_text(json.dumps(self.intent,ensure_ascii=False));args+=['--plan',str(plan)]
        self.sim.start(str(Path.home()/'.local/share/aipc-dev/aipc.sh'),args)
    def sim_output(self):
        self.sim_buffer+=bytes(self.sim.readAllStandardOutput())
        while b'\n' in self.sim_buffer:
            line,self.sim_buffer=self.sim_buffer.split(b'\n',1)
            try:v=json.loads(line)
            except ValueError:self.append(line.decode(errors='replace'));continue
            if v.get('snapshot'):self.playing=False;self.show_frame(v['snapshot'])
            else:self.append(v)
            if v.get('replay_directory'):self.load(Path(v['replay_directory']))
    def read_status(self):
        if self.mode.currentIndex()==3:
            self.monitor.setEnabled(False);self.workflow_request(dict(op='board_status'));return
        import threading
        self.monitor.setEnabled(False)
        def job():
            import asyncio
            from screwcell.client import Client
            async def read():
                c=Client('192.168.26.2',client_id='workbench_observer',role='observer')
                try:await asyncio.wait_for(c.connect(),8);return await c.call('STATUS')
                finally:await c.close()
            try:value=asyncio.run(read())
            except Exception as e:value=dict(error=str(e))
            self.status_ready.emit(value)
        threading.Thread(target=job,daemon=True).start()
    def status_received(self,value):self.append(value);self.monitor.setEnabled(True)
    def closeEvent(self,event):
        if self.live_owner or self.live_busy:
            self.append('当前任务尚未结束，请等待或完成本轮承托失能；关闭窗口不会停止机械臂。');event.ignore();return
        if self.workflow and self.workflow.state()!=QProcess.ProcessState.NotRunning:
            self.workflow.write(b'{"op":"shutdown"}\n');self.workflow.closeWriteChannel();self.workflow.waitForFinished(3000)
        if self.model and self.model.state()!=QProcess.ProcessState.NotRunning:
            self.model.write(b'{"op":"shutdown"}\n');self.model.closeWriteChannel()
            if not self.model.waitForFinished(3000):self.model.terminate();self.model.waitForFinished(3000)
        if self.sim and self.sim.state()!=QProcess.ProcessState.NotRunning:self.sim.terminate();self.sim.waitForFinished(3000)
        event.accept()

def main():
    p=argparse.ArgumentParser();p.add_argument('--app-root',required=True);p.add_argument('--replay');p.add_argument('--scene');p.add_argument('--live',action='store_true');p.add_argument('--prewarm-model',action='store_true');p.add_argument('--self-test-output');p.add_argument('--integration-test',action='store_true');a=p.parse_args()
    app=QApplication(sys.argv[:1]);w=Window(a);w.show()
    if a.self_test_output:
        def capture():
            out=Path(a.self_test_output);out.mkdir(parents=True,exist_ok=True);original_playing=w.playing;w.toggle();assert w.playing != original_playing;w.toggle();assert w.playing == original_playing
            evidence=dict(status='PASS_QT_OFFSCREEN_RENDER_AND_REPLAY_CONTROLS',frames_loaded=len(w.frames),motor_commands_sent=0)
            if a.integration_test:
                report=json.loads((w.sim_output_dir/'report.json').read_text());assert report['verified_simulated']==report['requested']==1
                evidence.update(status='PASS_QT_MODEL_PROCESS_AND_TCP_CONTACT_SIMULATION',simulation_directory=str(w.sim_output_dir),model_intent=w.intent)
            w.grab().save(str(out/'workbench.png'));(out/'ui_test.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n');(out/'events.txt').write_text(w.log.toPlainText())
            w.close();app.quit()
        if a.integration_test:
            test_started=time.monotonic();started_sim=False
            def progress():
                nonlocal started_sim
                if time.monotonic()-test_started>180:
                    out=Path(a.self_test_output);out.mkdir(parents=True,exist_ok=True);(out/'failure.txt').write_text(w.log.toPlainText());w.close();app.exit(1);return
                if w.intent is not None and not started_sim:started_sim=True;w.run.click()
                if started_sim and w.sim.state()==QProcess.ProcessState.NotRunning:capture();return
                QTimer.singleShot(100,progress)
            QTimer.singleShot(100,w.infer);QTimer.singleShot(200,progress)
        else:QTimer.singleShot(600,capture)
    sys.exit(app.exec())

if __name__=='__main__':main()
