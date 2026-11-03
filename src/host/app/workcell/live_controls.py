"""Qt integration for the fixed live workflow. Startup has no motor action."""
import json
import time
from pathlib import Path
from PyQt6.QtCore import QProcess, QTimer
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QPushButton, QSpinBox, QHBoxLayout, QMessageBox, QComboBox


class LiveControls:
    def setup_live(self, layout):
        self.workflow = None
        self.live_goal=None
        self.live_task_goal=None
        self.workflow_buffer = b''
        self.live_directory = None
        self.live_owner = False
        self.live_busy = False
        self.live_auto_execute = False
        self.live_poll_pending = False
        self.live_seen = set()
        self.live_command_valid = True
        self.applying_live_intent = False
        pointer=self.app_root/'reports/live_active_task.json'
        if pointer.exists():
            saved=json.loads(pointer.read_text());self.live_directory=saved['directory'];self.live_owner=True
            prepared=json.loads((Path(self.live_directory)/'prepared.json').read_text());self.live_task_goal=prepared.get('goal')
        self.goal_mode=QComboBox()
        self.goal_mode.addItems(['搬运数量 · 右盘到左盘','最终数量 · 两盘多退少补'])
        self.goal_mode.setCurrentIndex(1 if getattr(self.a,'live',False) else 0)
        layout.addWidget(self.goal_mode)
        self.goal_preview=QPushButton('查看两盘数量与补退计划 · 不运动')
        layout.addWidget(self.goal_preview)
        self.goal_preview.clicked.connect(self.preview_goal)
        self.goal_mode.currentIndexChanged.connect(self.goal_mode_changed)
        row = QHBoxLayout()
        row.addWidget(QLabel('螺丝'))
        self.screw_count = QSpinBox(); self.screw_count.setRange(0, 5); self.screw_count.setValue(1)
        row.addWidget(self.screw_count); row.addWidget(QLabel('螺母'))
        self.nut_count = QSpinBox(); self.nut_count.setRange(0, 5)
        row.addWidget(self.nut_count); layout.addLayout(row)
        self.screw_count.valueChanged.connect(self.manual_live_counts)
        self.nut_count.valueChanged.connect(self.manual_live_counts)
        self.live_prepare = QPushButton('准备实物路径 · 不运动')
        self.live_start = QPushButton('一键抓取 · 右盘 → 左盘（手已移开）')
        self.live_disable = QPushButton('已承托，申请结束本轮使能')
        self.live_disable.setEnabled(False)
        self.live_timing = QLabel('等待准备；盒壁间隙由现场摆放保证')
        self.live_timing.setWordWrap(True)
        for widget in (self.live_prepare, self.live_start, self.live_disable, self.live_timing):
            layout.addWidget(widget)
        self.live_prepare.clicked.connect(lambda: self.prepare_live(False))
        self.live_start.clicked.connect(lambda: self.prepare_live(True))
        self.live_disable.clicked.connect(self.disable_live)
        self.live_timer = QTimer(self)
        self.live_timer.timeout.connect(self.update_live)
        self.live_timer.start(1000)
        self.live_buttons()

    def workflow_request(self, value):
        if self.workflow is None:
            self.workflow = QProcess(self)
            self.workflow.setWorkingDirectory(str(self.root))
            self.workflow.readyReadStandardOutput.connect(self.workflow_output)
            self.workflow.readyReadStandardError.connect(lambda: self.append(bytes(self.workflow.readAllStandardError()).decode(errors='replace')[-1200:]))
            self.workflow.finished.connect(self.workflow_stopped)
            cfg = json.loads((self.root/'config/live_workflow.json').read_text())
            paths = ':'.join([str(self.root), cfg['runtime_root'], cfg['can_root']])
            args = ['sdk', 'env', 'PYTHONPATH='+paths, 'python', '-u', '-m', 'workcell.live_session', '--app-root', str(self.app_root), '--serve']
            self.workflow.start(str(Path.home()/'.local/share/aipc-dev/aipc.sh'), args)
            if not self.workflow.waitForStarted(2000):
                raise RuntimeError('任务进程未启动')
        value['request_id'] = str(time.time_ns())
        self.workflow.write((json.dumps(value, ensure_ascii=False)+'\n').encode())

    def prepare_live(self, execute):
        if self.live_owner or self.live_busy or not self.live_command_valid:
            return
        if self.goal_mode.currentIndex()==1 and self.live_goal is None:
            self.append('请先理解最终数量目标');return
        if self.live_goal is None and not 1 <= self.screw_count.value()+self.nut_count.value() <= 5:
            self.append('请选择合计 1～5 件零件'); return
        self.live_task_goal=self.live_goal
        self.live_busy = True; self.live_auto_execute = execute
        self.live_prepare.setEnabled(False); self.live_start.setEnabled(False)
        self.workflow_request(dict(op='prepare', screws=self.screw_count.value(), nuts=self.nut_count.value(),goal=self.live_goal))

    def workflow_output(self):
        self.workflow_buffer += bytes(self.workflow.readAllStandardOutput())
        while b'\n' in self.workflow_buffer:
            line, self.workflow_buffer = self.workflow_buffer.split(b'\n', 1)
            try:
                value = json.loads(line)
            except ValueError:
                self.append(line.decode(errors='replace')); continue
            event = value.get('event')
            if event != 'WORKFLOW_RESULT' or value.get('op') != 'poll':
                self.append(value)
            if event == 'RUN_DIRECTORY':
                self.live_directory = value['directory']; self.live_seen.clear()
            elif event == 'GOAL_DECISION':
                from .inventory_goal import describe
                self.decision.setText(describe(value['decision']))
                if value['decision']['status']!='NEEDS_TRANSFER':
                    self.live_busy=False;self.live_auto_execute=False;self.live_buttons()
            elif event == 'STAGE_START':
                self.phase.setText(value['stage'])
            elif event == 'STAGE_END':
                self.live_timing.setText(f"{value['stage']}：{value['seconds']:.2f} 秒")
            elif event == 'PREPARED':
                self.live_directory = value['directory']
                self.live_timing.setText(f"本轮准备：{value['prepared_total_s']:.2f} 秒")
                if self.live_auto_execute:
                    self.workflow_request(dict(op='execute', directory=self.live_directory, attended=True))
                else:
                    self.live_busy = False; self.live_buttons()
            elif event == 'STARTED':
                self.live_owner = True; self.live_busy = False
                self.phase.setText('实物执行中'); self.live_buttons()
            elif event == 'WORKFLOW_RESULT' and value.get('op') == 'poll':
                self.live_poll_pending = False
                state = value['result']
                wait = state.get('operator_wait')
                if wait:
                    self.phase.setText('等待本轮承托确认')
                    self.live_disable.setEnabled(state.get('alive', False))
                # The board writes its final summary just before process exit.
                # Keep polling until exit; verify_goal intentionally requires
                # the owner to have finished, not merely produced a summary.
                if state.get('summary') and not state.get('alive'):
                    self.append(state['summary']); self.live_owner = False
                    self.live_disable.setEnabled(False); self.live_buttons()
                    self.phase.setText('本轮结束，请核对实际搬运结果')
                    if self.live_task_goal is not None:
                        self.live_busy=True;self.live_buttons();self.workflow_request(dict(op='verify_goal',directory=self.live_directory))
                elif not state['alive']:
                    self.live_owner = False; self.live_buttons()
                    self.phase.setText('进程已退出，状态待核对')
                tail = state.get('log_tail', '')
                if tail not in self.live_seen:
                    self.live_seen = {tail}; self.append(tail)
            elif event == 'WORKFLOW_ERROR':
                self.live_poll_pending = False; self.live_busy = False; self.live_auto_execute = False
                self.monitor.setEnabled(True)
                self.phase.setText('未完成 / 查看原因'); self.live_buttons()
            elif event == 'WORKFLOW_RESULT' and value.get('op') in ('preview_goal','verify_goal'):
                self.live_busy=False;self.live_buttons()
                result=value['result']
                if value['op']=='preview_goal':self.decision.setText(result['description'])
                else:
                    labels={'VERIFIED_COUNT_GOAL':'数量符合目标；是否完全落入盒底仍需核对','VERIFIED_GOAL':'旧版数量核验通过；落位仍需核对','GOAL_NOT_REACHED':'视觉复核：数量目标未达到，请重新观察后补退','UNKNOWN_COUNT_CHANGED':'两盘总数改变，需复核掉落或漏检','PRESERVED_PART_CHANGED':'螺母发生变化，需复核'}
                    self.decision.setText(labels.get(result['status'],result['status']))
                    self.phase.setText('数量核验通过' if result['status']=='VERIFIED_COUNT_GOAL' else '数量核验待处理')
                    counts=result.get('decision',{}).get('counts')
                    if counts:
                        self.metrics.setText(f"当前 A（右盘）：{counts['A']['screw']}颗螺丝\n当前 B（左盘）：{counts['B']['screw']}颗螺丝\n实际落位另行核对")
            elif event == 'WORKFLOW_RESULT' and value.get('op') == 'board_status':
                self.monitor.setEnabled(True)

    def live_mode_changed(self):
        live=self.mode.currentIndex()==3
        for widget in (self.run,self.pause,self.pause_task,self.resume_task,self.import_btn,self.scene_btn):
            widget.setVisible(not live)
        self.pause_task.setEnabled(not live);self.resume_task.setEnabled(not live)
        self.pause.setEnabled(not live);self.view.setVisible(not live)
        self.goal_mode.setVisible(live);self.goal_preview.setVisible(live)
        self.live_prepare.setVisible(live);self.live_start.setVisible(live);self.live_disable.setVisible(live)
        if live:
            self.playing=False
            self.metrics.setText('本轮实际搬运结果：等待视觉核验\n右盘=A，左盘=B；螺母可设为0')
            self.run.setEnabled(False)

    def live_buttons(self):
        enabled = not (self.live_owner or self.live_busy) and self.live_command_valid
        goal_mode=self.goal_mode.currentIndex()==1
        ready=enabled and (not goal_mode or self.live_goal is not None)
        cfg=json.loads((self.root/'config/live_workflow.json').read_text())
        released=cfg.get('inventory_goal_execution_enabled',False)
        self.live_prepare.setEnabled(ready)
        self.live_start.setEnabled(ready and (not goal_mode or released))
        self.goal_preview.setEnabled(ready and goal_mode)
        self.screw_count.setEnabled(not goal_mode and not (self.live_owner or self.live_busy))
        self.nut_count.setEnabled(not goal_mode and not (self.live_owner or self.live_busy))
        self.goal_mode.setEnabled(not (self.live_owner or self.live_busy))
        self.live_start.setText('执行补退任务（手已移开）' if goal_mode and released else '补退执行 · 待双向实物验证' if goal_mode else '一键抓取 · 右盘 → 左盘（手已移开）')

    def invalidate_live_intent(self):
        if self.mode.currentIndex()==3:
            # Editing the instruction during preparation revokes its pending
            # launch, but never interrupts an already running board owner.
            self.live_auto_execute=False
            self.live_command_valid=False;self.live_goal=None;self.live_buttons()

    def manual_live_counts(self):
        if self.applying_live_intent or self.goal_mode.currentIndex()==1:return
        self.live_command_valid=True;self.live_goal=None;self.intent=None;self.live_buttons()
        self.decision.setText(f'手动任务：{self.screw_count.value()}颗螺丝、{self.nut_count.value()}颗螺母 → 左盘B盒')

    def workflow_stopped(self):
        self.workflow = None; self.live_busy = False; self.live_poll_pending = False
        self.live_buttons()
        if self.live_owner:
            self.append('监控进程已退出；板端任务不会因此被停止或失能。')

    def update_live(self):
        if self.mode.currentIndex() == 3:
            try:
                active = json.loads((self.app_root/'reports/rgbd_surface_preview_active.json').read_text())
                folder = Path(active['report_dir'])
                frame = json.loads((folder/'latest.json').read_text())
                age = time.monotonic()-frame['host_acquired_monotonic_s']
                self.image.show(); self.image_caption.show()
                self.image_caption.setText(f'实时 D435i · 帧龄 {age:.2f} 秒 · A=右盘 / B=左盘')
                self.image.setPixmap(QPixmap(str(folder/'latest_preview.jpg')).scaled(680, 330, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
            except (OSError, ValueError, KeyError):
                self.image_caption.setText('相机未连接或暂无新帧')
        if self.live_owner and not self.live_poll_pending:
            self.live_poll_pending = True
            self.workflow_request(dict(op='poll', directory=self.live_directory))

    def disable_live(self):
        answer = QMessageBox.question(self, '结束本轮使能', '确认机械臂已稳妥承托，可以结束本轮七轴使能？',
                                      QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes:
            self.live_disable.setEnabled(False)
            self.workflow_request(dict(op='supported_disable', directory=self.live_directory, confirmation='已承托，可以失能'))

    def apply_live_intent(self, intent):
        from .live_session import counts_from_intent
        self.live_command_valid=False
        try:screws,nuts=counts_from_intent(intent)
        except ValueError as error:
            self.live_buttons();self.append(str(error));return
        self.applying_live_intent=True
        self.screw_count.setValue(screws);self.nut_count.setValue(nuts)
        self.applying_live_intent=False;self.live_command_valid=True;self.live_buttons()
        self.append('已填入实物数量；点击一键抓取后执行。模型理解不会自动启动。')

    def goal_mode_changed(self):
        self.invalidate();self.live_goal=None
        self.decision.setText('请输入“让B盒恰好有三颗螺丝”并理解指令' if self.goal_mode.currentIndex()==1 else '请重新输入搬运数量，或使用手动数量')
        self.live_buttons()

    def apply_goal_intent(self,intent):
        from .inventory_goal import validate
        self.live_goal=validate(intent);self.intent=intent;self.live_command_valid=True
        self.live_buttons();self.preview_goal()

    def preview_goal(self):
        if self.live_owner or self.live_busy or self.live_goal is None:return
        self.live_busy=True;self.live_buttons()
        self.workflow_request(dict(op='preview_goal',goal=self.live_goal))
