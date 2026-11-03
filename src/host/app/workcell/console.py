"""Operator console: one page, one sentence. Local web server (127.0.0.1 only) around `OrderAgent`.

The page sends an order; understanding, planning, grasping, checking and deciding happen behind it. Two human gates remain by
design: "the arm is at the support pose" before a run, and "the arm is supported" after a fault hold.
"""
import argparse
import json
import platform
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.request import urlopen

from .order_agent import Events, OrderAgent
from .order_language import friendly_error
from .teach import Teach, TeachError
from .twin import Twin, find_urdf

PAGE = Path(__file__).parent / 'assets/console/index.html'


def platform_info():
    cpu = next((l.split(':', 1)[1].strip() for l in open('/proc/cpuinfo') if l.startswith('model name')), 'unknown')
    memory = next((int(l.split()[1]) // 1048576 for l in open('/proc/meminfo') if l.startswith('MemTotal')), 0)
    rocm = Path('/opt/rocm/.info/version')
    return dict(cpu=cpu, memory_gb=memory, rocm=rocm.read_text().strip() if rocm.exists() else 'n/a', kernel=platform.release(), gpu='AMD Radeon 890M Graphics')


def detector_info(app):
    """Which processor the live head-camera detector is using now (NPU, or the GPU fallback / GPU)."""
    try:
        active = json.loads((app / 'reports/rgbd_surface_preview_active.json').read_text())
        folder = Path(active['report_dir'])
        report, latest = json.loads((folder / 'report.json').read_text()), json.loads((folder / 'latest.json').read_text())
        return dict(backend=latest.get('inference_backend', report.get('environment', {}).get('backend', 'gpu')),
                    inference_ms=latest.get('inference_ms'), failure=report.get('environment', {}).get('failure'))
    except (OSError, KeyError, ValueError):
        return None


class Console:
    def __init__(self, app_root, worker=None):
        self.app = Path(app_root).resolve()
        self.events = Events(listener=self._remember)
        self.log = []
        self.lock = threading.Lock()
        self.agent = OrderAgent(self.app, worker=worker, events=self.events)
        self.busy = False
        self.report = None
        self.plan = None
        self.temperatures = None
        self.counts = None
        self.fault = False
        self.model = worker.info if worker else None
        self.twin = Twin(find_urdf(self.app))
        self.teach = Teach(self.app, json.loads((self.app / 'integration/config/live_workflow.json').read_text())['ssh_helper'])

    def _remember(self, item):
        with self.lock:
            self.log.append(item)
            if item['kind'] == 'observe' and 'counts' in item:self.counts = item['counts']
            if 'temperatures' in item:self.temperatures = item['temperatures']
            if item['kind'] == 'fault':self.fault = True
            if item['kind'] == 'disabled':self.fault = False

    def state(self, since=0):
        with self.lock:
            return dict(state=self.agent.state, busy=self.busy, fault=self.fault, events=self.log[since:], next=len(self.log), report=self.report,
                        plan=self.plan and dict(parts=self.plan.get('parts'), batches=[dict(parts=b['parts'], slots=b['slots'], reason=b['reason']) for b in self.plan['batches']]),
                        temperatures=self.temperatures, counts=self.counts, model=bool(self.model), alignments=list(self.agent.alignments))

    def order(self, text, ready, skip_cooling=False):
        if self.busy:
            raise ValueError('正在执行，请等待本轮结束')
        if self.teach.state in ('STARTING', 'ACTIVE', 'FINISHING'):
            raise ValueError('零重力示教正在运行，请先结束示教')
        self.busy, self.report, self.fault = True, None, False
        threading.Thread(target=self._run, args=(text, ready, skip_cooling), daemon=True).start()

    def _run(self, text, ready, skip_cooling=False):
        try:
            self.events('order', text)
            self.plan = self.agent.plan(text, skip_cooling)
            if self.plan['task']['action'] != 'plan':
                if not self.plan.get('refused'):
                    self.events('result', '需要澄清：' + (self.plan['task'].get('question') or '这句话我没法执行'))
            elif ready:
                self.report = self.agent.execute(self.plan, operator_ready=True)
            else:
                self.events('decide', '已规划好；勾选“机械臂在支撑位”后再次发送即可开始执行')
        except Exception as error:
            self.agent.state = 'FAULT_HELD' if self.fault else 'ERROR'
            self.events('error', friendly_error(error))
        finally:
            self.busy = False


def serve(console, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):pass

        def send(self, code, body, kind='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code);self.send_header('Content-Type', kind);self.send_header('Content-Length', str(len(data)));self.send_header('Cache-Control', 'no-store');self.end_headers();self.wfile.write(data)

        def do_GET(self):
            path, _, query = self.path.partition('?')
            if path == '/':
                return self.send(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
            if path == '/api/state':
                since = int(dict(p.split('=') for p in query.split('&') if '=' in p).get('since', 0))
                return self.send(200, console.state(since))
            if path == '/api/teach/state':
                snap = console.teach.snapshot()
                twin = console.twin
                live = snap.get('live')
                snap['twin'] = dict(points=twin.points(live['joint_rad'][:6])) if live else None
                for pose in snap.get('poses', []):
                    pose['tool'] = twin.tool(pose['joint_rad'][:6])
                return self.send(200, snap)
            if path == '/api/teach/replay':
                index = int(dict(p.split('=') for p in query.split('&') if '=' in p).get('i', 0))
                rows = console.teach.trajectories[index]['samples']
                return self.send(200, dict(name=console.teach.trajectories[index]['name'], t=[r['t'] for r in rows],
                                           points=[console.twin.points(r['joint_rad'][:6]) for r in rows]))
            if path == '/api/platform':
                return self.send(200, dict(platform_info(), rocm=(console.model or {}).get('hip', 'n/a'), model=console.model, detector=detector_info(console.app)))
            if path in ('/api/head.jpg', '/api/wrist.jpg'):
                try:
                    if path == '/api/head.jpg':
                        active = json.loads((console.app / 'reports/rgbd_surface_preview_active.json').read_text())
                        from io import BytesIO
                        from PIL import Image
                        # The detector's preview is a composite (text header above, coordinate list on the right): keep the camera view only.
                        with Image.open(Path(active['report_dir']) / 'latest_preview.jpg') as composite:
                            view = composite.crop((0, 82, min(960, composite.width), composite.height))
                            buffer = BytesIO();view.save(buffer, 'JPEG', quality=88)
                        return self.send(200, buffer.getvalue(), 'image/jpeg')
                    with urlopen('http://127.0.0.1:18766/frame.jpg', timeout=1) as response:
                        return self.send(200, response.read(), 'image/jpeg')
                except Exception:
                    return self.send(404, b'', 'image/jpeg')
            self.send(404, dict(error='not found'))

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
            try:
                if self.path == '/api/order':
                    text = str(body.get('text', '')).strip()
                    if not text or len(text) > 200:raise ValueError('指令为空或过长')
                    console.order(text, bool(body.get('ready')), bool(body.get('skip_cooling')))
                    return self.send(200, dict(ok=True))
                if self.path.startswith('/api/teach/'):
                    teach, action = console.teach, self.path.rsplit('/', 1)[1]
                    if action == 'start':
                        teach.start(str(body.get('confirmation', '')), busy=console.busy)
                    elif action == 'pose':
                        teach.record_pose(body.get('name', ''))
                    elif action == 'mark':
                        teach.mark(str(body.get('action', '')), body.get('name', ''))
                    elif action == 'finish':
                        teach.finish()
                    else:
                        raise TeachError('未知的示教操作')
                    return self.send(200, dict(ok=True, teach=teach.snapshot()))
                if self.path == '/api/confirm_supported':
                    if body.get('confirmation') != '已承托，可以失能':raise ValueError('需要操作员确认')
                    console.agent.confirm_supported()
                    return self.send(200, dict(ok=True))
            except Exception as error:
                return self.send(400, dict(ok=False, error=str(error)))
            self.send(404, dict(error='not found'))

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'CONSOLE http://127.0.0.1:{port}/', flush=True)
    server.serve_forever()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app-root', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--port', type=int, default=8780)
    p.add_argument('--no-model', action='store_true')
    a = p.parse_args()
    worker = None
    if not a.no_model:
        from .order_language import QwenWorker
        worker = QwenWorker(a.app_root)
    try:
        serve(Console(a.app_root, worker), a.port)
    finally:
        if worker:worker.close()


if __name__ == '__main__':
    main()
