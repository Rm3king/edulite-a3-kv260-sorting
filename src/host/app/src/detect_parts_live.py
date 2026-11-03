"""D435i screw/nut recognition preview, with an offline replay check.

Default is dry-run. --run opens the color camera; no photos are saved in live
mode. --replay DATASET --out NEW_DIR --run instead checks saved images without
opening a camera. No robot, depth coordinates, training or tracking commands.
"""
import argparse
from collections import Counter, deque
from datetime import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import time

import cv2
import numpy as np

from annotate_parts import CLASSES, COLORS

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / 'aipc_app/reports/preview-capture-20261017-152340/capture_plan.json'
WINDOW = 'D435i screw / nut detection - Q: quit'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_model(plan_path=None):
    plan = json.loads((Path(plan_path) if plan_path is not None else PLAN).read_text())
    checkpoint = ROOT / plan['checkpoint']
    if digest(checkpoint) != plan['checkpoint_sha256']:
        raise ValueError('模型哈希与已验证版本不符，请保留原文件并检查。')
    return checkpoint, plan


def configure_window():
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('没有图形桌面，请在小主机桌面终端运行。')
    if not Path(os.environ.get('QT_QPA_FONTDIR', '/nonexistent')).is_dir():
        for path in ('/usr/share/fonts/truetype/dejavu', '/usr/share/fonts/truetype/liberation2'):
            if Path(path).is_dir():
                os.environ['QT_QPA_FONTDIR'] = path
                break


class Detector:
    def __init__(self, checkpoint, plan, device):
        import torch
        self.torch = torch
        if device == 'gpu' and (not torch.version.hip or not torch.cuda.is_available()):
            raise RuntimeError('AMD GPU不可用。请用aipc.sh gpu入口；可显式加 --device cpu 排查。')
        self.device = 0 if device == 'gpu' else 'cpu'
        self.conf = plan['confidence_threshold']
        self.imgsz = int(plan.get('imgsz', 640))   # the 4-class model is trained and checked at 1280
        # Keep settings/cache outside the datasets and disable automatic installs.
        os.environ.update(YOLO_CONFIG_DIR=str(Path.home() / '.cache/aipc/parts-live-ultralytics'),
                          YOLO_AUTOINSTALL='false', MPLBACKEND='Agg')
        from ultralytics import YOLO, settings, __version__
        settings.update({key: False for key in ('sync', 'hub', 'wandb', 'mlflow', 'clearml',
                                               'comet', 'neptune', 'dvc', 'tensorboard') if key in settings})
        self.model = YOLO(str(checkpoint), task='segment')
        if self.model.names != CLASSES:
            raise ValueError('模型类别必须与 annotate_parts.CLASSES 一致（0=screw、1=nut、2=screw_m6、3=nut_m6）。')
        self.environment = {'torch': torch.__version__, 'hip': torch.version.hip,
                            'ultralytics': __version__, 'device': device,
                            'device_name': torch.cuda.get_device_name(0) if device == 'gpu' else 'CPU'}

    def predict(self, bgr, imgsz=None):
        imgsz = self.imgsz if imgsz is None else imgsz
        if not isinstance(imgsz, int) or imgsz < 320 or imgsz > 1920 or imgsz % 32:
            raise ValueError('imgsz must be a multiple of 32 between 320 and 1920.')
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('需要uint8 BGR图像。')
        if self.device == 0:
            self.torch.cuda.synchronize()
        started = time.perf_counter()
        # Both OpenCV replay and the camera use BGR, as required by NumPy input.
        result = self.model.predict(bgr, device=self.device, imgsz=imgsz, conf=self.conf,
                                    iou=0.7, retina_masks=True, quantize=32, verbose=False,
                                    save=False, save_txt=False, show=False)[0]
        classes = result.boxes.cls.cpu().numpy().astype(np.int64)
        scores = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        masks = ((result.masks.data.cpu().numpy() > 0.5).astype(np.uint8)
                 if result.masks is not None else np.zeros((0, *bgr.shape[:2]), dtype=np.uint8))
        if len(masks) != len(classes) or masks.shape[1:] != bgr.shape[:2]:
            raise ValueError('分割输出与当前图像尺寸不符。')
        if any(int(cid) not in CLASSES for cid in classes):
            raise ValueError('检测到未定义的模型类别。')
        if self.device == 0:
            self.torch.cuda.synchronize()
        elapsed_ms = 1000 * (time.perf_counter() - started)
        return {'classes': classes, 'scores': scores, 'boxes': boxes, 'masks': masks}, elapsed_ms


class NpuDetector:
    """The same predict() on the Ryzen AI NPU (AMD Quark INT8 of the same checkpoint, VitisAI EP), in its own process.

    The NPU environment (ryzen-ai) has no RealSense/Ultralytics and this one has no VitisAI runtime, so frames go through
    tmpfs and one JSON line each way. Why: the local VLM shares the GPU; with it generating, the GPU detector went from
    42 to 133 ms (p95 67 -> 306 ms) while the NPU one stayed at ~107 ms (2026-10-31, reports/npu_parts_v2_20261031).
    Any NPU failure switches to `fallback` (the GPU detector) for this and all later frames, and says so."""
    MODEL = Path(__file__).resolve().parents[1] / 'reports/npu_parts_v2_20261031/parts_v2_a8w8_736x1280.onnx'

    def __init__(self, fallback=None, model=None, timeout_s=2.):
        import json, subprocess
        self.json, self.fallback, self.timeout = json, fallback, timeout_s
        model = Path(model or self.MODEL)
        integration = Path(__file__).resolve().parents[1] / 'integration'
        self.frame = f'/dev/shm/npu_frame_{os.getpid()}.npy'
        self.process = subprocess.Popen([str(Path.home() / '.local/share/aipc-dev/aipc.sh'), 'npu', 'env', f'PYTHONPATH={integration}', 'python', '-u', '-m',
                                         'workcell.npu_parts', 'serve', '--quant', str(model), '--output', '/dev/shm'],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        deadline, ready = time.monotonic() + 120., None
        while time.monotonic() < deadline:          # the NPU activation script prints its own lines first
            ready = self._line(max(.1, deadline - time.monotonic()))
            if not ready or '"NPU_READY"' in ready:
                break
        if not ready or '"NPU_READY"' not in ready:
            raise RuntimeError('NPU service did not start: ' + str(ready)[:200])
        info = json.loads(ready)
        if 'VitisAIExecutionProvider' not in info['providers']:
            raise RuntimeError('NPU service started without the VitisAI EP: ' + str(info['providers']))
        self.counter, self.failure = 0, None
        self.environment = dict(backend='npu', providers=info['providers'], compile_s=info['compile_s'], model=info['model'],
                                model_sha256=info['model_sha256'], fallback=(fallback.environment if fallback else None))

    def _line(self, timeout):
        import select
        ready, _, _ = select.select([self.process.stdout], [], [], timeout)
        return self.process.stdout.readline() if ready else None

    def predict(self, bgr, imgsz=None):
        if self.failure is None:
            try:
                started = time.perf_counter()
                np.save(self.frame, bgr)
                self.counter += 1
                self.process.stdin.write(self.json.dumps(dict(id=self.counter, frame=self.frame)) + '\n');self.process.stdin.flush()
                line = self._line(self.timeout)
                while line and not line.lstrip().startswith('{'):   # stray non-JSON output of the runtime
                    line = self._line(self.timeout)
                reply = self.json.loads(line) if line else dict(error='timeout')
                if reply.get('error') or reply.get('id') != self.counter:
                    raise RuntimeError(reply.get('error') or 'reply out of order')
                with np.load(reply['out']) as z:
                    prediction = {k: z[k] for k in ('classes', 'scores', 'boxes', 'masks')}
                if prediction['masks'].shape[1:] != bgr.shape[:2] or len(prediction['masks']) != len(prediction['classes']):
                    raise ValueError('NPU segmentation does not match the frame')
                return prediction, 1000 * (time.perf_counter() - started)
            except Exception as error:
                self.failure = f'{type(error).__name__}: {error}'[:300]
                self.environment['failure'] = self.failure
                print(f'NPU_DETECTOR_FAILED {self.failure}; using the GPU detector from now on', flush=True)
                if self.fallback is None:
                    raise
        return self.fallback.predict(bgr, imgsz)

    @property
    def active(self):
        return 'npu' if self.failure is None else 'gpu_fallback'

    def close(self):
        try:
            self.process.stdin.write('{"op": "shutdown"}\n');self.process.stdin.flush();self.process.wait(5)
        except Exception:
            self.process.kill()


def render(bgr, prediction, elapsed_ms, fps=None, scale=1.5, mode='LIVE'):
    drawn = bgr.copy()
    counts = Counter(int(cid) for cid in prediction['classes'])
    height, width = bgr.shape[:2]
    for cid, score, box, mask in zip(prediction['classes'], prediction['scores'],
                                    prediction['boxes'], prediction['masks']):
        cid = int(cid)
        color = COLORS[cid]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(drawn, contours, -1, color, 1)
        x1, y1, x2, y2 = np.rint(box).astype(int)
        x1, x2 = np.clip([x1, x2], 0, width - 1)
        y1, y2 = np.clip([y1, y2], 0, height - 1)
        cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 2)
        text = f'{CLASSES[cid]} {score:.2f}'
        label_y = y1 - 5 if y1 >= 20 else min(height - 5, y2 + 17)
        cv2.putText(drawn, text, (min(x1, max(0, width - 120)), label_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
    drawn = cv2.resize(drawn, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    canvas = cv2.copyMakeBorder(drawn, 104, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
    fps_text = '--' if fps is None else f'{fps:.1f}'
    lines = (f'{mode} | screw {counts[0]} | nut {counts[1]} | screw_m6 {counts[2]} | nut_m6 {counts[3]}',
             f'Inference + CPU outputs: {elapsed_ms:.0f} ms | display FPS: {fps_text}',
             'Q / Esc: quit | No recording | 2D recognition preview')
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (10, 26 + index * 29), cv2.FONT_HERSHEY_SIMPLEX,
                    0.57, (25, 25, 25), 1, cv2.LINE_AA)
    return canvas


def quit_requested():
    key = cv2.waitKey(1) & 0xff
    return key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1


def run_live(detector, scale):
    import pyrealsense2 as rs
    configure_window()
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) != 1:
        raise RuntimeError(f'需要恰好1台RealSense，当前发现{len(devices)}台，请检查USB连接。')
    config = rs.config()
    config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
    # This display needs color only; RGB-D measurement remains a separate step.
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe = rs.pipeline(ctx)
    # Single-slot queue replaces old frames instead of building a growing delay.
    queue = rs.frame_queue(1)
    started = False
    times = deque(maxlen=30)
    try:
        pipe.start(config, queue)
        started = True
        for _ in range(30):
            queue.wait_for_frame(5000)
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        print('实时识别已启动：绿色screw=螺丝，橙色nut=螺母；Q/Esc退出。不保存图片、不占场景编号。', flush=True)
        last_log = time.monotonic()
        while True:
            frames = queue.wait_for_frame(5000).as_frameset()
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError('未收到彩色帧，请检查相机连接。')
            bgr = np.asanyarray(color.get_data()).copy()
            prediction, elapsed_ms = detector.predict(bgr)
            now = time.monotonic()
            times.append(now)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else None
            cv2.imshow(WINDOW, render(bgr, prediction, elapsed_ms, fps, scale))
            if quit_requested():
                break
            if now - last_log >= 5 and fps is not None:
                counts = Counter(int(cid) for cid in prediction['classes'])
                print(f'显示更新约{fps:.1f} FPS；screw={counts[0]} nut={counts[1]} screw_m6={counts[2]} nut_m6={counts[3]}；本帧推理及输出{elapsed_ms:.0f}ms', flush=True)
                last_log = now
    finally:
        if started:
            pipe.stop()
        cv2.destroyAllWindows()


def check_replay(dataset, output):
    dataset, output = dataset.resolve(), output.resolve()
    # Keep replay outputs outside datasets; preserve all existing directories.
    if output.exists() or output.is_symlink() or output.is_relative_to(ROOT / 'aipc_app/data'):
        raise ValueError('回放输出请用data目录以外的新目录，已有文件不覆盖。')
    if output.is_relative_to(dataset):
        raise ValueError('回放输出不能位于输入数据集内。')
    manifest = json.loads((dataset / 'manifest.json').read_text())
    if manifest['classes'] != {'0': 'screw', '1': 'nut'} or not manifest['images']:
        raise ValueError('回放数据集类别或图片列表不符。')
    for entry in manifest['images']:
        source = (dataset / entry['file']).resolve()
        if not source.is_relative_to(dataset) or digest(source) != entry['image_sha256']:
            raise ValueError('回放图片路径或哈希不符。')
        image = cv2.imread(str(source))
        if image is None or image.shape != (entry['height'], entry['width'], 3):
            raise ValueError('回放图片尺寸不符。')
    return manifest


def run_replay(detector, dataset, output, manifest, checkpoint, plan, scale, show):
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, output / 'detector_source.py')
    report = {'scope': 'SAVED_IMAGE_INTEGRATION_CHECK_NOT_LIVE_CAMERA_OR_LATENCY_BENCHMARK',
              'created_at': datetime.now().astimezone().isoformat(), 'environment': detector.environment,
              'checkpoint_sha256': digest(checkpoint), 'confidence_threshold': plan['confidence_threshold'],
              'nms_iou': 0.7, 'camera_opened': False, 'robot_access': False,
              'training': False, 'status': 'RUNNING', 'images': []}
    try:
        if show:
            configure_window()
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        for index, entry in enumerate(manifest['images']):
            bgr = cv2.imread(str(dataset / entry['file']))
            if digest(dataset / entry['file']) != entry['image_sha256']:
                raise ValueError('回放期间输入图片变化。')
            prediction, elapsed_ms = detector.predict(bgr)
            picture = render(bgr, prediction, elapsed_ms, scale=scale, mode='SAVED IMAGE REPLAY')
            # Use sequential names; do not trust arbitrary scene IDs as paths.
            stem = f'preview_{index + 1:03d}'
            if not cv2.imwrite(str(output / (stem + '.png')), picture):
                raise OSError('无法写回放预览图。')
            np.savez_compressed(output / (stem + '.npz'), **prediction)
            counts = Counter(int(cid) for cid in prediction['classes'])
            report['images'].append({'scene_id': entry['scene_id'], 'source_sha256': entry['image_sha256'],
                                     'counts': {name: counts[cid] for cid, name in CLASSES.items()},
                                     'preview': stem + '.png', 'predictions': stem + '.npz',
                                     'inference_and_outputs_ms': elapsed_ms})
            if show:
                cv2.imshow(WINDOW, picture)
                if quit_requested():
                    report['status'] = 'REPLAY_STOPPED_EARLY'
                    break
        else:
            report['status'] = 'SAVED_IMAGE_REPLAY_COMPLETE_LIVE_NOT_TESTED'
        if digest(checkpoint) != plan['checkpoint_sha256']:
            raise ValueError('回放期间模型文件变化。')
    except Exception as error:
        report.update(status='FAILED', error=str(error))
        raise
    finally:
        if show:
            cv2.destroyAllWindows()
        (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=('gpu', 'cpu'), default='gpu')
    parser.add_argument('--scale', type=float, default=1.5)
    parser.add_argument('--replay', type=Path, help='Offline dataset with manifest.json; never opens a camera')
    parser.add_argument('--out', type=Path, help='New output directory, required only with --replay')
    parser.add_argument('--show', action='store_true', help='Also display replay images in a desktop window')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    if not 0.75 <= args.scale <= 2:
        parser.error('scale必须为0.75..2。')
    if bool(args.replay) != bool(args.out) or (args.show and not args.replay):
        parser.error('--replay与--out需要同时指定；--show仅适用于回放。')
    try:
        checkpoint, plan = check_model()
        manifest = check_replay(args.replay, args.out) if args.replay else None
        required = ('torch', 'ultralytics') + (() if args.replay else ('pyrealsense2',))
        for module in required:
            if importlib.util.find_spec(module) is None:
                raise RuntimeError(f'当前Python缺少{module}，请使用.venv-yolo及aipc.sh gpu入口。')
        print(f'模型：{checkpoint}\n类别：0=screw（螺丝），1=nut（螺母）；conf={plan["confidence_threshold"]}', flush=True)
        if not args.run:
            print('DRY RUN：模型及依赖检查完成；未加载模型、未打开相机、未创建输出。加 --run 才执行。')
            return
        detector = Detector(checkpoint, plan, args.device)
        print(json.dumps(detector.environment, ensure_ascii=False), flush=True)
        print('正在预热模型，首次启动请稍等……', flush=True)
        for _ in range(3):
            detector.predict(np.zeros((480, 640, 3), dtype=np.uint8))
        if args.replay:
            run_replay(detector, args.replay, args.out, manifest, checkpoint, plan, args.scale, args.show)
        else:
            run_live(detector, args.scale)
    except KeyboardInterrupt:
        print('已退出识别预览。')
    except (RuntimeError, ValueError, OSError, ImportError) as error:
        parser.exit(1, f'{error}\n相机错误：关闭其他采图/RealSense Viewer窗口并检查USB；GPU错误：核对aipc.sh gpu入口。无需重装驱动。\n')


if __name__ == '__main__':
    main()
