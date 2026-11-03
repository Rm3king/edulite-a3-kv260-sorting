"""Local polygon annotation for exported screw/nut images; no camera or robot access."""
import argparse
import hashlib
import json
import math
import os
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

CLASSES = {0: 'screw', 1: 'nut', 2: 'screw_m6', 3: 'nut_m6'}   # 2/3: the bright M6 parts (black M12 screw/nut keep 0/1)
COLORS = {0: (70, 230, 70), 1: (0, 180, 255), 2: (255, 200, 40), 3: (255, 60, 200)}
HEADER = 108
WINDOW = 'Screw / nut annotation'


def cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    def on_segment(p, q, r):
        return cross(p, q, r) == 0 and all(min(p[i], q[i]) <= r[i] <= max(p[i], q[i]) for i in (0, 1))
    values = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    return ((values[0] * values[1] < 0 and values[2] * values[3] < 0)
            or on_segment(a, b, c) or on_segment(a, b, d)
            or on_segment(c, d, a) or on_segment(c, d, b))


def validate_polygon(points, width, height):
    if len(points) < 3 or len({tuple(p) for p in points}) != len(points):
        raise ValueError('Use at least 3 distinct outline points.')
    for point in points:
        if len(point) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
            raise ValueError('Polygon coordinates must be finite x/y pairs.')
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError('Outline point outside image.')
    count = len(points)
    for i in range(count):
        for j in range(i + 1, count):
            if j == i + 1 or (i == 0 and j == count - 1):
                continue
            if segments_intersect(points[i], points[(i + 1) % count], points[j], points[(j + 1) % count]):
                raise ValueError('Outline crosses itself; undo points and redraw.')
    if abs(sum(cross((0, 0), points[i], points[(i + 1) % count]) for i in range(count))) < 8:
        raise ValueError('Polygon has almost no area.')


def yolo_text(objects, width, height):
    rows = []
    for obj in objects:
        if obj['class_id'] not in CLASSES:
            raise ValueError('Unknown class ID.')
        validate_polygon(obj['points'], width, height)
        coordinates = [value for x, y in obj['points'] for value in (x / width, y / height)]
        rows.append(str(obj['class_id']) + ' ' + ' '.join(f'{v:.8f}' for v in coordinates))
    return ''.join(row + '\n' for row in rows)


def image_and_labels(dataset, entry):
    path = dataset / entry['file']
    image = cv2.imread(str(path))
    if image is None or hashlib.sha256(path.read_bytes()).hexdigest() != entry['image_sha256']:
        raise ValueError(f'Image missing or changed: {path}')
    height, width = image.shape[:2]
    if (height, width) != (entry['height'], entry['width']):
        raise ValueError('Image dimensions differ from manifest.')
    state_path = dataset / 'annotations' / (path.stem + '.json')
    label_path = dataset / 'labels' / (path.stem + '.txt')
    if not state_path.exists():
        if label_path.exists():
            raise ValueError(f'Existing external label has no editor state; preserve and review: {label_path}')
        return image, [], False
    state = json.loads(state_path.read_text())
    if state['image_sha256'] != entry['image_sha256'] or state.get('reviewed') is not True:
        raise ValueError('Annotation does not match the reviewed image.')
    objects = state['objects']
    expected = yolo_text(objects, width, height)
    if not label_path.exists() or label_path.read_text() != expected:
        raise ValueError(f'JSON and YOLO labels disagree: {label_path}; preserve files and review.')
    return image, objects, True


def save_labels(dataset, entry, objects):
    text = yolo_text(objects, entry['width'], entry['height'])
    stem = Path(entry['file']).stem
    state_path = dataset / 'annotations' / (stem + '.json')
    label_path = dataset / 'labels' / (stem + '.txt')
    state = {'schema_version': 1, 'reviewed': True, 'reviewed_at': datetime.now().astimezone().isoformat(),
             'image_sha256': entry['image_sha256'], 'scene_id': entry['scene_id'],
             'confirmed_empty': not objects, 'objects': objects}
    for path in (state_path, label_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            history = dataset / 'annotations' / 'history'
            history.mkdir(exist_ok=True)
            (history / f'{stem}_{time.time_ns()}{path.suffix}').write_bytes(path.read_bytes())
    # Temporary files avoid leaving a partially written individual label on interruption.
    for path, content in ((label_path, text), (state_path, json.dumps(state, ensure_ascii=False, indent=2) + '\n')):
        temporary = path.with_name(path.name + '.tmp')
        temporary.write_text(content)
        temporary.replace(path)


def check_dataset(dataset, manifest):
    saved, empty, objects = 0, 0, 0
    for entry in manifest['images']:
        _, labels, reviewed = image_and_labels(dataset, entry)
        saved += int(reviewed)
        empty += int(reviewed and not labels)
        objects += len(labels)
    result = {'reviewed_images': saved, 'total_images': len(manifest['images']),
              'confirmed_empty_images': empty, 'annotated_objects': objects}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result


class Editor:
    def __init__(self, dataset, manifest, scale, resume=True):
        self.dataset, self.entries, self.scale = dataset, manifest['images'], scale
        self.index, self.class_id = 0, 0
        self.message, self.quit_pending = '', False
        if resume:
            for i, entry in enumerate(self.entries):
                _, _, reviewed = image_and_labels(self.dataset, entry)
                if not reviewed:
                    self.index = i
                    break
        self.load()

    def load(self):
        self.image, self.objects, self.saved = image_and_labels(self.dataset, self.entries[self.index])
        self.points, self.dirty = [], False

    def mouse(self, event, x, y, flags, parameter):
        if event == cv2.EVENT_RBUTTONDOWN:
            if self.points:
                self.points.pop()
            return
        if event != cv2.EVENT_LBUTTONDOWN or y < HEADER:
            return
        x, y = int(x / self.scale), int((y - HEADER) / self.scale)
        height, width = self.image.shape[:2]
        if 0 <= x < width and 0 <= y < height:
            point = [x, y]
            if point not in self.points:
                self.points.append(point)

    def render(self):
        canvas = self.image.copy()
        for obj in self.objects:
            vertices = np.asarray(obj['points'], dtype=np.int32)
            color = COLORS[obj['class_id']]
            cv2.polylines(canvas, [vertices], True, color, 1, cv2.LINE_AA)
            cv2.putText(canvas, CLASSES[obj['class_id']], tuple(vertices[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        if self.points:
            vertices = np.asarray(self.points, dtype=np.int32)
            cv2.polylines(canvas, [vertices], False, COLORS[self.class_id], 1, cv2.LINE_AA)
            for point in self.points:
                cv2.circle(canvas, tuple(point), 2, COLORS[self.class_id], -1)
        canvas = cv2.resize(canvas, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_LINEAR)
        canvas = cv2.copyMakeBorder(canvas, HEADER, 0, 0, 0, cv2.BORDER_CONSTANT, value=(245, 245, 245))
        lines = [
            f'{self.index + 1}/{len(self.entries)}  {Path(self.entries[self.index]["file"]).name}  selected={CLASSES[self.class_id]}  objects={len(self.objects)}',
            '1=screw  2=nut  Left-click=outline  Enter=finish  Right-click=undo point',
            'U=remove last object  Esc=cancel outline  S=save/next  P=previous  Q=quit',
            self.message or ('Saved labels loaded.' if self.saved and not self.dirty else 'Review all objects; empty tray: press S with no objects.'),
        ]
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (8, 23 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1, cv2.LINE_AA)
        return canvas

    def run(self, smoke=False):
        # Some OpenCV wheels point Qt at a fonts directory absent from the wheel.
        # Select an existing system font directory for this process only.
        font_dir = os.environ.get('QT_QPA_FONTDIR')
        if not font_dir or not Path(font_dir).is_dir():
            for candidate in (Path('/usr/share/fonts/truetype/dejavu'), Path('/usr/share/fonts/truetype/liberation2')):
                if candidate.is_dir():
                    os.environ['QT_QPA_FONTDIR'] = str(candidate)
                    break
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.mouse)
        try:
            while True:
                cv2.imshow(WINDOW, self.render())
                key = cv2.waitKey(50) & 0xff
                if smoke:
                    print('GUI render smoke: PASS; no labels written')
                    return
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    print('Window closed; unsaved edits were not written.')
                    return
                if key == 255:
                    continue
                if key == ord('q'):
                    if (self.dirty or self.points) and not self.quit_pending:
                        self.message = 'Unsaved edits: S to save, or press Q again to discard and quit.'
                        self.quit_pending = True
                        continue
                    return
                self.quit_pending = False
                try:
                    if key in (ord('1'), ord('2')):
                        if self.points:
                            raise ValueError('Finish or cancel the current outline before changing class.')
                        self.class_id = key - ord('1')
                    elif key in (10, 13):
                        height, width = self.image.shape[:2]
                        validate_polygon(self.points, width, height)
                        self.objects.append({'class_id': self.class_id, 'points': self.points.copy()})
                        self.points, self.dirty, self.saved = [], True, False
                    elif key in (8, 127):
                        if self.points:
                            self.points.pop()
                    elif key == 27:
                        self.points = []
                    elif key == ord('u'):
                        if self.points:
                            raise ValueError('Esc cancels the current outline; U removes a finished object.')
                        if self.objects:
                            self.objects.pop()
                            self.dirty, self.saved = True, False
                    elif key == ord('s'):
                        if self.points:
                            raise ValueError('Press Enter to finish the current object, or Esc to cancel it.')
                        save_labels(self.dataset, self.entries[self.index], self.objects)
                        print(f'已保存：{self.entries[self.index]["file"]}，物体数 {len(self.objects)}')
                        if self.index == len(self.entries) - 1:
                            check_dataset(self.dataset, {'images': self.entries})
                            return
                        self.index += 1
                        self.load()
                    elif key == ord('p'):
                        if self.dirty or self.points:
                            raise ValueError('Save current edits before moving to another image.')
                        self.index = max(0, self.index - 1)
                        self.load()
                    self.message = ''
                except ValueError as error:
                    self.message = str(error)
        finally:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--scale', type=float, default=1.5)
    parser.add_argument('--check', action='store_true', help='Check saved annotations without opening a window')
    parser.add_argument('--smoke', action='store_true', help='Render once without writing labels')
    parser.add_argument('--from-first', action='store_true', help='Review from the first image instead of resuming the first unreviewed image')
    args = parser.parse_args()
    if not math.isfinite(args.scale) or not 0.5 <= args.scale <= 3:
        parser.error('--scale must be 0.5..3')
    manifest = json.loads((args.dataset / 'manifest.json').read_text())
    if manifest['classes'] != {'0': 'screw', '1': 'nut'} or not manifest['images']:
        parser.error('Expected a nonempty screw/nut dataset from prepare_part_images.py')
    if args.check:
        check_dataset(args.dataset, manifest)
    else:
        if not args.smoke:
            print('标注已有照片：1=螺丝，2=螺母；左键沿物体外轮廓点选，Enter完成一颗。')
            print('S保存本图并进入下一张；右键撤回一个点；U删除最后一颗；Esc取消当前轮廓；Q退出。')
        Editor(args.dataset, manifest, args.scale, resume=not args.from_first).run(args.smoke)


if __name__ == '__main__':
    main()
