"""Post-run result review: what the run recorded, the head-camera views before and after, and the local VLM's account of it.

The facts are the system's own records (per part: outcome, alignment mode and time, destination slot, the placement check);
the model (Qwen2.5-VL on the AI PC GPU, base weights) is asked to compare them with the images and explain a failure in
plain Chinese. It is read-only: it runs after the arm has stopped, its text is shown and stored, and it never feeds back
into motion. Without a model the deterministic summary is shown instead. (2026-10-31)
"""
import json
import re
import time
from pathlib import Path

from .order_planner import ZH

MAX_CHARS = 400
# What a recorded status means, in the words the operator and the model should use (the raw codes made the model guess:
# it called RETURN_VISION_REJECTED a head-camera error, but it is the wrist camera's alignment image being refused).
STATUS_ZH = {
    'RETURN_VISION_REJECTED': '腕部相机对准时图像被拒（看不清或不稳定），机械臂已按审核路径安全返回',
    'RETURN_VISION_LOST': '腕部相机连续对准时跟丢零件，机械臂已安全返回',
    'RETURN_CONTINUOUS_TIMEOUT': '腕部连续对准在时限内没有收敛，机械臂已安全返回',
    'RETURN_VISION_ERROR': '腕部视觉程序出错，机械臂已安全返回',
    'TASK_FAILED_RETURNED_SUPPORTED': '夹爪闭合后没有夹住零件（空夹），机械臂已安全返回',
    'RETURN_BUDGET_OR_TEMPERATURE': '电机温度或时间到达上限，提前返回',
    'RETURN_CAPTURE_REJECTED': '抓取前的采图检查没有通过，没有抓取',
    'NOT_ATTEMPTED': '本轮没有执行到这一件',
}
TRAY_CROP = (('A', '执行前的源盘（右盘 A）'), ('B', '执行后的目标盘（左盘 B）'))


def plain_reason(reason):
    return re.sub(r'状态 ([A-Z_]+)', lambda m: STATUS_ZH.get(m[1], m[1]), reason)


def color_image(points_path):
    """The head camera's colour frame saved next to a snapshot's points file, or None."""
    if not points_path:
        return None
    p = Path(points_path)
    image = p.with_name(p.name.split('_points')[0] + '_color.png')
    return image if image.exists() else None


def tray_crop(image, polygon, out, margin=24):
    """The tray's part of a 1920x1080 head frame: the model sees at most ~0.3 Mpx, the whole frame left the parts a few pixels."""
    from PIL import Image
    with Image.open(image) as im:
        xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
        box = (max(0, min(xs) - margin), max(0, min(ys) - margin), min(im.width, max(xs) + margin), min(im.height, max(ys) + margin))
        im.crop(box).convert('RGB').save(out, 'JPEG', quality=92)
    return out


def inside(point, polygon):
    """Ray casting: is the pixel inside the tray polygon."""
    x, y, hit = point[0], point[1], False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def tray_counts(points_path, polygon):
    """Detector (YOLO) objects whose centre lies in the tray: {class: n}. The 3B VLM could not count these small parts
    reliably (2026-10-31: four clearly visible parts read as one or three), so counts are the detector's, stated as such."""
    try:
        objects = json.loads(Path(points_path).read_text())['objects']
    except (OSError, ValueError, KeyError, TypeError):
        return None
    counts = {}
    for o in objects:
        c = o.get('center_uv_px')
        if c and inside(c, polygon):
            counts[o['part_class']] = counts.get(o['part_class'], 0) + 1
    return counts


def facts(order, report, alignments, temperatures=None, target_counts=None):
    """Plain Chinese lines the model can only restate or contradict with the images; also the fallback summary."""
    lines = [f'指令：{order}', f"结果：{plain_reason(report['text'])}"]
    if report.get('stop_reason'):
        lines.append('程序记录的停止原因：' + report['stop_reason'])
    if target_counts is not None:
        seen = '、'.join(f'{ZH.get(c, c)} {n} 个' for c, n in sorted(target_counts.items())) or '没有零件'
        lines.append(f'执行后头部相机（YOLO 检测）在目标盘里看到：{seen}')
    for part, slot in report['done']:
        lines.append(f'{ZH.get(part, part)}：已放入目标盘第 {slot} 号槽，头部相机已确认')
    for part, slot, reason in report['failed']:
        lines.append(f'{ZH.get(part, part)}：未完成（{plain_reason(reason)}）')
    for x in report.get('pending', []):
        part = x['part'] if isinstance(x, dict) else x
        lines.append(f'{ZH.get(part, part)}：没有执行')
    for a in alignments:
        lines.append(a['text'])
    if temperatures:
        lines.append('结束时 J2 温度 %.0f°C，J3 %.0f°C' % (temperatures.get(2, float('nan')), temperatures.get(3, float('nan'))))
    return '\n'.join(lines)


def review(worker, order, report, alignments, before=None, after=None, temperatures=None, trays=None, direction='A->B'):
    """-> dict(summary, source 'vlm'|'records', seconds, images, facts). Never raises.

    `trays` (live_workflow.json 'trays'): image 1 is the source tray A before the run, image 2 the target tray B after it."""
    crop = TRAY_CROP if direction == 'A->B' else (('B', '执行前的源盘（左盘 B）'), ('A', '执行后的目标盘（右盘 A）'))
    counts = tray_counts(after, trays[crop[1][0]]['roi_uv']) if trays and after else None
    text = facts(order, report, alignments, temperatures, counts)
    images = []
    for (tray, label), frame in zip(crop, (before, after)):
        image = color_image(frame)
        if image is None:
            continue
        try:
            images.append(tray_crop(image, trays[tray]['roi_uv'], image.with_name(image.stem + f'_review_{tray}.jpg')) if trays else image)
        except (OSError, KeyError, ValueError):
            images.append(image)
    if trays and len(images) == 2:
        text = '图1：' + crop[0][1] + '；图2：' + crop[1][1] + '。\n' + text
    out = dict(facts=text, images=[str(p) for p in images], source='records', summary=plain_reason(report['text']), seconds=0., target_counts=counts)
    if report.get('stop_reason'):
        # Fault results use verified placements and the recorded stop cause;
        # a model must not infer successful work from pre-existing tray objects.
        out['summary'] += '；' + report['stop_reason']
        return out
    if worker is None or not hasattr(worker, 'review'):
        return out
    started = time.perf_counter()
    try:
        reply = worker.review(text, images)
        summary = ' '.join(str(reply.get('summary', '')).split())[:MAX_CHARS]
        if summary:
            out.update(summary=summary, source='vlm', model_seconds=reply.get('generation_s'))
    except Exception as error:   # the review is commentary; a model problem must not turn a finished run into an error
        out['error'] = f'{type(error).__name__}: {error}'[:200]
    out['seconds'] = time.perf_counter() - started
    return out
