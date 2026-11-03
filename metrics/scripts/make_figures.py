#!/usr/bin/env python3
"""Comparison figures for the report and the video, drawn from the raw result files in metrics/logs (1920x1080, dark surface).

    python3 make_figures.py --logs ../metrics/logs --out ../metrics/evidence/figures

Colours follow one rule across all figures: software/CPU/ARM path = blue (series 1), hardware path (PL / NPU / board loop) = orange (series 2).
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

SURFACE, INK, INK2, GRID = '#1a1a19', '#f0efec', '#c3c2b7', '#3a3a38'
BLUE, ORANGE, AQUA = '#3987e5', '#d95926', '#199e70'      # categorical slots 1-3, dark-mode steps
for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Droid Sans Fallback'):
    if any(f.name == name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = name
        break
plt.rcParams.update({'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE, 'text.color': INK, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': GRID, 'axes.unicode_minus': False})


def load(p):
    return json.loads(Path(p).read_text())


def frame(title, subtitle, size=(19.2, 10.8)):
    fig = plt.figure(figsize=size, dpi=100)
    fig.text(.05, .93, title, fontsize=34, fontweight='bold', color=INK, va='top')
    fig.text(.05, .875, subtitle, fontsize=18, color=INK2, va='top')
    return fig


def style(ax, grid_axis='x'):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.spines['left'].set_color(GRID);ax.spines['bottom'].set_color(GRID)
    ax.grid(axis=grid_axis, color=GRID, linewidth=.8);ax.set_axisbelow(True)
    ax.tick_params(length=0, labelsize=17)


def fig_alignment(logs, out):
    run = load(logs / 'board_run_stats.json')
    base = json.loads((logs / 'servo_timing_stop_and_look.jsonl').read_text().splitlines()[0])
    al = run['continuous_alignment_seconds']
    rows = [('停顿对准（取三帧 → AI PC → 板端校验）', base['hover_to_descent_s']['p50'], base['hover_to_descent_s']['p95'], BLUE, f"{base['parts']} 个零件")]
    for key, label in (('screw_m12', '板端连续对准 · 大螺丝'), ('nut_m12', '板端连续对准 · 大螺母'), ('nut_m6', '板端连续对准 · 小螺母')):
        v = al[key]['converged_seconds'];rows.append((label, v['p50'], v['p95'], ORANGE, f"{al[key]['converged']}/{al[key]['attempts']} 次收敛"))
    fig = frame('悬停到收敛：板端闭环把对准压到 1–2.5 秒', '连续对准不停稳取图、不经网络；柱=P50，竖线=P95')
    ax = fig.add_axes([.30, .12, .62, .66]);style(ax)
    y = list(range(len(rows)))[::-1]
    for yi, (label, p50, p95, color, note) in zip(y, rows):
        ax.barh(yi, p50, height=.46, color=color)
        ax.plot([p95, p95], [yi - .3, yi + .3], color=INK, linewidth=2.5)
        ax.text(p95 + .15, yi, f'P50 {p50:.2f} s · P95 {p95:.2f} s · {note}', va='center', fontsize=17, color=INK)
    ax.set_yticks(y);ax.set_yticklabels([r[0] for r in rows]);ax.set_xlim(0, 16);ax.set_xlabel('秒', fontsize=17)
    fig.savefig(out / 'fig_alignment_time.png');plt.close(fig)


def fig_detector(logs, out):
    c = {k: load(logs / f'npu_contention_{k}.json') for k in ('gpu_idle', 'gpu_vlm', 'npu_idle', 'npu_vlm')}
    ratio = lambda a, b: c[b]['p95'] / c[a]['p95']
    fig = frame(f"大模型同载下：NPU 检测 P95 仅 +{(ratio('npu_idle', 'npu_vlm') - 1) * 100:.0f}%，GPU 升到 {ratio('gpu_idle', 'gpu_vlm'):.1f} 倍",
                'Qwen2.5-VL 连续生成时，头部检测的完整单帧延迟（P95，ms）；GPU 检测与大模型争用同一块 Radeon 890M')
    ax = fig.add_axes([.10, .12, .82, .66]);style(ax, 'y')
    xs = {'空闲': 0, '大模型同载': 1};w = .3
    for i, (name, color, idle, vlm) in enumerate((('GPU（ROCm FP32）', BLUE, 'gpu_idle', 'gpu_vlm'), ('NPU（Quark A8W8）', ORANGE, 'npu_idle', 'npu_vlm'))):
        for label, key in (('空闲', idle), ('大模型同载', vlm)):
            x = xs[label] + (i - .5) * (w + .04)
            ax.bar(x, c[key]['p95'], width=w, color=color, label=name if label == '空闲' else None)
            ax.text(x, c[key]['p95'] + 6, f"{c[key]['p95']:.0f}", ha='center', fontsize=22, fontweight='bold', color=INK)
            ax.text(x, -20, f"P50 {c[key]['p50']:.0f}", ha='center', fontsize=15, color=INK2)
    ax.set_xticks(list(xs.values()));ax.set_xticklabels(list(xs.keys()));ax.set_ylabel('ms（P95）', fontsize=17, labelpad=14);ax.set_ylim(0, 340);ax.tick_params(axis='x', pad=52)
    ax.legend(frameon=False, fontsize=18, loc='upper left', labelcolor=INK)
    fig.savefig(out / 'fig_detector_under_vlm.png');plt.close(fig)


def fig_pl(logs, out):
    vp = load(logs / 'board_bench_20261028.json');tr = load(logs / 'tracking_latency_loopback_20261028.json')
    panels = [('腕部图像掩码延迟 P50', 'ms', vp['arm_c_O3_one_core']['p50_ms'], vp['pl_sleep_poll_0.5ms']['p50_ms'], 'ARM A53 单核 C -O3', 'PL 掩码核'),
              ('每帧 CPU 时间', 'ms/帧', vp['arm_c_O3_one_core']['cpu_ms_per_frame'], vp['pl_sleep_poll_0.5ms']['cpu_ms_per_frame'], 'ARM 单核', 'PL（睡眠轮询）'),
              ('七轴跟踪看门狗检测延迟 P50', 'ms', tr['arm_rule_detect_ms']['p50'], tr['pl_detect_ms']['p50'], 'ARM 0.15 s 采样判据', 'PL 逐帧判定')]
    fig = frame('PL 承担的功能：更快、更省 CPU、判定更及时', '同一输入、同一算法/同一故障条件下的对照；KV260，PL 100 MHz')
    for i, (title, unit, arm, pl, la, lp) in enumerate(panels):
        ax = fig.add_axes([.06 + i * .31, .14, .26, .62]);style(ax, 'y')
        ax.bar([0, 1], [arm, pl], color=[BLUE, ORANGE], width=.6)
        for x, v in ((0, arm), (1, pl)):
            ax.text(x, v + max(arm, pl) * .02, f'{v:.2f}' if v < 20 else f'{v:.1f}', ha='center', fontsize=24, fontweight='bold', color=INK)
        ax.set_xticks([0, 1]);ax.set_xticklabels([la, lp], fontsize=14);ax.set_ylim(0, max(arm, pl) * 1.18);ax.set_title(f'{title}（{unit}）', fontsize=19, color=INK, pad=14)
        ax.text(.5, .93, f'{arm / pl:.2f}×', transform=ax.transAxes, ha='center', fontsize=30, color=ORANGE, fontweight='bold')
    fig.savefig(out / 'fig_pl_vs_arm.png');plt.close(fig)


def fig_trace(logs, out):
    data = load(logs / 'continuous_trace_example.json')
    c = data[0];t = c['trace']
    fig = frame('PL 修正通道在工作：目标 → PL 已施加偏移', f"大螺丝，板端连续对准 {c['seconds']:.2f} s 收敛；每个点是一帧视觉修正，PL 逐拍限速地跟随")
    ax = fig.add_axes([.08, .38, .84, .40]);style(ax, 'y')
    ts = [x['t'] for x in t]
    ax.plot(ts, [x['target_mm'][0] for x in t], color=BLUE, linewidth=2.5, label='视觉环写入的目标（工具 x，mm）')
    ax.plot(ts, [x['applied_mm'][0] for x in t], color=ORANGE, linewidth=2.5, label='PL 已施加偏移（工具 x，mm）')
    ax.set_ylabel('mm', fontsize=17);ax.legend(frameon=False, fontsize=17, loc='lower right', labelcolor=INK);ax.set_xticklabels([])
    ax2 = fig.add_axes([.08, .10, .84, .22]);style(ax2, 'y')
    ax2.plot(ts, [x['err'] for x in t], color=AQUA, linewidth=2.5)
    ax2.axhline(6, color=INK2, linewidth=1.2, linestyle=(0, (4, 4)));ax2.text(ts[0], 7.5, '收敛阈值 6 px', fontsize=14, color=INK2)
    ax2.set_ylabel('像素误差 px', fontsize=17);ax2.set_xlabel('秒（相对对准开始）', fontsize=17)
    fig.savefig(out / 'fig_correction_trace.png');plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser();p.add_argument('--logs', type=Path, required=True);p.add_argument('--out', type=Path, required=True);a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    for f in (fig_alignment, fig_detector, fig_pl, fig_trace):
        f(a.logs, a.out);print('wrote', f.__name__)
