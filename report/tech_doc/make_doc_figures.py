"""Print-style figures for the technical document (white background, 300 dpi PNG (Agg renders the CJK fallback; matplotlib's PDF backend cannot embed the CID-keyed Fandol fonts)), drawn from the package's raw logs.

    python3 make_doc_figures.py --logs <package>/metrics/logs --out <tech_doc>/figures

Colors: light sky blue for the software / GPU / ARM baseline, light apricot
orange for NPU / PL / this design, and mint for a third series. Bright pastel
fills and medium-tone strokes keep both bars and curves clear on white paper.
One y-scale per axes; values are labeled directly where there are few marks.
"""
import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

TL = Path('/mnt/aipc-data/Wenq_WorkSpace/tools/texlive/texmf-dist/fonts/opentype/public')
for f in ['tex-gyre/texgyretermes-regular.otf', 'tex-gyre/texgyretermes-bold.otf', 'fandol/FandolSong-Regular.otf', 'fandol/FandolSong-Bold.otf']:
    if (TL / f).exists():
        font_manager.fontManager.addfont(str(TL / f))
plt.rcParams.update({
    'font.family': ['TeX Gyre Termes', 'FandolSong'], 'savefig.dpi': 300, 'font.size': 9, 'axes.titlesize': 9.5, 'axes.labelsize': 9, 'xtick.labelsize': 8.5,
    'ytick.labelsize': 8.5, 'legend.fontsize': 8.5, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.edgecolor': '#a4adb4',
    'axes.linewidth': .6, 'xtick.color': '#59636b', 'ytick.color': '#59636b', 'axes.grid': True, 'grid.color': '#e8ecef', 'grid.linewidth': .5,
    'text.color': '#303b43', 'axes.labelcolor': '#303b43', 'axes.titlecolor': '#303b43',
    'xtick.major.size': 2.5, 'ytick.major.size': 2.5, 'xtick.major.width': .6, 'ytick.major.width': .6,
    'axes.axisbelow': True, 'legend.frameon': False, 'axes.unicode_minus': False, 'savefig.bbox': 'tight', 'savefig.pad_inches': .03})
BLUE, ORANGE, AQUA, INK, MUTED = '#68a8da', '#e9a05a', '#70bca7', '#303b43', '#808a91'
BLUE_FILL, ORANGE_FILL, AQUA_FILL = '#aed8f7', '#ffd2a6', '#b4e2d5'
W = 6.2                                                       # inches, matches the text width


def load(p):
    return json.loads(Path(p).read_text())


def label_bars(ax, bars, fmt='{:.0f}', dy=0.0, inside=False):
    for b in bars:
        v = b.get_width() if b.get_height() < b.get_width() and ax.get_yaxis().get_ticklabels() and False else b.get_height()
        ax.annotate(fmt.format(v), (b.get_x() + b.get_width() / 2, v), xytext=(0, 2 + dy), textcoords='offset points', ha='center', va='bottom',
                    fontsize=8, color=INK)


def contention(logs, out):
    c = {k: load(logs / f'npu_contention_{k}.json') for k in ('gpu_idle', 'gpu_vlm', 'npu_idle', 'npu_vlm')}
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.3), sharey=True)
    x = np.arange(2);w = .34
    for ax, stat, title in ((axes[0], 'p50', '(a) P50'), (axes[1], 'p95', '(b) P95')):
        g = ax.bar(x - w / 2 - .01, [c['gpu_idle'][stat], c['gpu_vlm'][stat]], w, color=BLUE_FILL, edgecolor=BLUE, linewidth=.5, label='GPU（ROCm FP32）')
        n = ax.bar(x + w / 2 + .01, [c['npu_idle'][stat], c['npu_vlm'][stat]], w, color=ORANGE_FILL, edgecolor=ORANGE, linewidth=.5, label='NPU（Quark A8W8）')
        label_bars(ax, g);label_bars(ax, n)
        ax.set_xticks(x, ['检测单独运行', '大模型同时生成']);ax.set_title(title, loc='left');ax.grid(axis='x', visible=False)
    axes[0].set_ylabel('单帧检测延迟 / ms');axes[0].set_ylim(0, max(c['gpu_vlm']['p95'], 1) * 1.18)
    axes[0].legend(loc='upper left')
    fig.savefig(out / 'doc_contention.png');plt.close(fig)


def wire(logs, out):
    report = load(logs / 'bus_timing.json');frames = np.load(logs / 'bus_frames.npz');st = report['steady_stream']
    start, dur, ident = frames['start'], frames['duration'], frames['ident']
    first = start[ident == 0x1200FD01];first = first[first >= st['start_s']]
    dev = np.sort(np.abs(np.diff(first) - 5e-3) * 1e6)
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.45), gridspec_kw=dict(width_ratios=[1, 1.25]))
    ax = axes[0];y = np.arange(1, len(dev) + 1) / len(dev)
    ax.plot(np.maximum(dev, .05), y, color=ORANGE, lw=1.6);ax.set_xscale('log');ax.set_xlim(.05, 2000);ax.set_ylim(0, 1.02)
    for q, name in ((.5, 'P50'), (.95, 'P95'), (.99, 'P99')):
        v = st['deviation_us'][name.lower()];ax.axhline(q, color='#cbd2d7', lw=.6, ls=(0, (3, 3)))
        ax.plot([max(v, .05)], [q], 'o', ms=4, color=INK, zorder=3)
        ax.annotate(f'{name} {v:.1f} µs' if v < 10 else f'{name} {v:.0f} µs', (max(v, .05), q), xytext={'P50': (5, -9), 'P95': (-6, 3), 'P99': (-6, -13)}[name], ha='left' if name == 'P50' else 'right', textcoords='offset points', fontsize=8)
    ax.set_xlabel('周期间隔偏差 |间隔 − 5 ms| / µs（对数轴）');ax.set_ylabel('累积比例');ax.set_title(f'(a) {st["ticks"]} 个周期的偏差分布', loc='left')
    # (b) one zoomed window: a late tick and the recovery to the 5 ms grid
    ax = axes[1]
    d = np.abs(np.diff(first) - 5e-3);k = int(np.argmax(d > 800e-6));t0 = first[k] - 1.0e-3;t1 = t0 + 12e-3
    lanes = [('指令帧（7 轴）', BLUE_FILL, 18), ('电机反馈', ORANGE_FILL, 2), ('位置读取', AQUA_FILL, 17)]
    sel = (start >= t0) & (start <= t1) & (dur > 20e-6)
    for row, (name, color, typ) in enumerate(lanes):
        m = sel & (((ident >> 24) & 31) == typ) & (ident >= 0)
        ax.broken_barh([((s - t0) * 1e3, dd * 1e3) for s, dd in zip(start[m], dur[m])], (row - .3, .6), facecolors=color, edgecolors=[BLUE, ORANGE, AQUA][row], lw=.25)
    ticks = first[(first >= t0) & (first <= t1)]
    for s in ticks:
        ax.axvline((s - t0) * 1e3, color=MUTED, lw=.7, ls=(0, (3, 2)))
    for a, b in zip(ticks, ticks[1:]):
        ax.annotate(f'{(b - a) * 1e3:.3f} ms', (((a + b) / 2 - t0) * 1e3, 2.62), ha='center', fontsize=7.5, color=INK)
    ax.set_yticks(range(3), [l[0] for l in lanes]);ax.set_ylim(2.95, -.6);ax.set_xlim(0, (t1 - t0) * 1e3);ax.grid(axis='y', visible=False)
    ax.set_xlabel('时间 / ms（虚线：每周期首个指令帧起点）');ax.set_title('(b) 一次晚点及其后回到 5 ms 栅格', loc='left')
    fig.tight_layout(w_pad=1.2);fig.savefig(out / 'doc_wire.png');plt.close(fig)


def tracking(logs, out):
    r = json.loads(gzip.decompress((logs / 'run_20261030_221647_result.json.gz').read_bytes()))
    t0 = min(x['time'] for x in r['tracking_diagnostics'])
    fig, axes = plt.subplots(2, 1, figsize=(W, 3.3), sharex=True, gridspec_kw=dict(height_ratios=[1.35, 1]))
    for m, color in ((2, BLUE), (3, ORANGE)):
        rows = [x for x in r['tracking_diagnostics'] if x['motor'] == m]
        t = np.array([x['time'] - t0 for x in rows]);cmd = np.array([x['loc_ref'] for x in rows]);act = np.array([x['feedback_position'] for x in rows])
        err = np.array([x['following_error'] for x in rows]) * 1e3
        axes[0].plot(t, cmd, color=color, lw=1.1, label=f'J{m} 指令');axes[0].plot(t, act, color=color, lw=.9, ls=(0, (2.5, 1.5)), label=f'J{m} 反馈')
        axes[1].plot(t, err, color=color, lw=.8, label=f'J{m}')
        lim = rows[0]['following_limit_rad'] * 1e3
    axes[1].axhline(lim, color=MUTED, lw=.7, ls=(0, (4, 2)));axes[1].axhline(-lim, color=MUTED, lw=.7, ls=(0, (4, 2)))
    axes[1].annotate(f'保护阈值 ±{lim:.0f} mrad（持续 300 ms 才判定）', (t[0], lim), xytext=(2, 2), textcoords='offset points', ha='left', fontsize=7.5, color=MUTED)
    axes[0].set_ylabel('关节位置 / rad');axes[1].set_ylabel('跟踪误差 / mrad');axes[1].set_xlabel('时间 / s')
    axes[0].legend(ncol=4, loc='upper center', bbox_to_anchor=(.5, 1.2));axes[1].legend(ncol=2, loc='lower left')
    fig.tight_layout(h_pad=.4);fig.savefig(out / 'doc_tracking.png');plt.close(fig)


def alignment(logs, out):
    run = load(logs / 'board_run_stats.json')['continuous_alignment_seconds']
    base_rows = [json.loads(l) for l in (logs / 'servo_timing_stop_and_look.jsonl').read_text().splitlines()]
    base = [r['hover_to_descent_s'] for row in base_rows if 'rows' in row for r in row['rows'] if r.get('hover_to_descent_s') is not None]
    summary = base_rows[0]['hover_to_descent_s']
    fig, axes = plt.subplots(1, 2, figsize=(W, 2.5), gridspec_kw=dict(width_ratios=[1.15, 1]))
    ax = axes[0]
    items = [('停顿：计至下降', None, BLUE), ('大螺丝·连续', 'screw_m12', ORANGE), ('大螺母·连续', 'nut_m12', ORANGE), ('小螺母·连续', 'nut_m6', ORANGE)]
    for i, (name, key, color) in enumerate(items):
        if key is None:
            p50, p95, note = summary['p50'], summary['p95'], f'{len(base) or base_rows[0]["parts"]} 个零件'
            if base:
                jitter = (np.random.default_rng(1).random(len(base)) - .5) * .28
                ax.scatter(base, i + jitter, s=7, color=color, alpha=.35, lw=0, zorder=2)
        else:
            v = run[key]['converged_seconds'];p50, p95 = v['p50'], v['p95'];note = f'{run[key]["converged"]}/{run[key]["attempts"]} 收敛'
        ax.plot([p50, p95], [i, i], color=color, lw=2.2, solid_capstyle='round', zorder=3)
        ax.plot([p50], [i], 'o', ms=5.5, color=color, mec='white', mew=.8, zorder=4);ax.plot([p95], [i], '|', ms=8, color=color, mew=1.6, zorder=4)
        ax.annotate(f'{p50:.2f} s  ({note})', (p95, i), xytext=(6, -3), textcoords='offset points', fontsize=7.8, color=INK)
    ax.set_yticks(range(len(items)), [x[0] for x in items]);ax.set_ylim(len(items) - .5, -.6);ax.set_xlim(0, 15.5);ax.grid(axis='y', visible=False)
    ax.set_xlabel('悬停后的用时 / s（圆点 P50，竖线 P95）');ax.set_title('(a) 连续计至收敛，停顿计至下降', loc='left')
    ax = axes[1]
    c = load(logs / 'continuous_trace_example.json')[0];t = [x['t'] for x in c['trace']]
    ax.plot(t, [x['err'] for x in c['trace']], color=ORANGE, lw=1.4, marker='o', ms=3, label='像素偏差 |e|')
    ax.set_xlabel('时间 / s');ax.set_ylabel('像素偏差 / px');ax.set_title(f'(b) 一次大螺丝对准（{c["seconds"]:.2f} s 收敛）', loc='left')
    ax2 = ax.twinx() if False else None
    ax.set_ylim(0, None)
    fig.tight_layout(w_pad=1.5);fig.savefig(out / 'doc_alignment.png');plt.close(fig)


def pl_vs_arm(logs, out):
    vp = load(logs / 'board_bench_20261028.json');tr = load(logs / 'tracking_latency_loopback_20261028.json')
    arm, pl = vp['arm_c_O3_one_core'], vp['pl_sleep_poll_0.5ms']
    fig, axes = plt.subplots(1, 3, figsize=(W, 2.15))
    panels = [('(a) 掩码延迟 P50 / ms', arm['p50_ms'], pl['p50_ms'], '{:.2f}'), ('(b) 每帧 CPU 时间 / ms', arm['cpu_ms_per_frame'], pl['cpu_ms_per_frame'], '{:.2f}')]
    for ax, (title, a, p, fmt) in zip(axes[:2], panels):
        bars = ax.bar([0, 1], [a, p], .58, color=[BLUE_FILL, ORANGE_FILL], edgecolor=[BLUE, ORANGE], linewidth=.5);label_bars(ax, bars, fmt)
        ax.set_xticks([0, 1], ['ARM 单核', 'PL 掩码核']);ax.set_ylim(0, max(a, p) * 1.2);ax.set_title(title, loc='left');ax.grid(axis='x', visible=False)
    ax = axes[2];rows = tr['rows'];rng = np.random.default_rng(2)
    for i, (key, summary, color) in enumerate((('arm_ms', 'arm_rule_detect_ms', BLUE), ('pl_ms', 'pl_detect_ms', ORANGE))):
        v = np.array([r[key] for r in rows]);ax.scatter(i + (rng.random(len(v)) - .5) * .3, v, s=9, color=color, alpha=.75, lw=0)
        p50 = tr[summary]['p50']                                  # the reported P50, so figure, text and metrics.csv agree
        ax.plot([i - .25, i + .25], [p50] * 2, color=INK, lw=1.2);ax.annotate(f'P50 {p50:.1f}', (i + .27, p50), va='center', fontsize=8)
    ax.set_xticks([0, 1], ['ARM 判据', 'PL 看门狗']);ax.set_xlim(-.6, 1.95);ax.set_title(f'(c) 看门狗检测延迟 / ms（{len(rows)} 次）', loc='left');ax.grid(axis='x', visible=False)
    fig.tight_layout(w_pad=1.0);fig.savefig(out / 'doc_pl_vs_arm.png');plt.close(fig)


def main():
    ap = argparse.ArgumentParser();ap.add_argument('--logs', type=Path, required=True);ap.add_argument('--out', type=Path, required=True);a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    for f in (contention, wire, tracking, alignment, pl_vs_arm):
        f(a.logs, a.out)


if __name__ == '__main__':
    main()
