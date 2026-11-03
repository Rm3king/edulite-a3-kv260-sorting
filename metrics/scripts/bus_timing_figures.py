#!/usr/bin/env python3
"""Figures from bus_timing_analysis.py outputs: the deviation histogram of the 5 ms tick and one zoomed bus window.

    python3 bus_timing_figures.py --json bus_timing.json --frames bus_frames.npz --out DIR
"""
import argparse
import json

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

INK, MUTED, GRID, BG = '#f0efec', '#c3c2b7', '#3a3a38', '#1a1a19'
BLUE, ORANGE, GREEN = '#3987e5', '#d95926', '#199e70'
BUCKETS = [1, 2, 5, 10, 25, 50, 100, 250, 500, 1000, 2000]


def style():
    cjk = next((n for n in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Droid Sans Fallback') if any(f.name == n for f in font_manager.fontManager.ttflist)), 'DejaVu Sans')
    plt.rcParams.update({'figure.facecolor': BG, 'axes.facecolor': BG, 'savefig.facecolor': BG, 'text.color': INK, 'axes.labelcolor': MUTED, 'xtick.color': MUTED, 'ytick.color': MUTED,
                         'axes.edgecolor': GRID, 'font.family': cjk, 'axes.unicode_minus': False})


def clean(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.grid(axis='y', color=GRID, lw=.8);ax.tick_params(length=0, labelsize=16)


def histogram(report, frames, out):
    st = report['steady_stream']
    first = frames['start'][frames['ident'] == 0x1200FD01];first = first[first >= st['start_s']]
    dev = np.abs(np.diff(first) - 5e-3) * 1e6
    edges = [0] + BUCKETS + [1e9]
    counts = np.histogram(dev, bins=edges)[0]
    labels = [f'≤{b}' for b in BUCKETS] + ['>2000']
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
    bars = ax.bar(range(len(counts)), counts, color=BLUE, width=.7)
    ax.set_yscale('log');ax.set_ylim(.8, counts.max() * 4)
    ax.set_xticks(range(len(counts)), labels);clean(ax)
    ax.set_xlabel('PL 周期偏差 |间隔 − 5 ms|，µs（线上帧起点）', fontsize=18);ax.set_ylabel('周期数（对数轴）', fontsize=18)
    for b, c in zip(bars, counts):
        if c:
            ax.text(b.get_x() + b.get_width() / 2, c * 1.25, f'{c}', ha='center', fontsize=15, color=INK)
    d = st['deviation_us']
    fig.text(.06, .95, f"5 ms 周期线上实测：P50 {d['p50']:.1f} µs，P95 {d['p95']:.0f} µs，P99 {d['p99']:.0f} µs，最大 {d['max']:.0f} µs", fontsize=30, fontweight='bold', va='top')
    fig.text(.06, .905, f"逻辑分析仪（50 MHz，CAN-H）稳态流段 {st['seconds']:.0f} s，{st['ticks']} 个周期；间隔中位数 {st['interval_ms']['p50']:.4f} ms，最大 {st['interval_ms']['max']:.2f} ms",
             fontsize=18, color=MUTED, va='top')
    fig.subplots_adjust(left=.07, right=.97, top=.84, bottom=.1)
    fig.savefig(out / 'fig_wire_period.png');plt.close(fig)


def window(report, frames, out):
    st = report['steady_stream']
    start, dur, ident = frames['start'], frames['duration'], frames['ident']
    first = start[ident == 0x1200FD01];first = first[first >= st['start_s']]
    reads = start[((ident >> 24) & 31) == 17]
    dev = np.abs(np.diff(first) - 5e-3)
    k = int(np.argmax(dev > 800e-6))                               # a tick that was delayed by a read transaction
    t0 = first[k] - 1.2e-3;t1 = t0 + 13e-3
    lanes = [('指令帧 0x7016（轴 1–7）', BLUE, lambda i: ((i >> 24) & 31) == 18),
             ('电机反馈', ORANGE, lambda i: ((i >> 24) & 31) == 2),
             ('位置读取请求/回复', GREEN, lambda i: ((i >> 24) & 31) == 17)]
    fig, ax = plt.subplots(figsize=(19.2, 10.8), dpi=100)
    sel = (start >= t0) & (start <= t1) & (dur > 20e-6)
    for row, (name, color, test) in enumerate(lanes):
        m = sel & np.array([test(int(x)) if x >= 0 else False for x in ident])
        for s, d in zip(start[m], dur[m]):
            ax.barh(row, d * 1e3, left=(s - t0) * 1e3, height=.55, color=color, edgecolor=BG, lw=1)
    for s in first[(first >= t0) & (first <= t1)]:
        ax.axvline((s - t0) * 1e3, color=MUTED, lw=1.2, ls=(0, (4, 4)))
    ax.set_yticks(range(len(lanes)), [l[0] for l in lanes]);ax.invert_yaxis();ax.set_xlim(0, (t1 - t0) * 1e3);ax.set_ylim(len(lanes) - .4, -.6)
    clean(ax);ax.grid(axis='y', visible=False);ax.grid(axis='x', color=GRID, lw=.8);ax.tick_params(axis='y', labelsize=19)
    ax.set_xlabel('ms（虚线：每个周期第一个指令帧的起点；位置读取优先级高于指令帧）', fontsize=18)
    ticks = first[(first >= t0) & (first <= t1)]
    delta = np.diff(ticks) * 1e3 if len(ticks) > 1 else []
    fig.text(.06, .95, '一个周期内的总线时序：7 个指令帧、电机反馈与位置读取', fontsize=30, fontweight='bold', va='top')
    grid = '；晚点的周期之后，下一周期回到 5 ms 栅格（两个间隔之和 %.3f ms）' % (delta[0] + delta[1]) if len(delta) > 1 and delta[0] > 5 else ''
    fig.text(.06, .905, '相邻周期起点间隔 ' + '、'.join(f'{x:.3f}' for x in delta) + ' ms' + grid, fontsize=18, color=MUTED, va='top')
    fig.subplots_adjust(left=.2, right=.97, top=.84, bottom=.1)
    fig.savefig(out / 'fig_wire_tick.png');plt.close(fig)


def main():
    ap = argparse.ArgumentParser();ap.add_argument('--json', required=True);ap.add_argument('--frames', required=True);ap.add_argument('--out', required=True)
    a = ap.parse_args()
    style()
    report = json.load(open(a.json));frames = np.load(a.frames)
    from pathlib import Path
    out = Path(a.out);out.mkdir(parents=True, exist_ok=True)
    histogram(report, frames, out);window(report, frames, out)


if __name__ == '__main__':
    main()
