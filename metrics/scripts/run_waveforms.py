#!/usr/bin/env python3
"""Tracking error and PL-timestamped feedback timing from one board run record (execution/result.json.gz).

    python3 run_waveforms.py ../logs/run_20261030_221647_result.json.gz --out ../logs/run_waveforms.json [--figure ../evidence/figures/fig_tracking_waveform.png]

1. Tracking (J2, J3): the commanded position (loc_ref 0x7016, read back from the motor) against the measured feedback position, sampled by
   the owner's guard during the run: RMSE, peak error, share of samples above 10 mrad, and the waveform.
2. Feedback timing (all 7 motors): the interval between consecutive feedback frames, from the receive timestamps that the PL's 1 us
   counter attaches to every CAN frame (`pl_rx_us`): P50 / P95 / P99 / max and the count of intervals > 2x median, for the window in which the
   PL trajectory stream runs (`feedback_interval_ms`) and for the whole record including the enable start-up and the final disable.
"""
import argparse
import gzip
import json
import math
import statistics


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))]


def main():
    ap = argparse.ArgumentParser();ap.add_argument('record');ap.add_argument('--out', required=True);ap.add_argument('--figure')
    a = ap.parse_args()
    r = json.loads(gzip.decompress(open(a.record, 'rb').read()))
    out = dict(record=a.record.split('/')[-1], tracking={}, feedback_interval_ms={})
    series = {}
    t0 = min(x['time'] for x in r['tracking_diagnostics'])
    for m in (2, 3):
        rows = [x for x in r['tracking_diagnostics'] if x['motor'] == m]
        err = [x['following_error'] * 1e3 for x in rows]               # mrad
        series[m] = ([x['time'] - t0 for x in rows], [x['loc_ref'] for x in rows], [x['feedback_position'] for x in rows], err)
        out['tracking'][f'J{m}'] = dict(samples=len(rows), rmse_mrad=math.sqrt(statistics.fmean(e * e for e in err)), mean_abs_mrad=statistics.fmean(abs(e) for e in err),
                                        peak_mrad=max(abs(e) for e in err), share_over_10_mrad=sum(abs(e) > 10 for e in err) / len(err),
                                        following_limit_mrad=rows[0]['following_limit_rad'] * 1e3, current_limit_a=rows[0]['current_limit_a'],
                                        peak_temperature_c=max(x['temperature_c'] for x in rows))
    # window of the running PL trajectory stream: from 0.3 s after the first path starts to the last path's completion
    paths = [e for e in r['events'] if e['event'] in ('PATH_START', 'PATH_COMPLETE')]
    t_run0 = next(e['time'] for e in paths if e['event'] == 'PATH_START') + 0.3
    t_run1 = max(e['time'] for e in paths if e['event'] == 'PATH_COMPLETE')
    out['stream_window_s'] = t_run1 - t_run0
    out['feedback_interval_ms_whole_record'] = {}
    for m in range(1, 8):
        rows = [x for x in r['feedback'] if x['motor'] == m]
        for name, sel in (('feedback_interval_ms', [x for x in rows if t_run0 <= x['time'] <= t_run1]), ('feedback_interval_ms_whole_record', rows)):
            stamps = [x['pl_rx_us'] for x in sel]
            dt = [((b - a_) & 0xffffffff) / 1e3 for a_, b in zip(stamps, stamps[1:])]
            med = statistics.median(dt)
            out[name][f'motor_{m}'] = dict(frames=len(stamps), p50=q(dt, .5), p95=q(dt, .95), p99=q(dt, .99), max=max(dt), median=med, over_2x_median=sum(d > 2 * med for d in dt))
    out['duration_s'] = (r['events'][-1]['time'] - r['events'][0]['time'])
    open(a.out, 'w').write(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1)[:1800])
    if a.figure:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        cjk = next((n for n in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'Droid Sans Fallback') if any(f.name == n for f in font_manager.fontManager.ttflist)), 'DejaVu Sans')
        plt.rcParams.update({'figure.facecolor': '#1a1a19', 'axes.facecolor': '#1a1a19', 'savefig.facecolor': '#1a1a19', 'text.color': '#f0efec', 'axes.labelcolor': '#c3c2b7',
                             'xtick.color': '#c3c2b7', 'ytick.color': '#c3c2b7', 'axes.edgecolor': '#3a3a38', 'font.family': cjk, 'axes.unicode_minus': False})
        fig, axes = plt.subplots(2, 1, figsize=(19.2, 10.8), dpi=100, gridspec_kw=dict(height_ratios=[3, 1.4]), sharex=True)
        ts, cmd, act, err = series[2]
        axes[0].plot(ts, cmd, color='#3987e5', lw=2.4, label='指令位置 loc_ref（J2）');axes[0].plot(ts, act, color='#d95926', lw=2.0, label='实测反馈位置（J2）')
        axes[0].set_ylabel('rad', fontsize=17);axes[0].legend(frameon=False, fontsize=18, loc='upper right', labelcolor='#f0efec')
        axes[1].plot(ts, err, color='#199e70', lw=2.0);axes[1].set_ylabel('跟踪误差 mrad', fontsize=17);axes[1].set_xlabel('秒（一次完整四件运行）', fontsize=17)
        for ax in axes:
            for s in ('top', 'right'):
                ax.spines[s].set_visible(False)
            ax.grid(axis='y', color='#3a3a38', lw=.8);ax.tick_params(length=0, labelsize=16)
        t = out['tracking']['J2']
        fig.text(.05, .95, f"J2 指令与反馈波形：跟踪 RMSE {t['rmse_mrad']:.1f} mrad，峰值 {t['peak_mrad']:.1f} mrad", fontsize=32, fontweight='bold', va='top')
        fig.text(.05, .905, f"一次完整四件运行 {out['duration_s']:.0f} s；{t['samples']} 个采样；保护限 {t['following_limit_mrad']:.0f} mrad（持续 300 ms）", fontsize=18, color='#c3c2b7', va='top')
        fig.subplots_adjust(left=.07, right=.96, top=.84, bottom=.09, hspace=.08)
        fig.savefig(a.figure);plt.close(fig)


if __name__ == '__main__':
    main()
