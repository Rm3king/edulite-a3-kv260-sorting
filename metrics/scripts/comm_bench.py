#!/usr/bin/env python3
"""AI PC <-> KV260 communication benchmark (runs on the AI PC; no motor command is sent).

    python3 comm_bench.py --host 192.168.26.2 --ssh "bash src/host/deploy/ssh_kv260.sh" --out ../logs/comm_bench.json

Measures on the dedicated 1 GbE link: ICMP RTT and loss, SSH command round trip (the transport of the job packages),
sustained effective throughput in both directions through the same SSH channel, and the wrist-camera HTTP frame fetch.
Reports P50/P95/P99/max, never only the mean.
"""
import argparse
import json
import re
import shlex
import statistics
import subprocess
import time
from urllib.request import urlopen


def quantiles(values):
    values = sorted(values)
    pick = lambda p: values[min(len(values) - 1, int(p * len(values)))]
    return dict(n=len(values), p50=pick(.5), p95=pick(.95), p99=pick(.99), max=values[-1], mean=statistics.fmean(values))


def ping(host, count, interval):
    out = subprocess.run(['ping', '-c', str(count), '-i', str(interval), '-q', host], capture_output=True, text=True).stdout
    loss = float(re.search(r'([\d.]+)% packet loss', out)[1])
    rtt = re.search(r'= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+)', out)
    samples = subprocess.run(['ping', '-c', str(count), '-i', str(interval), host], capture_output=True, text=True).stdout
    times = [float(x) for x in re.findall(r'time=([\d.]+) ms', samples)]
    return dict(sent=count, loss_percent=loss, rtt_ms=quantiles(times), summary_min_avg_max_mdev=[float(x) for x in rtt.groups()])


def ssh_round_trip(ssh, runs):
    times = []
    for _ in range(runs):
        start = time.perf_counter()
        subprocess.run(shlex.split(ssh) + ['true'], check=True, capture_output=True)
        times.append((time.perf_counter() - start) * 1e3)
    return quantiles(times)


def throughput(ssh, megabytes):
    size = megabytes * 1024 * 1024
    result = {}
    start = time.perf_counter()
    subprocess.run(shlex.split(ssh) + [f'head -c {size} /dev/urandom'], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    result['board_to_host_MBps'] = megabytes / (time.perf_counter() - start)
    start = time.perf_counter()
    source = subprocess.Popen(['head', '-c', str(size), '/dev/urandom'], stdout=subprocess.PIPE)
    subprocess.run(shlex.split(ssh) + ['cat > /dev/null'], stdin=source.stdout, check=True, stderr=subprocess.DEVNULL)
    result['host_to_board_MBps'] = megabytes / (time.perf_counter() - start)
    return result


def frame_fetch(url, runs):
    times, sizes = [], []
    for _ in range(runs):
        start = time.perf_counter()
        with urlopen(url, timeout=3) as response:
            sizes.append(len(response.read()))
        times.append((time.perf_counter() - start) * 1e3)
    return dict(latency_ms=quantiles(times), frame_bytes=statistics.fmean(sizes))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default='192.168.26.2');p.add_argument('--ssh', required=True);p.add_argument('--out', required=True)
    p.add_argument('--pings', type=int, default=200);p.add_argument('--ssh-runs', type=int, default=40);p.add_argument('--megabytes', type=int, default=48)
    p.add_argument('--frame-url', default='http://127.0.0.1:18766/frame.jpg')
    a = p.parse_args()
    report = dict(host=a.host, started=time.strftime('%Y-%m-%d %H:%M:%S'), icmp=ping(a.host, a.pings, .2), ssh_command_ms=ssh_round_trip(a.ssh, a.ssh_runs),
                  throughput=throughput(a.ssh, a.megabytes), motor_commands_sent=0)
    try:
        report['wrist_frame_http'] = frame_fetch(a.frame_url, 40)
    except OSError as error:
        report['wrist_frame_http'] = dict(error=repr(error)[:120])
    json.dump(report, open(a.out, 'w'), indent=2)
    print(json.dumps(report, indent=2))
