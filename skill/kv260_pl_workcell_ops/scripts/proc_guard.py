"""No orphaned host services between runs.

2026-10-30: the trial process was stopped while its wrist_servo_host services (ports 18768+) and their planning workers kept
running; the next run's stop-and-look servo could not bind its port ('Address already in use') and the M6 screw returned
unpicked. Two guarantees:

* every wrist_servo_host runs in its own process group and ends that whole group (itself and its workers) as soon as the
  process that started it is gone (`exit_with_parent`); the trial ends services by group (`end_group`);
* a run refuses to start while any port it needs is taken (`require_free_ports`), naming the holders.
"""
import os
import signal
import socket
import subprocess
import threading
import time


def exit_with_parent(period=.5):
    """In a service: end this process group when the parent process exits (it is then re-parented)."""
    parent = os.getppid()

    def watch():
        while True:
            time.sleep(period)
            if os.getppid() != parent:
                try:
                    os.killpg(os.getpgid(0), signal.SIGTERM)
                finally:
                    os._exit(1)
    threading.Thread(target=watch, daemon=True).start()


def end_group(process, timeout=3.):
    """In the trial: end a service started with start_new_session=True together with all its children."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=timeout)


def holders(port):
    """'pid=... name' strings of the processes listening on `port` (from ss), for the error message."""
    try:
        out = subprocess.run(['ss', '-ltnpH', 'sport = :%d' % port], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line.split('users:')[-1].strip() for line in out.splitlines() if line.strip()]


def require_free_ports(host, ports):
    """Refuse to start a run while a port it needs is taken (a leftover service would make that part fail mid-run)."""
    busy = []
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind((host, port))
        except OSError:
            busy.append((port, holders(port)))
        finally:
            s.close()
    if busy:
        raise RuntimeError('Ports still in use by an earlier run, stop those processes first: ' +
                           '; '.join('%d %s' % (port, ' '.join(h) or '?') for port, h in busy))
