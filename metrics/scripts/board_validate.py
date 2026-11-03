#!/usr/bin/python3
"""Internal-only hardware validation; external CAN TX stays inhibited."""
import json
import random
import time
from canctl import CanDevice


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    dev = CanDevice()
    report = {'scope': 'INTERNAL_ONLY_EXTERNAL_TX_INHIBITED', 'physical_can': 'NOT_TESTED'}
    try:
        report['smoke'] = dev.selftest()
        dev.configure('internal')
        start = dev.status()
        rng = random.Random(260)
        for i in range(500):
            dev.heartbeat()
            ident = rng.randrange(1 << 29)
            payload = bytes(rng.randrange(256) for _ in range(i % 9))
            dev.enqueue(ident, payload)
            deadline = time.monotonic() + .05
            while time.monotonic() < deadline:
                dev.heartbeat()
                frame = dev.receive()
                if frame is not None:
                    require(frame['id'] == ident and frame['data'] == payload.hex()
                            and frame['extended'] and not frame['rtr'], f'Frame {i}: {frame}')
                    break
                time.sleep(.0002)
            else:
                raise TimeoutError(f'Frame {i}: {dev.status()}')
        finish = dev.status()
        require(finish['submitted'] - start['submitted'] == 500, 'Duplicate or missing submissions')
        require(finish['received'] - start['received'] == 500, 'Duplicate or missing receptions')
        require(finish['rx_overflow'] == start['rx_overflow'], 'RX overflow')
        require(finish['queue_counts'] == 0 and finish['fault'] == 0, 'Not clean after stress')
        report['random_frames'] = {'status': 'PASS', 'frames': 500, 'dlc_range': [0, 8], 'seed': 260}

        # Fill the actual hardware TX FIFO while deadlines are 30 ms away.
        dev.heartbeat()
        baseline = dev.status()
        due = (dev.read(0x408) + 30000) & 0xffffffff
        for i in range(16):
            dev.enqueue(0x1abc000 + i, bytes([i]) * 8, due_us=due, ttl_us=20000)
        require(dev.read(0x41c) & 0xffff == 16, 'TX FIFO did not fill')
        try:
            dev.enqueue(0x123, b'x', due_us=due, ttl_us=20000)
        except BufferError:
            pass
        else:
            raise AssertionError('Full queue not rejected by host')
        received = []
        deadline = time.monotonic() + .1
        while len(received) < 16 and time.monotonic() < deadline:
            dev.heartbeat()
            frame = dev.receive()
            if frame is not None:
                i = len(received)
                require(frame['id'] == 0x1abc000+i and frame['data'] == (bytes([i])*8).hex(), 'FIFO order/data')
                received.append(frame)
            else:
                time.sleep(.0002)
        require(len(received) == 16, 'FIFO drain timed out')
        require(dev.read(0x420)-baseline['submitted'] == 16, 'FIFO submissions mismatch')
        require(dev.read(0x42c) == baseline['rx_overflow'], 'FIFO burst overflow')
        report['full_tx_fifo'] = {'status': 'PASS', 'frames': 16, 'host_rejects_17th': True}

        # Expire in staging before commit. No CAN command should be submitted.
        baseline = dev.status()
        now = dev.read(0x408)
        for off, value in zip(range(0x440, 0x458, 4),
                              ((now+1000)&0xffffffff, (now+2000)&0xffffffff, 0x123, 0, 0, 0)):
            dev.write(off, value)
        time.sleep(.005)
        dev.heartbeat();dev.write(0x458, 1);time.sleep(.005)
        require(dev.read(0x424) == baseline['expired']+1, 'Expired frame not counted')
        require(dev.read(0x420) == baseline['submitted'], 'Expired frame submitted')
        report['expired_before_submit'] = 'PASS'

        # Independent hardware clock progresses while software sleeps.
        start_host = time.monotonic_ns();start_hw = dev.read(0x408)
        time.sleep(.02)
        elapsed_hw = (dev.read(0x408)-start_hw) & 0xffffffff
        elapsed_host = (time.monotonic_ns()-start_host)/1000
        require(.95 < elapsed_hw/elapsed_host < 1.05, 'Unexpected hardware timer rate')
        report['timer_sanity'] = {'hardware_us': elapsed_hw, 'host_us': elapsed_host,
                                  'not_external_bitrate_measurement': True}

        dev.heartbeat();dev.write(0x414, 2000)
        time.sleep(.01)
        status = dev.status()
        require(status['control'] == 0 and status['fault'] == 1 and status['queue_counts'] == 0,
                'Heartbeat timeout did not disarm and clear queues')
        report['heartbeat_timeout'] = 'PASS'
        dev.configure('internal');dev.stop()
        report['final'] = dev.status()
        require(report['final']['control'] == 0 and report['final']['fault'] == 0, 'Final not disabled/clean')
        report['status'] = 'PASS'
    finally:
        dev.stop();dev.close()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
