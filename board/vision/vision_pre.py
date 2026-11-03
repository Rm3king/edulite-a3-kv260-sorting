"""Board driver of the PL wrist-camera pre-processing kernel, with the same computation on the ARM as the fallback and baseline.

    python3 vision_pre.py selftest FRAME.raw ...   # PL vs ARM masks must be identical; prints timings
    python3 vision_pre.py bench FRAME.raw          # PL / ARM latency distribution on one frame

The kernel reads a 640x400 grey frame from DDR and writes a packed 640x400 mask (finger boxes filled, blur, Otsu level over
the region of interest or a given level, dark or bright side, 7x7 close). The frame buffer comes from the CMA dma-heap through
the setcap helper kv260-vision-alloc; the HP port is not coherent, so the ARM cleans the frame and invalidates the mask
around each run (vision_cache.c). Nothing here touches the CAN engine or the motion registers.
"""
import ctypes
import mmap
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

W, H = 640, 400
IMG_BYTES, MASK_BYTES = W * H, W * H // 8
MASK_OFFSET = 256 * 1024
BUFFER_BYTES = 512 * 1024
FINGER_BOXES = ((0, 0, 200, 100), (360, 0, 560, 95))
HERE = Path(__file__).resolve().parent
ALLOC = '/usr/local/libexec/kv260-vision-alloc'
VISION_OFFSET = 0x20000          # inside the kv260-can UIO window
REG = dict(ctrl=0x00, img=0x10, mask=0x1c, roi_x=0x28, roi_y=0x30, roi_w=0x38, roi_h=0x40,
           box0_x0=0x48, box0_y0=0x50, box0_x1=0x58, box0_y1=0x60, box1_x0=0x68, box1_y0=0x70, box1_x1=0x78, box1_y1=0x80,
           fill=0x88, cap=0x90, level_mode=0x98, manual_level=0xa0, polarity=0xa8, close_en=0xb0,
           level=0xb8, otsu_level=0xc8, count=0xd8, status=0xe8)


def _library(name, sources):
    """Build (once) and load a small C helper next to this file."""
    so = HERE / name
    if not so.exists() or any(so.stat().st_mtime < (HERE / s).stat().st_mtime for s in sources):
        subprocess.run(['gcc', '-O3', '-Wall', '-shared', '-fPIC', '-o', str(so)] + [str(HERE / s) for s in sources], check=True)
    return ctypes.CDLL(str(so))


class Params:
    def __init__(self, roi=(0, 0, W, H), boxes=FINGER_BOXES, fill=200, cap=60, level_mode=0, manual_level=0, polarity=0, close=1):
        self.roi, self.boxes, self.fill, self.cap = tuple(map(int, roi)), boxes, fill, cap
        self.level_mode, self.manual_level, self.polarity, self.close = level_mode, manual_level, polarity, close


def roi_around(expected, radius=220):
    """The region of interest locate_screw_near uses: `radius` around the expected position, clipped to the frame."""
    ex, ey = expected
    x0, y0 = max(0, int(ex - radius)), max(0, int(ey - radius))
    x1, y1 = min(W, int(ex + radius)), min(H, int(ey + radius))
    return (x0, y0, x1 - x0, y1 - y0)


class VisionARM:
    """The kernel's computation on one ARM core (vision_ref.c)."""
    name = 'arm'

    def __init__(self):
        self.lib = _library('libvision_ref.so', ['vision_ref.c'])
        self.lib.vision_ref.restype = ctypes.c_int
        self.work = np.zeros(7 * W * H, np.uint8)
        self.mask = np.zeros(MASK_BYTES, np.uint8)

    def run(self, gray, p):
        gray = np.ascontiguousarray(gray, np.uint8)
        assert gray.shape == (H, W)
        box = (ctypes.c_int * 8)(*[v for b in p.boxes for v in b])
        level, ot, count = ctypes.c_int(), ctypes.c_int(), ctypes.c_uint32()
        t = time.perf_counter()
        status = self.lib.vision_ref(gray.ctypes.data_as(ctypes.c_void_p), self.mask.ctypes.data_as(ctypes.c_void_p), *p.roi, box,
                                     p.fill, p.cap, p.level_mode, p.manual_level, p.polarity, p.close,
                                     self.work.ctypes.data_as(ctypes.c_void_p), ctypes.byref(level), ctypes.byref(ot), ctypes.byref(count))
        elapsed = time.perf_counter() - t
        if status:
            raise ValueError('region of interest smaller than 12x12 or outside the frame')
        return dict(mask=self.mask.copy(), level=level.value, otsu_level=ot.value, count=count.value, seconds=elapsed)


def _uio():
    for d in Path('/sys/class/uio').glob('uio*'):
        if (d / 'name').read_text().strip() == 'kv260-can':
            return '/dev/' + d.name, int((d / 'maps/map0/size').read_text(), 16)
    raise RuntimeError('kv260-can UIO device not found; load the CAN firmware first')


class VisionPL:
    """The PL kernel. Raises RuntimeError when the loaded bitstream has no vision kernel (UIO window < 192 KiB)."""
    name = 'pl'

    def __init__(self):
        dev, size = _uio()
        if size < VISION_OFFSET + 0x10000:
            raise RuntimeError('loaded firmware has no vision kernel (UIO window %#x)' % size)
        self.cache = _library('libvision_cache.so', ['vision_cache.c'])
        self.cache.vp_clean.argtypes = self.cache.vp_invalidate.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        self.fd = os.open(dev, os.O_RDWR | os.O_SYNC)
        self.regs = mmap.mmap(self.fd, VISION_OFFSET + 0x10000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        if not self.read('ctrl') & 0x4:
            raise RuntimeError('vision kernel not idle after load (ap_ctrl %#x)' % self.read('ctrl'))
        a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            child = subprocess.run([ALLOC, str(BUFFER_BYTES), str(b.fileno())], pass_fds=(b.fileno(),), stdin=subprocess.DEVNULL,
                                   capture_output=True, text=True, timeout=10)
            if child.returncode:
                raise RuntimeError('kv260-vision-alloc failed: ' + child.stderr.strip())
            text, fds, _, _ = socket.recv_fds(a, 64, 1)
        finally:
            a.close(); b.close()
        phys, size = text.decode().split()
        self.phys, self.buf_fd = int(phys, 16), fds[0]
        self.buf = mmap.mmap(self.buf_fd, int(size), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.view = np.frombuffer(self.buf, np.uint8)
        self.addr = ctypes.addressof(ctypes.c_char.from_buffer(self.buf))

    def read(self, name):
        return struct.unpack_from('<I', self.regs, VISION_OFFSET + REG[name])[0]

    def write(self, name, value):
        struct.pack_into('<I', self.regs, VISION_OFFSET + REG[name], int(value) & 0xffffffff)

    def run(self, gray, p, timeout=0.2, poll_sleep=0.):
        """poll_sleep > 0: wait for the kernel by sleeping that long between polls, leaving the core free for other work."""
        gray = np.ascontiguousarray(gray, np.uint8)
        assert gray.shape == (H, W)
        t = time.perf_counter()
        self.view[:IMG_BYTES] = gray.ravel()
        self.cache.vp_clean(self.addr, IMG_BYTES)
        self.cache.vp_invalidate(self.addr + MASK_OFFSET, MASK_BYTES)
        img, mask = self.phys, self.phys + MASK_OFFSET
        self.write('img', img & 0xffffffff); struct.pack_into('<I', self.regs, VISION_OFFSET + REG['img'] + 4, img >> 32)
        self.write('mask', mask & 0xffffffff); struct.pack_into('<I', self.regs, VISION_OFFSET + REG['mask'] + 4, mask >> 32)
        for name, value in zip(('roi_x', 'roi_y', 'roi_w', 'roi_h'), p.roi):
            self.write(name, value)
        for i, box in enumerate(p.boxes):
            for name, value in zip(('x0', 'y0', 'x1', 'y1'), box):
                self.write(f'box{i}_{name}', value)
        for name in ('fill', 'cap', 'level_mode', 'manual_level', 'polarity'):
            self.write(name, getattr(p, name))
        self.write('close_en', p.close)
        t_start = time.perf_counter()
        self.write('ctrl', 1)
        deadline = t_start + timeout
        while not self.read('ctrl') & 0x2:
            if time.perf_counter() > deadline:
                raise TimeoutError('vision kernel did not finish')
            if poll_sleep:
                time.sleep(poll_sleep)
        t_done = time.perf_counter()
        self.cache.vp_invalidate(self.addr + MASK_OFFSET, MASK_BYTES)
        if self.read('status') & 0xff:
            raise ValueError('region of interest smaller than 12x12 or outside the frame')
        out = self.view[MASK_OFFSET:MASK_OFFSET + MASK_BYTES].copy()
        t_end = time.perf_counter()
        return dict(mask=out, level=self.read('level') & 0xff, otsu_level=self.read('otsu_level') & 0xff, count=self.read('count'),
                    seconds=t_end - t, kernel_seconds=t_done - t_start)


def open_best():
    """The PL kernel when the loaded firmware has it and the allocator works, otherwise the ARM computation."""
    try:
        return VisionPL()
    except (RuntimeError, OSError, FileNotFoundError, ValueError) as error:
        print('vision: PL kernel unavailable (%s); using the ARM computation' % error, file=sys.stderr)
        return VisionARM()


def unpack(mask):
    return np.unpackbits(mask.reshape(H, W // 8), axis=1, bitorder='little').astype(bool)


def _frames(paths):
    for path in paths:
        yield path, np.fromfile(path, np.uint8).reshape(H, W)


if __name__ == '__main__':
    import json
    command, paths = sys.argv[1], sys.argv[2:]
    arm = VisionARM()
    pl = VisionPL()
    if command == 'selftest':
        cases = [Params(roi=roi_around((320, 200))), Params(roi=roi_around((200, 250)), cap=255, polarity=1),
                 Params(level_mode=1, manual_level=100, polarity=1, close=0), Params(roi=(37, 21, 13, 12), cap=255)]
        failures = 0
        for path, gray in _frames(paths):
            for i, p in enumerate(cases):
                a, b = arm.run(gray, p), pl.run(gray, p)
                same = a['level'] == b['level'] and a['count'] == b['count'] and np.array_equal(a['mask'], b['mask'])
                failures += not same
                print(json.dumps(dict(frame=path, case=i, same=same, level=b['level'], count=b['count'],
                                      arm_ms=round(a['seconds'] * 1e3, 2), pl_ms=round(b['seconds'] * 1e3, 2),
                                      pl_kernel_ms=round(b['kernel_seconds'] * 1e3, 3))))
        print('SELFTEST', 'PASS' if failures == 0 else 'FAIL', failures)
    elif command == 'bench':
        gray = np.fromfile(paths[0], np.uint8).reshape(H, W)
        p = Params(roi=roi_around((320, 200)))
        n = int(os.environ.get('VP_REPEAT', '200'))
        engines = [('arm_c_O3_one_core', lambda: arm.run(gray, p)), ('pl_busy_poll', lambda: pl.run(gray, p)),
                   ('pl_sleep_poll_0.5ms', lambda: pl.run(gray, p, poll_sleep=0.0005))]
        rows = {}
        for name, call in engines:
            wall, cpu0, t0 = [], time.process_time(), time.perf_counter()
            for _ in range(n):
                t = time.perf_counter()
                call()
                wall.append((time.perf_counter() - t) * 1e3)
            total = time.perf_counter() - t0
            q = np.percentile(wall, [50, 95, 99, 100])
            rows[name] = dict(frames=n, p50_ms=round(q[0], 2), p95_ms=round(q[1], 2), p99_ms=round(q[2], 2), max_ms=round(q[3], 2),
                              fps=round(n / total, 1), cpu_ms_per_frame=round((time.process_time() - cpu0) * 1e3 / n, 2),
                              cpu_fraction_of_one_core=round((time.process_time() - cpu0) / total, 3))
        print(json.dumps(rows, indent=1))
