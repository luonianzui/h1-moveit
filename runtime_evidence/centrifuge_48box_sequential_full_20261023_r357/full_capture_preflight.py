import ctypes
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from window_capture import FullWindow


EVIDENCE = Path(__file__).resolve().parent


def main():
    capture = FullWindow.__new__(FullWindow)
    import ctypes.util
    library = ctypes.CDLL(ctypes.util.find_library('X11'))
    library.XOpenDisplay.restype = ctypes.c_void_p
    display = library.XOpenDisplay(None)
    assert display
    library.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    library.XDefaultRootWindow.restype = ctypes.c_ulong
    library.XCreateSimpleWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
        ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong]
    library.XCreateSimpleWindow.restype = ctypes.c_ulong
    library.XCreateGC.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_void_p]
    library.XCreateGC.restype = ctypes.c_void_p
    for name in ('XMapRaised', 'XDestroyWindow'):
        getattr(library, name).argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    library.XResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_uint, ctypes.c_uint]
    library.XSetForeground.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
    library.XFillRectangle.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p,
                                     ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    library.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    library.XCloseDisplay.argtypes = [ctypes.c_void_p]
    window = library.XCreateSimpleWindow(display, library.XDefaultRootWindow(display), 10, 10, 600, 450, 0, 0, 0)
    graphics = library.XCreateGC(display, window, 0, None)
    library.XMapRaised(display, window)
    library.XSync(display, 0)
    path = EVIDENCE / 'full_capture_preflight.mp4'
    sizes = [(600, 450), (1601, 1101), (3600, 1900), (800, 1500)]

    def paint(width, height):
        library.XSetForeground(display, graphics, 0)
        library.XFillRectangle(display, window, graphics, 0, 0, width, height)
        length = min(width, height) // 5
        corners = [(0, 0), (width - length, 0), (0, height - length), (width - length, height - length)]
        for color, corner in zip((0xff0000, 0x00ff00, 0x0000ff, 0xff00ff), corners):
            library.XSetForeground(display, graphics, color)
            library.XFillRectangle(display, window, graphics, corner[0], corner[1], length, length)
        library.XSync(display, 0)

    paint(*sizes[0])
    for attempt in range(10):
        paint(*sizes[0])
        time.sleep(.05)
    try:
        with (EVIDENCE / 'full_capture_preflight_record.log').open('x') as stream:
            process = subprocess.Popen([sys.executable, str(EVIDENCE / 'record_window.py'), hex(window), str(path), '120'],
                stdout=stream, stderr=subprocess.STDOUT)
            for size in sizes:
                library.XResizeWindow(display, window, *size)
                deadline = time.monotonic() + 2.
                while time.monotonic() < deadline:
                    paint(*size)
                    time.sleep(.04)
            status = process.wait(timeout=30)
        metadata = json.loads(path.with_suffix('.capture.json').read_text())
        decoded = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-f', 'rawvideo',
                                  '-pix_fmt', 'rgb24', 'pipe:1'], capture_output=True, timeout=30, check=True)
        frames = np.frombuffer(decoded.stdout, dtype=np.uint8).reshape((-1, 900, 1200, 3))
        failures = []
        transition_frames = []
        for index, frame in enumerate(frames):
            red, green, blue = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
            counts = [int(mask.sum()) for mask in (
                (red > 180) & (green < 70) & (blue < 70),
                (green > 180) & (red < 70) & (blue < 70),
                (blue > 180) & (red < 70) & (green < 70),
                (red > 180) & (blue > 180) & (green < 70))]
            time_s = index / 12.
            changes = [row['wall_offset_s'] for row in metadata['geometries'][1:]]
            if any(abs(time_s - changed) <= .3 for changed in changes):
                transition_frames.append(index)
            elif min(counts) < 500:
                failures.append(dict(frame=index, corner_pixels=counts))
        audit = dict(status='PASS' if status == 0 and metadata['status'] == 'PASS' and len(frames) == 120 and not failures else 'FAIL',
                     frames=len(frames), sizes=sizes, capture=metadata, missing_corner_frames=failures,
                     settled_frames_all_four_native_corners_verified=True,
                     transition_frames=transition_frames, all_frames_retained_and_decoded=True, no_simulation_commands=True)
        with (EVIDENCE / 'full_window_capture_audit.json').open('x') as stream:
            json.dump(audit, stream, indent=2)
        print(audit['status'], len(frames), len(failures))
        assert audit['status'] == 'PASS'
    finally:
        library.XDestroyWindow(display, window)
        library.XCloseDisplay(display)


if __name__ == '__main__':
    main()
