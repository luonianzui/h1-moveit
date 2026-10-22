import ctypes
import ctypes.util
import json
from pathlib import Path
import subprocess
import time

from video_capture import capture_command, capture_environment


EVIDENCE = Path(__file__).resolve().parent


def main():
    library = ctypes.CDLL(ctypes.util.find_library('X11'))
    signatures = {
        'XOpenDisplay':([ctypes.c_char_p], ctypes.c_void_p),
        'XDefaultRootWindow':([ctypes.c_void_p], ctypes.c_ulong),
        'XCreateSimpleWindow':([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
                                ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong], ctypes.c_ulong),
        'XStoreName':([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_char_p], ctypes.c_int),
        'XMapWindow':([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
        'XResizeWindow':([ctypes.c_void_p, ctypes.c_ulong, ctypes.c_uint, ctypes.c_uint], ctypes.c_int),
        'XSync':([ctypes.c_void_p, ctypes.c_int], ctypes.c_int),
        'XDestroyWindow':([ctypes.c_void_p, ctypes.c_ulong], ctypes.c_int),
        'XCloseDisplay':([ctypes.c_void_p], ctypes.c_int),
    }
    for name, (arguments, result) in signatures.items():
        function = getattr(library, name)
        function.argtypes, function.restype = arguments, result
    display = library.XOpenDisplay(b':0')
    assert display
    window = library.XCreateSimpleWindow(display, library.XDefaultRootWindow(display),
                                         100, 100, 1400, 1050, 0, 0, 0x335577)
    assert window
    library.XStoreName(display, window, b'r219 recorder resize preflight')
    library.XMapWindow(display, window)
    library.XSync(display, 0)
    time.sleep(1)
    path = EVIDENCE / 'resize_preflight.mp4'
    command = capture_command(hex(window), path, frames=120)
    sizes = [(1601, 1101), (1200, 900), (1800, 1200), (1400, 1050)]
    try:
        with (EVIDENCE / 'resize_preflight.log').open('x') as stream:
            process = subprocess.Popen(command, env=capture_environment(), stdout=stream, stderr=subprocess.STDOUT)
            for width, height in sizes:
                time.sleep(1)
                assert process.poll() is None
                library.XResizeWindow(display, window, width, height)
                library.XSync(display, 0)
            returncode = process.wait(timeout=30)
        probe = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-show_streams',
                                '-of', 'json', str(path)], capture_output=True, text=True, timeout=30)
        with (EVIDENCE / 'resize_decode.log').open('x') as stream:
            decode = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-f', 'null', '-'],
                                    stdout=stream, stderr=subprocess.STDOUT, timeout=30)
        metadata = json.loads(probe.stdout) if probe.returncode == 0 else {}
        frames = int(metadata.get('streams', [{}])[0].get('nb_read_frames', 0))
        audit = dict(status='PASS' if returncode == probe.returncode == decode.returncode == 0 and frames == 120 else 'FAIL',
                     command=command, sizes=sizes, frames=frames, metadata=metadata,
                     window_specific_fixed_region=True, no_simulation_or_robot_commands=True)
        with (EVIDENCE / 'resize_capture_audit.json').open('x') as stream:
            json.dump(audit, stream, indent=2)
        print(json.dumps(audit))
        assert audit['status'] == 'PASS'
    finally:
        library.XDestroyWindow(display, window)
        library.XCloseDisplay(display)


if __name__ == '__main__':
    main()
