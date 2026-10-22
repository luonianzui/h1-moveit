import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from window_capture import FullWindow, fit_frame
from camera_pose_guard import CameraGuard


def main():
    window, output = sys.argv[1:3]
    path = Path(output)
    assert not path.exists()
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else None
    requested_stop = False

    def stop(signum, frame):
        nonlocal requested_stop
        requested_stop = True

    signal.signal(signal.SIGINT, stop)
    capture = FullWindow(window)
    camera = CameraGuard(path.parent) if limit is None and (path.parent / 'camera_request.json').is_file() else None
    if camera is not None:
        camera.start()
    command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n', '-f', 'rawvideo',
               '-pixel_format', 'rgb24', '-video_size', '1200x900', '-framerate', '12', '-i', 'pipe:0',
               '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21', '-g', '24',
               '-pix_fmt', 'yuv420p',
               '-movflags', 'frag_keyframe+empty_moov+default_base_moof', str(path)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    start = time.monotonic()
    emitted = 0
    captures = 0
    duplicate_frames = 0
    maximum_gap = 0.
    previous = start
    geometries = []
    error = None
    try:
        while not requested_stop and (limit is None or emitted < limit):
            if camera is not None and camera.fault is not None:
                raise RuntimeError(camera.fault)
            now = time.monotonic()
            wait = start + emitted / 12 - now
            if wait > 0:
                time.sleep(wait)
            source = capture.read()
            frame, geometry = fit_frame(source)
            now = time.monotonic()
            maximum_gap = max(maximum_gap, now - previous)
            previous = now
            if maximum_gap > 2.:
                raise RuntimeError('Recorder full-window sampling gap exceeds 2 wall seconds')
            if not geometries or geometries[-1]['geometry'] != geometry:
                geometries.append(dict(wall_offset_s=now - start, geometry=geometry))
            needed = max(1, int((now - start) * 12) + 1 - emitted)
            if limit is not None:
                needed = min(needed, limit - emitted)
            content = frame.tobytes()
            for repeat in range(needed):
                process.stdin.write(content)
            emitted += needed
            duplicate_frames += needed - 1
            captures += 1
    except Exception as exception:
        error = repr(exception)
    finally:
        if camera is not None:
            camera.close()
        capture.close()
        process.stdin.close()
        status = process.wait(timeout=60)
        audit = dict(status='PASS' if error is None and status == 0 else 'FAIL', error=error,
            normal_sigint=requested_stop, test_frame_limit=limit, frames=emitted, unique_captures=captures,
            duplicate_frames=duplicate_frames, maximum_capture_gap_wall_s=maximum_gap,
            elapsed_wall_s=time.monotonic() - start, geometries=geometries,
            entire_native_window_each_capture=True, no_fixed_upper_left_crop=True, encoder_command=command)
        audit['wide_camera_guard_enabled'] = camera is not None
        audit['wide_camera_guard_status'] = None if camera is None else ('PASS' if camera.fault is None else 'FAIL')
        if camera is not None and camera.fault is not None:
            audit['status'], audit['error'] = 'FAIL', camera.fault
        with path.with_suffix('.capture.json').open('x') as stream:
            json.dump(audit, stream, indent=2)
    assert audit['status'] == 'PASS', audit


if __name__ == '__main__':
    main()
