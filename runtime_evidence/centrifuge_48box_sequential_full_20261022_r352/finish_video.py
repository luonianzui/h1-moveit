import json
from pathlib import Path
import signal
import subprocess

import psutil
from video_capture import recorder_alive

EVIDENCE = Path(__file__).resolve().parent
identity = json.loads((EVIDENCE / 'run/video_pid.json').read_text())
normal_sigint = False
if recorder_alive(identity):
    process = psutil.Process(identity['pid'])
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=60)
        normal_sigint = True
    except psutil.TimeoutExpired:
        raise RuntimeError('Video recorder did not finish normally; no forced kill')
path = EVIDENCE / 'run/full_cycle.mp4'
probe = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-show_streams', '-show_format',
                        '-of', 'json', str(path)], text=True, capture_output=True, timeout=120)
with (EVIDENCE / 'run/video_probe.json').open('x') as stream:
    stream.write(probe.stdout)
with (EVIDENCE / 'run/video_probe_error.log').open('x') as stream:
    stream.write(probe.stderr)
with (EVIDENCE / 'run/video_decode.log').open('x') as stream:
    decoded = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-f', 'null', '-'],
                             stdout=stream, stderr=subprocess.STDOUT, timeout=300)
audit = dict(status='PASS' if normal_sigint and probe.returncode == decoded.returncode == 0 and
             (EVIDENCE / 'run/video_decode.log').stat().st_size == 0 else 'FAIL',
             full_decode_returncode=decoded.returncode, probe_returncode=probe.returncode,
             path=str(path), size_bytes=path.stat().st_size if path.exists() else 0,
             metadata=json.loads(probe.stdout) if probe.returncode == 0 else {},
             normal_sigint=normal_sigint, specific_window_only=True)
with (EVIDENCE / 'video_audit.json').open('x') as stream:
    json.dump(audit, stream, indent=2)
print(audit['status'], audit['size_bytes'])
assert audit['status'] == 'PASS'
