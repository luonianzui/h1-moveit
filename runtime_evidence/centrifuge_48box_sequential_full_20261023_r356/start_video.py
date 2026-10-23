import json
from pathlib import Path
import subprocess
import signal
import time

import psutil
from video_capture import capture_command, capture_environment

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
assert json.loads((EVIDENCE / 'dense_audit_retry1.json').read_text())['status'] == 'PASS'
assert json.loads((EVIDENCE / 'protected_before.json').read_text())['status'] == 'PASS'
assert (EVIDENCE / 'run/launch_pid.json').is_file()
assert json.loads((EVIDENCE / 'run/controller_health.json').read_text())['status'] == 'PASS'
windows = subprocess.run(['xwininfo', '-root', '-tree'], text=True, capture_output=True, check=True)
matches = [line for line in windows.stdout.splitlines() if '"Gazebo Sim"' in line and '"gz-sim-gui"' in line]
assert len(matches) == 1, matches
window = matches[0].strip().split()[0]
geometry = subprocess.run(['xwininfo', '-id', window], capture_output=True, text=True, check=True)
import re
width = int(re.search(r'Width:\s+(\d+)', geometry.stdout).group(1))
height = int(re.search(r'Height:\s+(\d+)', geometry.stdout).group(1))
assert width >= 1200 and height >= 900, 'Gazebo window is smaller than the fixed capture region'
with (EVIDENCE / 'run/viewport.json').open('x') as stream:
    json.dump(dict(window_id=window, window_line=matches[0], native_width=width, native_height=height,
                   window_specific_region=[0, 0, 1199, 899],
                   fixed_capture_to_avoid_resize_caps_failure=True, entire_resized_window_not_claimed=True), stream, indent=2)
camera = 'pose: { position: { x: 0.03 y: -1.3 z: 1.85 } orientation: { x: -0.18319605196242109 y: 0.26830874567596436 z: 0.5332908834106234 w: 0.7810572688415937 } }'
with (EVIDENCE / 'run/camera_request.json').open('x') as stream:
    json.dump(dict(request=camera, intended='complete loading and centrifuge cycle'), stream, indent=2)
response = subprocess.run(['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', camera],
                          text=True, capture_output=True, check=True, timeout=8)
with (EVIDENCE / 'run/camera_response.log').open('x') as stream:
    stream.write(response.stdout + response.stderr)
assert 'data: true' in response.stdout
subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n', '-f', 'x11grab',
                '-window_id', window, '-i', ':0', '-frames:v', '1', str(EVIDENCE / 'run/start_scene.png')],
               check=True, timeout=10)
assert json.loads((EVIDENCE / 'run/recorder_smoke_audit.json').read_text())['status'] == 'PASS'
command = capture_command(window, EVIDENCE / 'run/full_cycle.mp4')
with (EVIDENCE / 'run/video.log').open('x') as stream:
    process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                               stderr=subprocess.STDOUT, env=capture_environment(), start_new_session=True)
with (EVIDENCE / 'run/video_pid.json').open('x') as stream:
    json.dump(dict(pid=process.pid, created=psutil.Process(process.pid).create_time(), command=command),
              stream, indent=2)
deadline = time.monotonic() + 15
path = EVIDENCE / 'run/full_cycle.mp4'
while not (path.is_file() and path.stat().st_size > 0):
    if process.poll() is not None:
        raise RuntimeError('Video recorder exited during startup')
    if time.monotonic() > deadline:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        raise TimeoutError('Video file startup timeout')
    time.sleep(.05)
print(process.pid)
