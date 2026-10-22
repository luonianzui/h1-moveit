import json
from pathlib import Path
import subprocess
import signal
import time
import sys

import psutil
from video_capture import capture_command, capture_environment

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
assert json.loads((EVIDENCE / 'paired_fixture_actual_opening_audit.json').read_text())['status'] == 'PASS'
assert json.loads((EVIDENCE / 'buttons_collision_audit.json').read_text())['status'] == 'PASS'
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
                   entire_native_window_each_capture=True, output_dimensions=[1200, 900],
                   scale_and_letterbox_no_fixed_crop=True), stream, indent=2)
import math
from scipy.spatial.transform import Rotation
position = [1.65, -1.55, 1.95]
target = [0.25, 0.0, 0.95]
yaw = math.atan2(target[1] - position[1], target[0] - position[0])
pitch = math.atan2(position[2] - target[2], math.hypot(target[0] - position[0], target[1] - position[1]))
quaternion = Rotation.from_euler('xyz', [0., pitch, yaw]).as_quat()
camera = f'pose: {{ position: {{ x: {position[0]} y: {position[1]} z: {position[2]} }} orientation: {{ x: {quaternion[0]} y: {quaternion[1]} z: {quaternion[2]} w: {quaternion[3]} }} }}'
with (EVIDENCE / 'run/camera_request.json').open('x') as stream:
    json.dump(dict(request=camera, intended='robot front upper view of complete cycle'), stream, indent=2)
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
assert json.loads((EVIDENCE / 'full_window_capture_audit.json').read_text())['status'] == 'PASS'
command = [sys.executable, str(EVIDENCE / 'record_window.py'), window, str(EVIDENCE / 'run/full_cycle.mp4')]
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
