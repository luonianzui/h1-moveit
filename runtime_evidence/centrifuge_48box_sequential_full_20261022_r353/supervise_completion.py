import json
from pathlib import Path
import subprocess
import sys
import time

from camera_guard import restore_with_retries
from terminal_completion import finish_before_retirement
from video_capture import recorder_alive

EVIDENCE = Path(__file__).resolve().parent
deadline = time.monotonic() + 7200
result_path = EVIDENCE / 'run/result.json'
result = None
while time.monotonic() < deadline:
    if result_path.exists():
        try:
            result = json.loads(result_path.read_text())
            break
        except json.JSONDecodeError:
            pass
    time.sleep(1)
if result is None:
    raise TimeoutError('No completed result; preserve scene without reset or release')
print('COMPLETION', result['status'], flush=True)
if result['status'] == 'FAIL':
    machine = result['final_machine']
    assert machine['ready'] and not machine['rotor_enabled'] and abs(machine['rotor_rpm']) <= .02
    with (EVIDENCE / 'pause_supervisor.log').open('x') as stream:
        subprocess.run([sys.executable, str(EVIDENCE / 'pause_after_result.py')],
                       stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=30)
else:
    assert result['status'] == 'PASS' and result['holding'] is None
    window = json.loads((EVIDENCE / 'run/viewport.json').read_text())['window_id']
def inspect_terminal():
    if result['status'] == 'PASS':
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n', '-f', 'x11grab',
                        '-window_id', window, '-i', ':0', '-frames:v', '1',
                        str(EVIDENCE / 'run/final_scene.png')], check=True, timeout=10)
    if result['status'] == 'PASS':
        camera = 'pose: { position: { x: 0.32 y: 0 z: 2.5 } orientation: { x: 0 y: 0.6386638956379793 z: 0 w: 0.7694858136051279 } }'
        accepted, attempts, error = restore_with_retries(camera, 'terminal_overhead')
        with (EVIDENCE / 'run/final_overhead_camera.json').open('x') as stream:
            json.dump(dict(request=camera, attempts=attempts, accepted=accepted, error=error,
                before_video_sigint=True, terminal_inspection_only=True, no_robot_command=True), stream, indent=2)
        if not accepted:
            raise RuntimeError(error)
        time.sleep(2)
        subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-n', '-f', 'x11grab',
            '-window_id', window, '-i', ':0', '-frames:v', '1', str(EVIDENCE / 'run/final_overhead.png')],
            check=True, timeout=10)
        time.sleep(1)


def finish_video():
    with (EVIDENCE / 'video_finish.log').open('x') as stream:
        subprocess.run([sys.executable, str(EVIDENCE / 'finish_video.py')],
                       stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=420)


def retire_scene():
    with (EVIDENCE / 'retire.log').open('x') as stream:
        subprocess.run([sys.executable, str(EVIDENCE / 'retire_scene.py')],
                       stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=45)


identity = json.loads((EVIDENCE / 'run/video_pid.json').read_text())
outcomes = finish_before_retirement(inspect_terminal, finish_video, retire_scene,
                                    lambda: recorder_alive(identity))
with (EVIDENCE / 'run/completion_finalization.json').open('x') as stream:
    json.dump(outcomes, stream, indent=2)
print('FINALIZATION', outcomes, flush=True)
assert outcomes['status'] == 'PASS', outcomes
print('RETIRED_AND_VIDEO_DONE', flush=True)
