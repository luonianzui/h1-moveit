import json
from pathlib import Path
import subprocess

from video_capture import capture_command, capture_environment


EVIDENCE = Path(__file__).resolve().parent
windows = subprocess.run(['xwininfo', '-root', '-tree'], text=True, capture_output=True, check=True)
matches = [line for line in windows.stdout.splitlines() if '"Gazebo Sim"' in line and '"gz-sim-gui"' in line]
assert len(matches) == 1
window = matches[0].strip().split()[0]
path = EVIDENCE / 'run/recorder_smoke.mp4'
command = capture_command(window, path, frames=72)
with (EVIDENCE / 'run/recorder_smoke.log').open('x') as stream:
    completed = subprocess.run(command, env=capture_environment(), stdout=stream,
                               stderr=subprocess.STDOUT, timeout=30)
probe = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-show_streams',
                        '-of', 'json', str(path)], text=True, capture_output=True, timeout=30)
with (EVIDENCE / 'run/recorder_smoke_decode.log').open('x') as stream:
    decode = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-f', 'null', '-'],
                            stdout=stream, stderr=subprocess.STDOUT, timeout=30)
metadata = json.loads(probe.stdout) if probe.returncode == 0 else {}
frames = int(metadata.get('streams', [{}])[0].get('nb_read_frames', 0))
result = dict(status='PASS' if completed.returncode == probe.returncode == decode.returncode == 0 and frames == 72 else 'FAIL',
              command=command, frames=frames, window_id=window, metadata=metadata,
              full_decode_returncode=decode.returncode, window_only=True)
with (EVIDENCE / 'run/recorder_smoke_audit.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
assert result['status'] == 'PASS'
