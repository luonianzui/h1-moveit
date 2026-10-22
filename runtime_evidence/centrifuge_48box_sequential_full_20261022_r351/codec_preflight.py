import json
from pathlib import Path
import subprocess

from video_capture import capture_command, capture_environment


EVIDENCE = Path(__file__).resolve().parent
for name in ('ximagesrc', 'videoconvert', 'videoscale', 'x264enc', 'mp4mux', 'filesink'):
    response = subprocess.run(['gst-inspect-1.0', name], capture_output=True, text=True, timeout=10)
    with (EVIDENCE / ('codec_plugin_' + name + '.log')).open('x') as stream:
        stream.write(response.stdout + response.stderr)
    assert response.returncode == 0, name
path = EVIDENCE / 'codec_synthetic.mp4'
command = capture_command('0x1', path)
assert command[2:7] == ['ximagesrc', 'display-name=:0', 'xid=1', 'use-damage=false', 'show-pointer=false']
command[2:command.index('!')] = ['videotestsrc', 'num-buffers=72', 'pattern=ball']
with (EVIDENCE / 'codec_synthetic.log').open('x') as stream:
    completed = subprocess.run(command, env=capture_environment(), stdout=stream,
                               stderr=subprocess.STDOUT, timeout=30)
probe = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-show_streams',
                        '-of', 'json', str(path)], text=True, capture_output=True, timeout=30)
with (EVIDENCE / 'codec_decode.log').open('x') as stream:
    decode = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-f', 'null', '-'],
                            stdout=stream, stderr=subprocess.STDOUT, timeout=30)
metadata = json.loads(probe.stdout) if probe.returncode == 0 else {}
frames = int(metadata.get('streams', [{}])[0].get('nb_read_frames', 0))
result = dict(status='PASS' if completed.returncode == probe.returncode == decode.returncode == 0 and frames == 72 else 'FAIL',
              command=command, frames=frames, metadata=metadata, no_desktop_capture=True,
              full_decode_returncode=decode.returncode, parser_plugin_not_required=True)
with (EVIDENCE / 'codec_audit.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
assert result['status'] == 'PASS'
