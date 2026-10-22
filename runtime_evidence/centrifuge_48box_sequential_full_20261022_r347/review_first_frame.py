import json
from pathlib import Path
import subprocess
import time

from PIL import Image


ROOT = Path(__file__).resolve().parent
output = ROOT / 'run/startup_frame.png'
assert not output.exists()
attempts = []
deadline = time.monotonic() + 60.
while time.monotonic() < deadline:
    try:
        result = subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-n',
            '-ss', '0.5', '-i', str(ROOT / 'run/full_cycle.mp4'), '-frames:v', '1', str(output)],
            capture_output=True, text=True, timeout=5)
        attempts.append(dict(returncode=result.returncode, stderr=result.stderr[:500]))
    except subprocess.TimeoutExpired:
        attempts.append(dict(returncode=None, error='bounded_first_fragment_decode_timeout'))
    if output.is_file():
        with Image.open(output) as frame:
            frame.verify()
        with (ROOT / 'run/startup_frame_decode.json').open('x') as stream:
            json.dump(dict(status='PASS', attempts=attempts, source='full_cycle.mp4',
                           sample_wall_s=.5, first_real_recorded_frame_not_external_mockup=True), stream, indent=2)
        print('REAL_RECORDING_FIRST_FRAME_READY', flush=True)
        break
    time.sleep(1.)
else:
    raise TimeoutError('First real recording fragment did not decode within 60 wall seconds')
