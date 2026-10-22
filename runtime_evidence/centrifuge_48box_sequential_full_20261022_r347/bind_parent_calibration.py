import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_front_camera_full_20261015_r284'
manifest = {relative: checksum for checksum, relative in
            (line.split('  ', 1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
content = (PARENT / 'parent_servo_calibration_audit.json').read_bytes()
assert hashlib.sha256(content).hexdigest() == manifest['parent_servo_calibration_audit.json']
certificate = json.loads(content)
source = Path(certificate['parent']) / 'calibration_audit.json'
assert hashlib.sha256(source.read_bytes()).hexdigest() == certificate['source_calibration_audit_sha256']
assert json.loads(source.read_text()) == certificate['physical_certificate']
assert certificate['only_small_empty_arm_ramp_certified'] and certificate['loaded_motion_not_certified']
with (ROOT / 'parent_servo_calibration_audit.json').open('xb') as stream:
    stream.write(content)
print('BOUND_OLD_EMPTY_SERVO_CALIBRATION_NOT_NEW_PHYSICS', flush=True)
