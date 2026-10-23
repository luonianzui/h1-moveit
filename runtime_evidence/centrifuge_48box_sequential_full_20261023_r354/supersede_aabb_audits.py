import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
assert not (ROOT / 'identity.json').exists() and not (ROOT / 'run').exists()
destination = ROOT / 'superseded_unsealed_aabb_audits'
names = ['calibrated_gain_audit.json', 'control_fork_audit.json', 'dual_layout_audit.json',
         'gazebo_dual.launch.py.acceleration_audit.json', 'inherited_inputs_audit.json',
         'inherited_inputs_runtime_scope_erratum.json', 'private_lid_rate_fork_audit.json',
         'robot_dual.urdf.acceleration_audit.json', 'servo_fork_audit.json', 'startup_ramp_model_audit.json']
records = []
for name in names:
    path = ROOT / name
    assert path.is_file() and not path.is_symlink()
    records.append(dict(original=name, retained='superseded_unsealed_aabb_audits/' + name,
                        bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
destination.mkdir()
for row in records:
    (ROOT / row['original']).rename(ROOT / row['retained'])
    assert hashlib.sha256((ROOT / row['retained']).read_bytes()).hexdigest() == row['sha256']
with (ROOT / 'superseded_aabb_audit_ledger.json').open('x') as stream:
    json.dump(dict(reason='Unsealed AABB-only candidate incompatible; retain all prior input audits before ground-trimmed OBB qualification',
                   no_formal_run_or_old_success_modified=True, no_file_deleted=True, records=records), stream, indent=2)
