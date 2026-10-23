import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
baseline = Path('/home/liu/h1_moveit/backups/backup_1009_release_20261009/archive_payload_sha256.json')
records = json.loads(baseline.read_text())
checked = 0
failures = []
for relative, expected in records.items():
    if not relative.startswith('src/'):
        continue
    checked += 1
    path = WORKSPACE / relative
    with path.open('rb') as stream:
        actual = hashlib.file_digest(stream, 'sha256').hexdigest()
    if actual != expected:
        failures.append(relative)
with (ROOT / 'run/fresh_original_source_check.json').open('x') as stream:
    json.dump(dict(status='PASS' if checked == 785 and not failures else 'FAIL',
                   checked=checked, failures=failures, fresh_recomputed_before_gazebo_launch=True,
                   reference=str(baseline)), stream, indent=2)
assert checked == 785 and not failures
print('FRESH_FORMAL_SOURCE_PASS', checked, flush=True)
