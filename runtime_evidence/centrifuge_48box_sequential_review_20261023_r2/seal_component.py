import hashlib
import json
from pathlib import Path

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_dart_inertia_20261023_r1'

def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

assert not (ROOT / 'SHA256SUMS').exists()
assert (ROOT / 'conda_six_before.txt').read_bytes() == (ROOT / 'conda_six_after.txt').read_bytes()
assert (ROOT / 'conda_six_revision2_before.txt').read_bytes() == (ROOT / 'conda_six_revision2_after.txt').read_bytes()
for name in ('inertia_revision2.log', 'inertia_repeat_revision2.log'):
    rows = (ROOT / name).read_text().splitlines()
    result = json.loads(next(row for row in reversed(rows) if row.startswith('{"status"')))
    assert result['status'] == 'PASS_EXACT_INERTIA_QUERIES_NOT_FULL_TASK'
    assert result['world_steps'] == 5400 and result['component_speedup'] > 1.
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
    and not {'build', 'colcon_log'}.intersection(path.relative_to(ROOT).parts))
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
print(json.dumps(dict(members=len(members), sha256=digest(ROOT / 'SHA256SUMS'))))

