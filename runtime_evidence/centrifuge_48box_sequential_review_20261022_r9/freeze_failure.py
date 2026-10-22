import hashlib
import json
from pathlib import Path


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r351'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(name):
    return json.loads((ROOT / name).read_text())


assert read('run/result.json')['status'] == 'FAIL'
assert read('run/result.json')['holding'] is None
assert read('run/result.json')['order']['locations'] == {'left': 'table', 'right': 'table'}
assert read('run/retirement.json')['status'] == 'PASS' and not read('run/retirement.json')['remaining']
assert read('video_audit.json')['status'] == 'PASS'
assert read('run/completion_finalization.json')['status'] == 'PASS'
assert 'SPEED_POST_AUDITS_AND_PROTECTION_COMPLETED' in (ROOT / 'post_speed.log').read_text()
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
               'additional_protected', 'dual_protected', 'current_protected', 'startup_protected'):
    assert read(prefix + '_before.json') == read(prefix + '_after.json')
    assert read(prefix + '_after.json')['status'] == 'PASS'
for name, expected in read('identity.json')['sha256'].items():
    assert digest(ROOT / name) == expected
assert (ROOT / 'REPORT.md').exists() and (REVIEW / 'REPORT.md').exists()
assert 'Button tracking exceeded relaxed simulation limit' in read('run/result.json')['error']
assert read('run/planner_lifecycle.json')['requests'] == 0
assert json.loads((REVIEW / 'performance_limits.json').read_text())['sampled_world_rtf'] is None
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
    and not {'build', 'colcon_log', '__pycache__', '.pytest_cache'}.intersection(path.relative_to(ROOT).parts))
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
result = dict(status='SEALED_FAILED_PREFIX_NOT_COMPLETE_TASK', members=len(members),
    manifest_sha256=digest(ROOT / 'SHA256SUMS'), source_identity_sha256=digest(ROOT / 'identity.json'),
    video_sha256=digest(ROOT / 'run/full_cycle.mp4'), nine_protection_groups_same_pass=True)
with (REVIEW / 'final_review.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
