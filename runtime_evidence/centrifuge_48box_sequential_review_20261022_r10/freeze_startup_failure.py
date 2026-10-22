import hashlib
import json
from pathlib import Path


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r352'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert not (ROOT / 'run/execution.claim').exists()
assert not (ROOT / 'run/result.json').exists()
assert not (ROOT / 'run/full_cycle.mp4').exists()
assert json.loads((ROOT / 'run/leg_initialization.json').read_text())['status'] == 'FAIL'
retirement = json.loads((ROOT / 'run/retirement.json').read_text())
assert retirement['status'] == 'PASS' and not retirement['remaining']
assert 'JOINT_SOURCE_STEP:10178000000:10179000000:1000000' in (ROOT / 'run/launch.log').read_text()
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
               'additional_protected', 'dual_protected', 'current_protected', 'startup_protected'):
    before = json.loads((ROOT / (prefix + '_before.json')).read_text())
    after = json.loads((ROOT / (prefix + '_after.json')).read_text())
    assert before == after and after['status'] == 'PASS'
for relative, expected in json.loads((ROOT / 'identity.json').read_text())['sha256'].items():
    assert digest(ROOT / relative) == expected, relative
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
    and not {'build', 'colcon_log', '__pycache__', '.pytest_cache'}.intersection(path.relative_to(ROOT).parts))
assert not (ROOT / 'SHA256SUMS').exists()
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
report = dict(status='SEALED_FAILED_STARTUP_NOT_EXECUTED', members=len(members),
              manifest_sha256=digest(ROOT / 'SHA256SUMS'), source_identity_sha256=digest(ROOT / 'identity.json'),
              nine_protection_groups_same_pass=True, actual_task_video_created=False,
              all_old_successes_preserved=True)
with (REVIEW / 'final_review.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report), flush=True)
