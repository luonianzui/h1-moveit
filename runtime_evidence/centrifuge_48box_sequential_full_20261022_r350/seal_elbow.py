import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text())


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


deadline = time.monotonic() + 1800
while time.monotonic() < deadline:
    path = ROOT / 'preparation_dispatch.log'
    if path.exists() and 'ELBOW_REGRESSIONS_PREPARED_NOT_SEALED_OR_RUNNING' in path.read_text():
        break
    time.sleep(5)
else:
    raise TimeoutError('No completed preparation; no seal or launch')
assert not (ROOT / 'identity.json').exists() and not (ROOT / 'run').exists()
assert not os.environ.get('LD_PRELOAD')
offline = read('elbow_offline_revision2.json')
assert offline['status'] == 'PASS_OFFLINE_NOT_PHYSICAL'
assert len(offline['records']) == 2
for row in offline['records']:
    assert row['result']['status'] == 'PASS' and row['wall_s'] < 110
    assert row['result']['elbow_posture_selection']['basket_max_from_down_deg'] < 45
assert read('elbow_chain_audit.json')['status'] == 'PASS_OFFLINE_NOT_PHYSICAL'
assert '1425 passed' in (ROOT / 'tests_revision3.log').read_text()
assert '14 passed' in (ROOT / 'startup_review_freeze_tests.log').read_text()
assert '15 passed' in (ROOT / 'startup_review_hold_tests.log').read_text()
assert '15 passed' in (ROOT / 'elbow_transport_rate_tests.log').read_text()
assert '12 passed' in (ROOT / 'elbow_empty_approach_tests.log').read_text()
assert '20 passed' in (ROOT / 'elbow_posture_tests.log').read_text()
assert (ROOT / 'conda_six_before.txt').read_bytes() == (ROOT / 'conda_six_after.txt').read_bytes()
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected',
               'resumed_protected', 'additional_protected', 'dual_protected'):
    assert read(prefix + '_before.json')['status'] == 'PASS'
for name in ('current_bucket_exit_probe.json', 'current_micro_alignment_probe.json'):
    assert read(name)['current_worker_sha256'] == digest(ROOT / 'dual_worker.py')
with (ROOT / 'tests_before_seal.log').open('x') as stream:
    subprocess.run([sys.executable, '-m', 'pytest', '-q', str(ROOT)], stdout=stream, stderr=subprocess.STDOUT, check=True)
with (ROOT / 'dependency_audit.log').open('x') as stream:
    subprocess.run([sys.executable, str(ROOT / 'audit_scene_dependencies.py'), 'audit'],
                   stdout=stream, stderr=subprocess.STDOUT, check=True)
with (ROOT / 'elbow_final_preflight.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_NOT_PHYSICAL',
                   source_sha256=digest(ROOT / 'elbow_posture.py'),
                   offline_sha256=digest(ROOT / 'elbow_offline_revision2.json'),
                   chain_sha256=digest(ROOT / 'elbow_chain_audit.json'),
                   all_seven_before_protection_groups_recomputed_this_run=True,
                   full_actual_fresh_execution_and_independent_audit_still_required=True), stream, indent=2)
available = shutil.disk_usage(ROOT).free
assert available > 2096208487
with (ROOT / 'seal_revision2.log').open('x') as stream:
    subprocess.run([sys.executable, str(ROOT / 'seal_identity.py')], stdout=stream, stderr=subprocess.STDOUT, check=True)
identity_path = ROOT / 'identity.json'
identity = read('identity.json')
assert identity['elbow_posture_contract'] == 'paired_elbow_up_fixed_grasp_v1'
assert identity['elbow_retrieval_transport_cap_rad_s'] == .3
for relative, expected in identity['sha256'].items():
    path = ROOT / relative
    assert digest(path) == expected, relative
    path.chmod(0o444)
identity_path.chmod(0o444)
resources = read('scene_dependency_audit_revision2.json')['selected_resources']
with (ROOT / 'fresh_launch_protection_and_budget.json').open('x') as stream:
    json.dump(dict(status='PASS', sealed_members=len(identity['sha256']), selected_resources=len(resources),
                   identity_sha256=digest(identity_path), available_bytes=available,
                   required_start_budget_bytes=2096208487, no_scene_started=True,
                   old_models_holds_grip_guards_unchanged=True, no_preload_or_contact_cap_change=True), stream, indent=2)
print('ELBOW_ONCE_SEALED_NOT_LAUNCHED', len(identity['sha256']), len(resources), flush=True)
