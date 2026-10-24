import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261024_r359'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(name, content):
    with (REVIEW / name).open('x') as stream:
        json.dump(content, stream, indent=2, ensure_ascii=False)


def seal(folder):
    assert not (folder / 'SHA256SUMS').exists()
    for descriptor in (1, 2):
        destination = os.readlink(f'/proc/self/fd/{descriptor}')
        assert not (destination.startswith('/') and Path(destination).resolve().is_relative_to(folder.resolve()))
    members = sorted(path for path in folder.rglob('*') if path.is_file() and not path.is_symlink()
                     and not {'build', 'colcon_log', '__pycache__', '.pytest_cache'}.intersection(path.relative_to(folder).parts))
    with (folder / 'SHA256SUMS').open('x') as stream:
        stream.write(''.join(digest(path) + '  ' + str(path.relative_to(folder)) + '\n' for path in members))
    for path in members + [folder / 'SHA256SUMS']:
        path.chmod(0o444)
    result = dict(members=len(members), manifest_sha256=digest(folder / 'SHA256SUMS'))
    print(folder.name, result, flush=True)
    return result


def checkpoint():
    sys.path.insert(0, str(ROOT))
    from scene_contract import read_contract
    assert not (ROOT / 'identity.json').exists() and not (ROOT / 'run').exists()
    assert 'R359_PREPARED_WITH_CENTERED_EXIT_NOT_SEALED_NOT_LAUNCHED' in (ROOT / 'preparation_full_stdout.log').read_text()
    contract = read_contract(ROOT)
    tests = {}
    for name, count in (('tests_speed_final.log', 2359), ('tests_final_qualification.log', 2359),
                        ('package_tests.log', 46), ('loading_regression_tests_final.log', 122)):
        content = (ROOT / name).read_text()
        match = re.search(r'(\d+) passed in ([\d.]+)s', content)
        assert match and int(match[1]) == count and ' failed' not in content
        tests[name] = dict(passed=count, seconds=float(match[2]), sha256=digest(ROOT / name))
    assert '100% tests passed, 0 tests failed out of 6' in (ROOT / 'ctest.log').read_text()
    assert (ROOT / 'conda_six_before.txt').read_bytes() == (ROOT / 'conda_six_after.txt').read_bytes()
    qualification = json.loads((ROOT / 'centered_exit_qualification_revision2.json').read_text())
    assert qualification['status'] == 'PASS_R358_FAILED_AND_R356_BOTH_SUCCESS_SNAPSHOTS_NOT_PHYSICAL'
    records = []
    for row in qualification['records']:
        plan, audit = row['plan'], row['independent_full_scene_dense_review']
        assert plan['checked_samples'] == audit['final_path_dense_checks']
        assert digest(Path(row['source'])) == row['source_sha256']
        records.append(dict(source=row['source'], source_sha256=row['source_sha256'],
                            accepted_dense_checks=plan['checked_samples'], independent_dense_checks=audit['final_path_dense_checks'],
                            correction=plan['stages'][0].get('clearance_correction')))
    protection = {}
    for prefix in ('r354', 'r357', 'r358'):
        before = json.loads((ROOT / f'{prefix}_protection_before.json').read_text())
        after = json.loads((ROOT / f'{prefix}_protection_after.json').read_text())
        assert before == after and after['status'] == 'PASS'
        protection[prefix] = dict(status='PASS', checked_files=len(after['sha256']),
                                 after_sha256=digest(ROOT / f'{prefix}_protection_after.json'))
    for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
                   'additional_protected', 'dual_protected', 'current_protected', 'startup_protected'):
        assert json.loads((ROOT / f'{prefix}_before.json').read_text())['status'] == 'PASS'
    auxiliary = json.loads((REVIEW / 'auxiliary_seals.json').read_text())
    for name, record in auxiliary.items():
        folder = ROOT.parent / name
        assert digest(folder / 'SHA256SUMS') == record['manifest_sha256']
        for line in (folder / 'SHA256SUMS').read_text().splitlines():
            expected, relative = line.split('  ', 1)
            assert digest(folder / relative) == expected
    source_hashes = {path.name: digest(path) for path in sorted(ROOT.iterdir())
                     if path.is_file() and path.suffix in ('.py', '.sh')}
    save('OFFLINE_CHECKPOINT.json', dict(status='OFFLINE_QUALIFIED_NOT_EXECUTION_SEALED_NOT_RUN',
        user_switched_to_documentation_and_github=True, identity_exists=False, run_exists=False,
        physical_success_claimed=False, new_task_video_exists=False, april_tag_installed=False,
        source_hashes=source_hashes, tests=tests, package_ctest_passed=6,
        selected_scene_dependencies=len(contract['selected_resources']), navigation=contract['regression'],
        geometry=records, latest_protection=protection, auxiliaries=auxiliary,
        conda_six_unchanged=True, original_nine_groups_before_pass=True,
        documentation_protection_scope='separate bounded after audit; backups checked during preparation only',
        free_bytes_at_checkpoint=shutil.disk_usage(ROOT).free,
        latest_complete_success='r356', latest_complete_wall_s=481.783994177,
        future_execution_requires_new_identity=True))


if sys.argv[1:] == ['core']:
    checkpoint()
    result = seal(ROOT)
    save('offline_seal_receipt.json', dict(status='SEALED_OFFLINE_CANDIDATE_NOT_SIMULATION_SUCCESS', **result))
elif sys.argv[1:] == ['review']:
    assert json.loads((REVIEW / 'documentation_validation.json').read_text())['status'] == 'PASS'
    documents = json.loads((REVIEW / 'documentation_after.json').read_text())
    for name, expected in documents.items():
        assert digest(REVIEW.parents[1] / name) == expected['sha256']
    receipt = json.loads((REVIEW / 'offline_seal_receipt.json').read_text())
    assert digest(ROOT / 'SHA256SUMS') == receipt['manifest_sha256']
    for line in (ROOT / 'SHA256SUMS').read_text().splitlines():
        expected, relative = line.split('  ', 1)
        assert digest(ROOT / relative) == expected
    seal(REVIEW)
else:
    raise ValueError('Expected core or review')
