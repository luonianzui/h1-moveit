import hashlib
import json
from pathlib import Path
import sys


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r348'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(name):
    return json.loads((ROOT / name).read_text())


def seal(folder):
    assert not (folder / 'SHA256SUMS').exists()
    members = sorted(path for path in folder.rglob('*') if path.is_file() and not path.is_symlink()
        and not {'build','colcon_log','__pycache__','.pytest_cache'}.intersection(path.relative_to(folder).parts))
    with (folder / 'SHA256SUMS').open('x') as stream:
        stream.write(''.join(digest(path)+'  '+str(path.relative_to(folder))+'\n' for path in members))
    for path in members+[folder/'SHA256SUMS']:
        path.chmod(0o444)
    result = dict(members=len(members),manifest_sha256=digest(folder/'SHA256SUMS'))
    print(folder.name,result,flush=True)
    return result


if sys.argv[1:] == ['review']:
    assert json.loads((REVIEW/'documentation_validation.json').read_text())['status'] == 'PASS'
    documents = json.loads((REVIEW/'documentation_after.json').read_text())
    for name, expected in documents.items():
        assert digest(REVIEW.parents[1]/name) == expected['sha256'], name
    assert (ROOT/'SHA256SUMS').exists()
    seal(REVIEW)
else:
    assert sys.argv[1:] == ['core']
    assert 'SPEED_POST_AUDITS_AND_PROTECTION_COMPLETED' in (ROOT/'post_speed.log').read_text()
    identity = read('identity.json')
    for name, expected in identity['sha256'].items():
        assert digest(ROOT/name) == expected, name
    assert read('run/result.json')['status'] == 'PASS'
    assert not read('run/result.json')['actual_holder_registry']
    assert read('run/result.json')['order']['locations'] == {'left':'table','right':'table'}
    for name in ('run/retirement.json','run/completion_finalization.json','video_audit.json','trace_audit.json',
        'pipeline_audit.json','bounded_release_audit.json','preload_observation_audit.json','localization_delivery_audit.json','localizer_pipeline_audit.json',
        'speed_physical_audit.json','independent_speed_grip.json'):
        assert read(name)['status'] == 'PASS', name
    assert not read('run/retirement.json')['remaining']
    assert read('speed_task_audit.json')['actual_anchors'] == 4
    assert read('speed_task_audit.json')['actual_maximum_holders'] == 1
    assert len(read('speed_physical_audit.json')['gates']) == 28
    assert all(read('speed_physical_audit.json')['gates'].values())
    assert read('speed_actual_paths_review.json')['exact_published_paths']
    assert read('speed_actual_paths_review.json')['final_path_dense_checks'] > 0
    assert read('pipeline_audit.json')['actual_joined_pairs'] == 4
    assert read('table_alignment_audit.json')['complete_task']
    assert read('interval_audit.json')['status'] == 'PASS'
    assert read('planner_resources_audit.json')['status'] == 'PASS_ACTUAL_STATIC_RESOURCE_REUSE_AND_GUARDED_STATIONARY_PREFETCH'
    assert read('table_alignment_audit.json')['status'] == 'PASS_SUPPORTED_ALIGNMENT_OBSERVATIONS_NOT_COMPLETE_TASK'
    assert json.loads((REVIEW/'native_profile_audit.json').read_text())['status'] == 'PASS_OWNED_PREUPDATE_CPU_SAMPLING_NOT_SPEED_CERTIFICATE'
    assert json.loads((REVIEW/'native_symbol_audit.json').read_text())['status'] == 'PASS_EXACT_EXPORTED_SYMBOL_SIZE_AND_PROCESS_MAP_AUDIT'
    assert json.loads((REVIEW/'dart_solver_audit.json').read_text())['status'] == 'PASS_ACTUAL_SOLVER_GROUP_SAMPLES_NOT_COMPLETE_CENSUS'
    assert read('run/dart_cache_runtime_identity.json')['status'] == 'PASS_ACTUAL_NEW_GAZEBO_CACHED_SOLVER'
    for prefix in ('protected','source_1009','recent_protected','latest_protected','resumed_protected',
        'additional_protected','dual_protected','current_protected','startup_protected'):
        for phase in ('before','after'):
            assert read(f'{prefix}_{phase}.json')['status'] == 'PASS'
    inspection = json.loads((REVIEW/'image_review.json').read_text())
    assert inspection['actual_tool_inspection'] and inspection['terminal_two_boxes_on_table']
    assert (ROOT/'REPORT.md').exists() and (REVIEW/'REPORT.md').exists()
    core = seal(ROOT)
    wall = read('run/executor_wall_clock.json')
    with (REVIEW/'final_review.json').open('x') as stream:
        json.dump(dict(status='PASS_COMPLETE_SEQUENTIAL_TASK', within_180_wall_s=wall['within_180_wall_s'],
            core=core,source_identity_sha256=digest(ROOT/'identity.json'),source_members=len(identity['sha256']),
            wall=wall,pipeline=read('pipeline_audit.json'),task=read('speed_task_audit.json'),
            geometry_component=read('geometry_fastpath_qualification.json'),
            prior_r337_success_preserved=True,
            grip=read('independent_speed_grip.json'),physical=read('speed_physical_audit.json'),
            paths=read('speed_actual_paths_review.json'),video=read('video_audit.json'),
            video_sha256=digest(ROOT/'run/full_cycle.mp4'),image_review=inspection,
            nine_protection_groups_before_after_pass=True, r318_r327_r331_r333_and_failed_r334_preserved=True),stream,indent=2)


