import hashlib
import inspect
import json
from pathlib import Path
import time

from contact_phase_policy import bounded_motion
import dense_shortcut
from dense_shortcut_source import restore_dense_function
from dual_motion import common_arm_timing
import dual_worker


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_native_gripper_feedback_20261014_r269'
EXPECTED_MANIFEST = '013fe02ea9649aa052303ca8f46921d33f474a686f4e8aade8ef1efd73b49150'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    assert digest(PARENT / 'SHA256SUMS') == EXPECTED_MANIFEST
    manifest = {relative.lstrip('*'): checksum for checksum, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    relative = 'run/5_retrieval_pickup_request.json'
    for name in (relative, 'run/5_retrieval_pickup_result.json', 'dual_worker.py'):
        assert digest(PARENT / name) == manifest[name]
    assert json.loads((PARENT / 'run/5_retrieval_pickup_result.json').read_text())['status'] == 'FAIL'
    actual_request = json.loads((PARENT / relative).read_text())
    assert actual_request['retrieval'] is True and actual_request['original_runtime_holder_references'] == {}
    candidate = inspect.getsource(dense_shortcut.free_path)
    assert hashlib.sha256(candidate.encode()).hexdigest() == '826bcb3e5b617c6cb37636b55e2dade2ee9ffeea8ad1bb1707305a1d25e0519e'
    assert restore_dense_function(candidate) == inspect.getsource(dual_worker.free_path)
    begin = time.monotonic()
    inherited_selector = dual_worker.select_free_path
    dual_worker.select_free_path = dense_shortcut.select_free_path
    try:
        result = dual_worker.plan_pickup(actual_request)
    finally:
        dual_worker.select_free_path = inherited_selector
    assert result['status'] == 'PASS' and result['maximum_joint_step_rad'] == .002
    assert result['exact_actual_full_state_used'] and result['no_physical_attachment']
    lookahead = result['prospective_retrieval_lookahead']
    assert lookahead['actual_post_grasp_plan_and_guards_still_required']
    assert lookahead['no_command_or_native_reference_freeze_from_prediction']
    approach = next(stage['samples'] for stage in result['stages'] if stage['phase'] == 'approach')
    rate, tracking = bounded_motion('paired_retrieval_approach', .8)
    timing = common_arm_timing(actual_request['positions'], approach, rate)
    assert rate == .4 and tracking == .002 and timing['original_floor_sim_s'] == .020
    with (EVIDENCE / 'dense_retrieval_actual_probe_result.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    audit = dict(status='PASS', scope='Frozen r269 exact failed actual scene, offline only, not fresh execution or 180 certificate',
                 original_request_sha256=manifest[relative], original_failure_result_sha256=manifest['run/5_retrieval_pickup_result.json'],
                 current_worker_sha256=digest(EVIDENCE / 'dual_worker.py'),
                 current_dense_module_sha256=digest(EVIDENCE / 'dense_shortcut.py'),
                 modified_function_sha256=hashlib.sha256(candidate.encode()).hexdigest(),
                 only_shortcut_sampling_changed_rad=.002, tree_edge_sampling_unchanged_rad=.035,
                 original_final_full_state_sampling_unchanged_rad=.002,
                 original_ik_grasp_targets_models_acm_rate_and_all_physical_guards_preserved=True,
                 flips_deg=result['flips_deg'], pickup_checks=result['checked_samples'],
                 prospective_pilot_checks=lookahead['pilot_checks'], prospective_carry_checks=lookahead['carry_checks'],
                 approach_samples=len(approach), approach_rate_rad_s=rate,
                 approach_command_duration_sim_s=timing['duration_sim_s'], elapsed_wall_s=time.monotonic()-begin,
                 actual_fresh_online_scene_and_post_grasp_plan_still_required=True,
                 no_old_source_write_no_robot_command_no_new_launch_no_claim=True)
    with (EVIDENCE / 'dense_retrieval_actual_probe_audit.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    print(json.dumps(audit), flush=True)


if __name__ == '__main__':
    main()
