import copy
import json
from pathlib import Path

from h1_2_vision.candidate3_execution_contract import (
    ExecutionContractError,
    validate_result,
)
import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
RESULT_PATH = (
    WORKSPACE / 'runtime_evidence/826_apriltag_plan_only_20260826/'
    'stage_i_diagnostic_g5/nominal/result.json')


@pytest.fixture(scope='module')
def result():
    return json.loads(RESULT_PATH.read_text(encoding='utf-8'))


def test_nominal_g5_result_is_execution_admissible(result):
    plan = validate_result(result)
    assert plan['door_arc_target_count'] == 601
    assert sum(plan['stage_waypoint_counts']) == 8713


@pytest.mark.parametrize(('path', 'value', 'status'), [
    (('certificate', 'minimum_environment_clearance_m'), 0.009999,
     'CERTIFICATE_CLEARANCE'),
    (('certificate', 'no_partial'), False, 'CERTIFICATE_NO_PARTIAL'),
    (('plan', 'contains_partial'), True, 'PLAN_PARTIAL'),
    (('plan', 'door_arc_target_count'), 600, 'PLAN_DOOR_TARGET_COUNT'),
])
def test_mutated_contract_fails_closed(result, path, value, status):
    changed = copy.deepcopy(result)
    target = changed
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    # Re-seal nested documents so each assertion proves the semantic gate,
    # rather than merely exercising the outer content hash.
    from h1_2_vision.candidate3_contract import canonical_hash
    changed['plan_hash'] = canonical_hash(changed['plan'])
    changed['certificate']['plan_hash'] = changed['plan_hash']
    changed['certificate_hash'] = canonical_hash(changed['certificate'])
    with pytest.raises(ExecutionContractError, match=status):
        validate_result(changed)


def test_support_checkpoint_mutation_fails_even_when_rehashed(result):
    changed = copy.deepcopy(result)
    changed['plan']['stages'][3]['messages'][1]['points'][-1][
        'positions'][0] += 0.001
    from h1_2_vision.candidate3_contract import canonical_hash, stage_hash
    changed['plan']['stage_hashes'][3] = stage_hash(
        changed['plan']['stages'][3])
    changed['plan_hash'] = canonical_hash(changed['plan'])
    changed['certificate']['plan_hash'] = changed['plan_hash']
    changed['certificate_hash'] = canonical_hash(changed['certificate'])
    with pytest.raises(
            ExecutionContractError, match='CUBE_ATTACH_GRIPPER_ENDPOINT'):
        validate_result(changed)


def test_post_navigation_alignment_hash_is_bound_across_plan_certificate(
        result):
    from h1_2_vision.candidate3_contract import canonical_hash

    changed = copy.deepcopy(result)
    alignment_hash = 'a' * 64
    changed['plan']['source_epoch_status'] = (
        f'FROZEN_POST_NAV_ALIGNMENT:{alignment_hash}')
    changed['plan']['post_nav_alignment_hash'] = alignment_hash
    changed['plan_hash'] = canonical_hash(changed['plan'])
    changed['certificate']['plan_hash'] = changed['plan_hash']
    changed['certificate']['source_epoch_status'] = (
        changed['plan']['source_epoch_status'])
    changed['certificate']['post_nav_alignment_hash'] = 'b' * 64
    changed['certificate_hash'] = canonical_hash(changed['certificate'])
    with pytest.raises(
            ExecutionContractError,
            match='CERTIFICATE_POST_NAV_ALIGNMENT_HASH'):
        validate_result(changed)


def test_dynamic_scene_can_freeze_once_and_normalizes_support_state():
    source = (WORKSPACE / 'src/h1_2_vision/h1_2_vision/'
              'dynamic_scene.py').read_text(encoding='utf-8')
    assert "self.declare_parameter('freeze_once', False)" in source
    assert 'self.frozen_once = True' in source
    assert 'message.data.strip().upper()' in source
    assert 'scene.robot_state.joint_state = joint_state' in source
    assert 'scene.robot_state.is_diff = False' in source
    assert "'require_post_navigation_hold', False" in source
    assert "'/dynamic_scene/post_nav_alignment'" in source
    assert 'validate_door_tag_cross_check(' in source
    assert "'CABINET_TAGS_10_12_RIGID_MODEL_TAG11_CHECKED'" in source
    assert "'epoch_status': epoch.status" in source


def test_execution_supervisor_has_bounded_rtf_and_tag_occlusion_holds():
    source = (WORKSPACE / 'src/h1_2_vision/h1_2_vision/'
              'candidate3_execution_supervisor.py').read_text(encoding='utf-8')
    assert "'trajectory_timeout_factor', 10.0" in source
    assert 'self.timeout_factor > 10.0' in source
    assert "'tag_reacquisition_timeout_s', 5.0" in source
    assert 'self.tag_reacquisition_timeout > 5.0' in source
    assert "'CABINET_TAG_OCCLUSION_HOLD'" in source
    assert "'CABINET_TAG_REACQUIRED_RESUME_SUFFIX'" in source
    assert "'CABINET_TAG_RESUME_CONNECTOR_CPP_AUDIT_PASS'" in source
    assert "'CABINET_TAG_REACQUIRED_RESUME_CPP_CONNECTOR'" in source
    assert "'candidate3_clearance_auditor'" in source
    assert 'connector_error_rad=error' in source
    assert 'math.sqrt(6.0 * error / 2.0)' in source
    assert "self.declare_parameter('resume_stage_index', -1)" in source
    assert '(((-1, -1), (3, 2)))' not in source
    assert '((-1, -1), (1, 0), (2, 3), (3, 2), (3, 3), (6, 1))' in source
    assert "'RECOVERY_UNLOCK_ENDPOINT_READY'" in source
    assert "'RECOVERY_DOOR_ATTACH_READY'" in source
    assert "'door_recovery_owner_file'" in source
    assert "'door_recovery_already_attached'" in source
    assert "'cube_recovery_preload_file'" in source
    assert "'cube_recovery_already_attached'" in source
    assert "'recover_attach'" in source
    assert "'recover_detach'" in source
    assert "'RECOVERY_DOOR_ALREADY_ATTACHED_READY'" in source
    assert "'RECOVERY_CUBE_ALREADY_ATTACHED_READY'" in source
    assert "'REMAINING_PATH_REPLAN_CPP_AUDIT_PASS'" in source
    assert 'expected_left = (' in source
    assert ('self.wait_support('
            "'left'"
            ', expected_left, 1.0)') in source
    assert "'RECOVERY_HANDLE_RELEASE_READY'" in source
    assert "'require_post_navigation_alignment', False" in source
    assert "'/h1_2_navigation/incubator_navigation/status'" in source
    assert 'validate_navigation_hold_status(' in source


def test_handle_recovery_uses_measured_contact_connector():
    source = (WORKSPACE / 'src/h1_2_vision/h1_2_vision/'
              'candidate3_handle_recovery.py').read_text(encoding='utf-8')
    assert 'if closed_error > 0.006' in source
    assert "'gripper_clamp_error_rad': closed_error" in source
    assert 'actual_gripper = [joints[name] for name in gripper_names]' in source
    assert 'math.sqrt(6.0 * connector_error / 2.0)' in source
    assert 'recovery_time_scale = 2' in source
    assert "'RECOVERY_CABINET_TAG_HOLD'" in source
    assert "'RECOVERY_TAG_REACQUIRED_RESUME_SUFFIX'" in source
    assert 'math.radians(7.0)' in source
    assert "'MECHANISM_RELEASE_READY'" in source
    assert 'elif index == 1:' in source
    # The moving limit is the model's exact 0.35 rad/s joint limit plus only a
    # floating-point comparison epsilon; this must not become a policy margin.
    assert '(0.01 if stationary_mechanism else 0.35 + 1e-6)' in source


def test_locked_handle_return_overcomes_friction_without_blocking_unlock():
    model = (WORKSPACE / 'src/h1_2_description/models/incubator/'
             'model.sdf').read_text(encoding='utf-8')
    plugin = (WORKSPACE / 'src/h1_2_moveit_demo/src/'
              'incubator_latch_system.cpp').read_text(encoding='utf-8')
    assert '<handle_detent_torque>0.08</handle_detent_torque>' in model
    assert (
        '<locked_handle_return_torque>0.40</locked_handle_return_torque>'
        in model)
    assert '<friction>0.02</friction>' in model
    assert 'locked_ && locked_return_enabled_' in plugin
    assert 'locked_return_enabled_ = true;' in plugin
    assert 'locked_return_enabled_ = false;' in plugin
    assert 'locked_handle_return_torque_ : handle_detent_torque_' in plugin
