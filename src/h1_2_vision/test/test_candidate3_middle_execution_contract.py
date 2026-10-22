"""Certificate and worker tests for the middle-v1 plan-only chain."""

import copy
from pathlib import Path

from h1_2_vision.candidate3_middle_contract import (
    AUDIT_EVIDENCE_SCHEMA,
    AUDITOR_VERSION,
    build_middle_plan,
    canonical_hash,
    default_asset_paths,
    HANDLE_EXACT_ID,
    HANDLE_STL_SHA256,
    load_middle_assets,
    make_certificate,
    MASK_CONFLICT_PAIR,
    MASK_OBSERVED_DISTANCE_M,
    MASK_RECHECK_ANGULAR_STEP_RAD,
    MASK_RECHECK_DENSE_SCOPE,
    MASK_RECHECK_PRISMATIC_STEP_M,
    MiddleContractError,
    PHASE_NAMES,
)
from h1_2_vision.candidate3_middle_execution_contract import (
    MiddleExecutionContractError,
    RESULT_SCHEMA,
    validate_result,
)
from h1_2_vision.candidate3_middle_worker import build_result

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
WORKER_SOURCE = (
    WORKSPACE / 'src/h1_2_vision/h1_2_vision/'
    'candidate3_middle_worker.py').read_text(encoding='utf-8')


@pytest.fixture(scope='module')
def plan():
    """Build a deterministic contract-only plan from the sealed assets."""
    assets = load_middle_assets(default_asset_paths(WORKSPACE))
    return build_middle_plan(
        assets, scene_epoch_id=9, raw_scene_hash='a' * 64,
        physical_scene_hash='b' * 64)


def valid_evidence(plan_value):
    """Return the smallest exact-auditor document accepted by middle-v1."""
    neighborhood = [{
        'offset': offset,
        'dense_sample_index': 527 + offset,
        'exact_stl': True,
        'pair': [HANDLE_EXACT_ID, 'left_dex1_finger_2_link'],
        'distance_m': 0.0105,
    } for offset in (-1, 0, 1)]
    return {
        'schema': AUDIT_EVIDENCE_SCHEMA,
        'auditor_version': AUDITOR_VERSION,
        'audit_method': 'FCL_UNPADDED_DYNAMIC_EXACT_STL',
        'plan_hash': canonical_hash(plan_value),
        'raw_scene_hash': plan_value['raw_scene_hash'],
        'physical_scene_hash': plan_value['physical_scene_hash'],
        'dynamic_object_schedule_hash':
            plan_value['dynamic_object_schedule_hash'],
        'controller_commands_issued': 0,
        'samples': 5621,
        'collision_free': True,
        'minimum_environment_clearance_m': 0.010001,
        'minimum_environment_clearance_is_lower_bound': True,
        'environment_clearance_derivation': {
            'exact_world_objects': 'direct_unpadded_FCL_distance',
            'environment_table_10mm_margin':
                '0.010_plus_guard_distance',
            'green_cube_10mm_guard': '0.010_plus_guard_distance',
            'self_pairs_excluded': True,
        },
        'forward_passed': True,
        'reverse_passed': True,
        'timing_passed': True,
        'no_partial': True,
        'planned_action_duration_s': 32.296167783,
        'dynamic_handle_exact': {
            'id': HANDLE_EXACT_ID,
            'geometry_type': 'mesh',
            'mesh_count': 1,
            'primitive_count': 0,
            'geometry_sha256': HANDLE_STL_SHA256,
            'retained_every_phase': True,
            'audited_phases': list(PHASE_NAMES),
            'angle_rad': 1.5707963267948966,
            'door_angle_min_rad': 0.0,
            'door_angle_max_rad': 1.0471975511965976,
        },
        'right_handle_allowed_contact_pairs': [
            [HANDLE_EXACT_ID, 'right_dex1_finger_1_pad_link'],
            [HANDLE_EXACT_ID, 'right_dex1_finger_2_pad_link'],
        ],
        'right_door_allowed_contact_pairs': [],
        'right_handle_contact_window': {
            'door_open_message_range': [5, 13],
            'cube_message_range': [0, 6],
            'door_close_message_range': [0, 8],
            'post_release_retreat_strict': True,
        },
        'torso_joint': {
            'commanded': False,
            'minimum_rad': 0.0,
            'maximum_rad': 0.0,
        },
        'left_handle': {
            'allowed_contact_pairs': [],
            'contact_count': 0,
            'minimum_clearance_m': 0.0105,
        },
        'mask_projection': {
            'removed_visual_mask_ids': list(MASK_CONFLICT_PAIR),
            'unknown_visual_mask_ids': [],
            'classification': 'ALLOWLIST_MASKS_PROJECTED',
            'handle_exact_retained': True,
            'raw_scene_hash': plan_value['raw_scene_hash'],
            'physical_scene_hash': plan_value['physical_scene_hash'],
        },
        'mask_conflict_recheck': {
            'visual_pair': list(MASK_CONFLICT_PAIR),
            'phase': 'cube_release',
            'source_message_index': 4,
            'dense_sample_index': 527,
            'dense_index_scope': MASK_RECHECK_DENSE_SCOPE,
            'dense_grid_max_angular_step_rad':
                MASK_RECHECK_ANGULAR_STEP_RAD,
            'dense_grid_max_prismatic_step_m':
                MASK_RECHECK_PRISMATIC_STEP_M,
            'observed_visual_distance_m': MASK_OBSERVED_DISTANCE_M,
            'classification': 'MASK_ONLY_FALSE_POSITIVE',
            'physical_exact_stl_neighborhood': neighborhood,
        },
        'per_phase_metrics': [{
            'phase': phase,
            'samples': 1,
            'right_handle_authorized_samples': (
                1 if phase not in ('door_pre_support', 'right_retreat')
                else 0),
            'minimum_is_lower_bound': True,
            'minimum_environment_clearance_m': 0.010001,
        } for phase in PHASE_NAMES],
    }


def sealed_result(plan_value, evidence):
    """Seal a result exactly as the worker does after external FCL evidence."""
    certificate = make_certificate(plan_value, evidence)
    return {
        'schema': RESULT_SCHEMA,
        'plan': plan_value,
        'plan_hash': canonical_hash(plan_value),
        'audit_evidence': evidence,
        'audit_evidence_hash': canonical_hash(evidence),
        'certificate': certificate,
        'certificate_hash': canonical_hash(certificate),
    }


def test_valid_exact_evidence_completes_certificate_chain(plan):
    """A correctly bound exact audit is execution-admissible."""
    evidence = valid_evidence(plan)
    assert validate_result(sealed_result(plan, evidence)) == plan


@pytest.mark.parametrize(('mutation', 'status'), [
    (lambda value: value['dynamic_handle_exact'].__setitem__(
        'retained_every_phase', False), 'AUDIT_HANDLE_RETAINED'),
    (lambda value: value['left_handle'].__setitem__(
        'contact_count', 1), 'AUDIT_LEFT_HANDLE_CONTACT'),
    (lambda value: value['mask_projection'].__setitem__(
        'removed_visual_mask_ids', []), 'AUDIT_MASK_PROJECTION_BRANCH'),
    (lambda value: value['mask_conflict_recheck'][
        'physical_exact_stl_neighborhood'][1].__setitem__(
            'distance_m', 0.009999), 'AUDIT_MASK_PHYSICAL_CLEARANCE'),
    (lambda value: value['mask_conflict_recheck'][
        'physical_exact_stl_neighborhood'][2].__setitem__(
            'dense_sample_index', 529), 'AUDIT_MASK_NEIGHBORHOOD'),
])
def test_dynamic_handle_and_mask_evidence_fail_closed(plan, mutation, status):
    """Rehashed claims cannot bypass the physical evidence gates."""
    evidence = valid_evidence(plan)
    mutation(evidence)
    with pytest.raises(MiddleContractError, match=status):
        make_certificate(plan, evidence)


def test_already_physical_raw_scene_branch_keeps_exact_stl_gate(plan):
    """A clean six-object CDR needs no synthetic visual-mask witness."""
    evidence = valid_evidence(plan)
    evidence['mask_projection'].update({
        'removed_visual_mask_ids': [],
        'classification': 'RAW_SCENE_ALREADY_PHYSICAL',
    })
    evidence['mask_conflict_recheck'].update({
        'observed_visual_distance_m': None,
        'classification': 'RAW_SCENE_ALREADY_PHYSICAL',
    })
    certificate = make_certificate(plan, evidence)
    assert certificate['valid'] is True

    evidence['mask_conflict_recheck'][
        'physical_exact_stl_neighborhood'][0]['distance_m'] = 0.009
    with pytest.raises(
            MiddleContractError, match='AUDIT_MASK_PHYSICAL_CLEARANCE'):
        make_certificate(plan, evidence)


def test_torso_authorization_mutation_is_rejected_after_rehash(plan):
    """No certificate can authorize torso motion inside the middle plan."""
    changed_plan = copy.deepcopy(plan)
    changed_plan['torso_controller_authorized'] = True
    evidence = valid_evidence(changed_plan)
    with pytest.raises(MiddleContractError, match='PLAN_TORSO_CONTROLLER'):
        make_certificate(changed_plan, evidence)


def test_pending_result_is_not_execution_admissible(plan):
    """A plan without exact evidence is never an execution certificate."""
    result = {
        'schema': RESULT_SCHEMA,
        'plan': plan,
        'plan_hash': canonical_hash(plan),
        'audit_evidence': None,
        'audit_evidence_hash': '',
        'certificate': {'valid': False},
        'certificate_hash': canonical_hash({'valid': False}),
    }
    with pytest.raises(
            MiddleExecutionContractError, match='RESULT_AUDIT_EVIDENCE'):
        validate_result(result)


def test_worker_is_offline_and_calls_only_retime_check():
    """The worker has no ROS/controller and invokes only retime-check."""
    assert 'import rclpy' not in WORKER_SOURCE
    assert 'MoveItPy' not in WORKER_SOURCE
    assert "'retime-check'" in WORKER_SOURCE
    assert "'replay'" not in WORKER_SOURCE
    assert '.execute(' not in WORKER_SOURCE


def test_worker_runs_nine_offline_retime_checks_and_stays_pending():
    """The first pass produces a timed plan but no validity claim."""
    result = build_result(
        WORKSPACE, 3, 'c' * 64, 'd' * 64,
        WORKSPACE / 'src/h1_2_moveit_demo/scripts/'
        'h1_joint_trajectory_asset.py')
    assert result['certificate']['valid'] is False
    assert result['certificate']['status'] == (
        'EXACT_GEOMETRY_AUDIT_REQUIRED')
    assert len(result['plan']['retime_audits']) == 9
    assert result['plan']['planned_action_duration_s'] == pytest.approx(
        32.296167783)
    assert result['plan']['controller_commands_issued'] == 0
