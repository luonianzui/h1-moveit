"""Pure-data tests for the candidate-3 middle-only contract."""

import copy
import json
from pathlib import Path

from h1_2_vision.candidate3_middle_contract import (
    ASSET_SPECS,
    build_middle_plan,
    canonical_hash,
    default_asset_paths,
    FORBIDDEN_LEGACY_STAGES,
    HANDLE_ANGLE_RAD,
    load_middle_assets,
    MiddleContractError,
    PHASE_NAMES,
    SEAM_JOINT_NAMES,
    SEAM_NOMINAL,
    SEAM_STATE_HASH,
    SEAM_TOLERANCES,
    validate_seam_positions,
)

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]


@pytest.fixture(scope='module')
def assets():
    """Load the immutable middle assets once for this test module."""
    return load_middle_assets(default_asset_paths(WORKSPACE))


def test_only_three_8263_middle_assets_are_admitted(assets):
    """Only right door, left cube, and reverse right door are accepted."""
    assert tuple(assets) == tuple(value.logical_name for value in ASSET_SPECS)
    assert tuple(value['payload']['stage'] for value in assets.values()) == (
        'door', 'cube', 'door')
    assert all(value['payload_sha256'] == canonical_hash(value['payload'])
               for value in assets.values())


def test_middle_plan_has_no_legacy_left_handle_stage(assets):
    """The plan contains no legacy left-handle stage or command."""
    plan = build_middle_plan(
        assets, scene_epoch_id=1, raw_scene_hash='1' * 64,
        physical_scene_hash='2' * 64)
    assert plan['stage_names'] == list(PHASE_NAMES)
    assert not set(plan['stage_names']) & set(FORBIDDEN_LEGACY_STAGES)
    assert plan['left_handle_stage_count'] == 0
    assert all(phase['authorized_hand'] == 'right'
               for phase in plan['phases']
               if phase['source_asset'] != 'cube')
    assert all(phase['authorized_hand'] == 'left'
               for phase in plan['phases']
               if phase['source_asset'] == 'cube')
    assert all(phase['handle_exact_retained'] is True
               for phase in plan['phases'])
    assert all(phase['handle_angle_rad'] == pytest.approx(HANDLE_ANGLE_RAD)
               for phase in plan['phases'])
    assert plan['exact_geometry_audit_requirement'] == (
        'EXTERNAL_EVIDENCE_REQUIRED')
    assert plan['controller_commands_issued'] == 0


def test_torso_zero_is_cryptographically_bound_into_seam():
    """The seam hash includes the strict torso-zero position and tolerance."""
    contract = dict(zip(SEAM_JOINT_NAMES, SEAM_NOMINAL))
    validate_seam_positions(contract)
    assert contract['torso_joint'] == 0.0
    assert SEAM_STATE_HASH == canonical_hash({
        'schema': 'h1_candidate3_middle_seam/v1',
        'joint_names': list(SEAM_JOINT_NAMES),
        'nominal_positions': list(SEAM_NOMINAL),
        'absolute_tolerances': list(SEAM_TOLERANCES),
        'torso_controller_authorized': False,
    })
    changed = dict(contract)
    changed['torso_joint'] = 0.003001
    with pytest.raises(MiddleContractError, match='SEAM_POSITION:torso_joint'):
        validate_seam_positions(changed)


def test_wrong_hand_topic_fails_closed_even_if_asset_is_resealed(tmp_path):
    """A left-hand command cannot be inserted into a right-door asset."""
    paths = default_asset_paths(WORKSPACE)
    for logical_name, source in paths.items():
        target = tmp_path / source.name
        target.write_bytes(source.read_bytes())
        paths[logical_name] = target
    door_path = paths['door_open']
    document = json.loads(door_path.read_text(encoding='utf-8'))
    document['payload']['messages'][0]['topic'] = (
        '/left_gripper_controller/joint_trajectory')
    document['sha256'] = canonical_hash(document['payload'])
    door_path.write_text(json.dumps(document), encoding='utf-8')
    # The immutable file hash is checked before semantic parsing.
    with pytest.raises(MiddleContractError, match='ASSET_FILE_HASH:door_open'):
        load_middle_assets(paths)


def test_forbidden_four_asset_set_is_rejected(assets):
    """Adding any fourth/legacy asset changes the exact admitted asset set."""
    changed = copy.deepcopy(assets)
    changed['handle_unlock'] = changed['door_open']
    with pytest.raises(MiddleContractError, match='ASSET_SET'):
        build_middle_plan(
            changed, scene_epoch_id=1, raw_scene_hash='1' * 64,
            physical_scene_hash='2' * 64)
