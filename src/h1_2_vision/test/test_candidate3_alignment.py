"""Pure and frozen-CDR tests for post-navigation visual alignment."""

import copy
import math
from pathlib import Path

from h1_2_vision.candidate3_alignment import (
    base_pose_residual,
    MAX_BASE_X_M,
    MAX_BASE_Y_M,
    MAX_BASE_YAW_RAD,
    MAX_BASE_Z_M,
    planning_from_base_residual,
    recover_scene_alignment,
    require_common_rigid_transform,
    validate_base_envelope,
    validate_se3,
)
from h1_2_vision.candidate3_base_residual_matrix import derive_case
from h1_2_vision.candidate3_contract import yaw_transform
import numpy as np
import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
ATTEMPT11 = WORKSPACE.joinpath(
    'runtime_evidence', '826_apriltag_execution_20260827',
    'stage_l_formal', 'attempt11', 'planner',
    'epoch_1_900a6b2ae3bd_1572059932ce44668f1804b70b59036c')


def test_base_translation_and_yaw_produce_opposite_scene_correction():
    """A positive base motion produces the inverse object correction."""
    base = yaw_transform([0.008, -0.006, 0.0], math.radians(0.5))
    planning = planning_from_base_residual(base)
    recovered = base_pose_residual(planning)
    assert np.allclose(recovered, base, atol=1.0e-12, rtol=0.0)
    correction = planning @ np.linalg.inv(
        planning_from_base_residual(np.eye(4)))
    assert correction[0, 3] < 0.0
    assert correction[1, 3] > 0.0
    assert math.atan2(correction[1, 0], correction[0, 0]) < 0.0


def test_scene_rigid_transform_tamper_fails_closed():
    """One independently moved object cannot masquerade as base motion."""
    common = yaw_transform([0.01, -0.02, -1.0], math.radians(0.4))
    candidates = {
        name: common.copy()
        for name in ('body', 'tray', 'door', 'cube', 'guard')
    }
    assert np.array_equal(require_common_rigid_transform(candidates), common)
    candidates['door'][0, 3] += 1.1e-6
    with pytest.raises(ValueError, match='SCENE_EPOCH_RIGID_TRANSLATION'):
        require_common_rigid_transform(candidates)


def test_nonfinite_and_nonrigid_se3_fail_closed():
    """Non-finite and improper matrices never reach planning."""
    nonfinite = np.eye(4)
    nonfinite[0, 3] = math.nan
    with pytest.raises(ValueError, match='NONFINITE'):
        validate_se3(nonfinite)
    reflection = np.eye(4)
    reflection[0, 0] = -1.0
    with pytest.raises(ValueError, match='ROTATION_DETERMINANT'):
        validate_se3(reflection)


@pytest.mark.parametrize(('base', 'status'), [
    (yaw_transform([MAX_BASE_X_M + 1.0e-9, 0.0, 0.0], 0.0),
     'BASE_X_ENVELOPE'),
    (yaw_transform([0.0, MAX_BASE_Y_M + 1.0e-9, 0.0], 0.0),
     'BASE_Y_ENVELOPE'),
    (yaw_transform([0.0, 0.0, MAX_BASE_Z_M + 1.0e-9], 0.0),
     'BASE_Z_ENVELOPE'),
    (yaw_transform([0.0, 0.0, 0.0], MAX_BASE_YAW_RAD + 1.0e-9),
     'BASE_YAW_ENVELOPE'),
])
def test_super_envelope_base_translation_fails_closed(base, status):
    """Every tested base residual beyond an explicit bound is rejected."""
    with pytest.raises(ValueError, match=status):
        validate_base_envelope(base)


@pytest.mark.parametrize('base', [
    yaw_transform([MAX_BASE_X_M, 0.0, 0.0], 0.0),
    yaw_transform([-MAX_BASE_X_M, 0.0, 0.0], 0.0),
    yaw_transform([0.0, MAX_BASE_Y_M, 0.0], 0.0),
    yaw_transform([0.0, -MAX_BASE_Y_M, 0.0], 0.0),
    yaw_transform([0.0, 0.0, 0.0], MAX_BASE_YAW_RAD),
    yaw_transform([0.0, 0.0, 0.0], -MAX_BASE_YAW_RAD),
])
def test_exact_symmetric_base_envelope_boundary_is_inclusive(base):
    """The certified symmetric boundary survives matrix round-off."""
    recovered = base_pose_residual(planning_from_base_residual(base))
    validate_base_envelope(recovered)


def _attempt11_pair():
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message

    epoch = deserialize_message(
        (ATTEMPT11 / 'epoch.cdr').read_bytes(), SceneEpoch)
    scene = deserialize_message(
        (ATTEMPT11 / 'scene.cdr').read_bytes(), PlanningScene)
    return epoch, scene


def test_unmodified_attempt11_recovers_nominal_pelvis_geometry():
    """The protected attempt11 pair recovers the nominal operating frame."""
    epoch, scene = _attempt11_pair()
    result = recover_scene_alignment(epoch, scene)
    assert np.allclose(
        result['base_residual'], np.eye(4), atol=1.0e-12, rtol=0.0)
    assert np.allclose(
        result['cabinet_delta'], np.eye(4), atol=1.0e-12, rtol=0.0)
    assert np.allclose(
        result['cube_delta'], np.eye(4), atol=1.0e-12, rtol=0.0)
    assert result['handle_translation_error_m'] < 1.0e-12
    assert result['handle_rotation_error_rad'] < 1.0e-12


def test_attempt11_single_object_pose_tamper_is_rejected():
    """A millimetre scene-only door mutation violates common rigidity."""
    epoch, scene = _attempt11_pair()
    changed = copy.deepcopy(scene)
    door = next(
        value for value in changed.world.collision_objects
        if value.id == 'incubator_door_exact')
    door.pose.position.x += 0.001
    with pytest.raises(ValueError, match='SCENE_EPOCH_RIGID_TRANSLATION'):
        recover_scene_alignment(epoch, changed)


def test_tag11_in_gate_noise_checks_door_but_does_not_drive_scene():
    """A 1 mm Tag 11 variation is recorded, never used as door authority."""
    epoch, scene = _attempt11_pair()
    changed = copy.deepcopy(epoch)
    changed.door.pose_world.pose.position.x += 0.001
    result = recover_scene_alignment(changed, scene)
    assert result['door_tag_translation_error_m'] == pytest.approx(0.001)
    assert np.allclose(
        result['world']['door'],
        recover_scene_alignment(epoch, scene)['world']['door'],
        atol=1.0e-12, rtol=0.0)


def test_tag11_out_of_gate_pose_fails_before_planning():
    epoch, scene = _attempt11_pair()
    changed = copy.deepcopy(epoch)
    changed.door.pose_world.pose.position.y += 0.005001
    with pytest.raises(ValueError, match='DOOR_TAG_TRANSLATION'):
        recover_scene_alignment(changed, scene)


def test_base_matrix_moves_scene_but_not_epoch_world_poses():
    """Derived base cases keep tag world poses byte-for-byte semantic."""
    from rclpy.serialization import serialize_message

    epoch, scene = _attempt11_pair()
    original_cabinet = copy.deepcopy(epoch.cabinet.pose_world.pose)
    original_table = next(
        value for value in scene.world.collision_objects
        if value.id == 'environment_table_10mm_margin')
    original_table_pose = copy.deepcopy(original_table.pose)
    entry = {'case': 'unit_base_xy_yaw', 'base_x_mm': 0.0004,
             'base_y_mm': -0.0004, 'base_yaw_deg': 0.00005}
    derived_epoch, derived_scene, requested, _ = derive_case(
        epoch, scene, entry, 99, serialize_message)
    assert derived_epoch.cabinet.pose_world.pose == original_cabinet
    changed_table = next(
        value for value in derived_scene.world.collision_objects
        if value.id == 'environment_table_10mm_margin')
    assert changed_table.pose != original_table_pose
    recovered = recover_scene_alignment(derived_epoch, derived_scene)
    assert np.allclose(
        recovered['base_residual'], requested, atol=1.0e-12, rtol=0.0)
