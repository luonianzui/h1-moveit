"""Pure tests for the Nav2-stop to AprilTag-handle handoff."""

import math
from types import SimpleNamespace

from h1_2_vision.candidate3_alignment import (
    door_pose_from_cabinet,
    handle_pose_from_cabinet,
)
from h1_2_vision.candidate3_contract import yaw_transform
from h1_2_vision.candidate3_post_nav_contract import (
    build_alignment_certificate,
    validate_navigation_hold_status,
    validate_tag_roles,
)
import numpy as np
import pytest


def navigation_pass():
    return {
        'state': 'PASS',
        'reason': 'INCUBATOR_STAGING_GOAL_LONG_HOLD',
        'gazebo_only': True,
        'goal_pose': [0.0, 0.0, 0.0],
        'latest_pose': [0.012, -0.009, 0.03],
        'latest_root_position': [0.010, -0.008],
        'localized_goal_error_m': math.hypot(0.012, -0.009),
        'physical_goal_error_m': math.hypot(0.010, -0.008),
        'latest_up_z': 0.995,
        'latest_height': 1.02,
        'long_hold_duration_s': 30.0,
        'long_hold_elapsed_s': 31.0,
        'nav2_lifecycle_states': {
            'amcl': 3,
            'planner_server': 3,
            'controller_server': 3,
            'behavior_server': 3,
            'bt_navigator': 3,
        },
    }


def test_navigation_pass_is_bound_and_nonfinite_or_short_hold_fails():
    result = validate_navigation_hold_status(navigation_pass())
    assert len(result['status_hash']) == 64
    assert result['hold_elapsed_s'] == 31.0

    changed = navigation_pass()
    changed['long_hold_elapsed_s'] = 29.999
    with pytest.raises(ValueError, match='NAVIGATION_HOLD_DURATION'):
        validate_navigation_hold_status(changed)
    changed = navigation_pass()
    changed['latest_up_z'] = math.nan
    with pytest.raises(ValueError, match='NAVIGATION_UP_Z'):
        validate_navigation_hold_status(changed)


def test_navigation_reported_pose_error_must_match_pose():
    changed = navigation_pass()
    changed['localized_goal_error_m'] += 0.001
    with pytest.raises(
            ValueError, match='NAVIGATION_LOCALIZED_ERROR_INCONSISTENT'):
        validate_navigation_hold_status(changed)


def test_tag_roles_are_not_interchangeable():
    cabinet = SimpleNamespace(
        valid=True, stable=True, config_hash='a' * 64,
        contributing_tag_ids=[12, 10])
    door = SimpleNamespace(
        valid=True, stable=True, config_hash='a' * 64,
        contributing_tag_ids=[11])
    validate_tag_roles(cabinet, door, 'a' * 64)
    door.contributing_tag_ids = [10]
    with pytest.raises(ValueError, match='DOOR_TAG_ROLE'):
        validate_tag_roles(cabinet, door, 'a' * 64)


def test_tag11_noise_does_not_move_fixed_offset_handle_target():
    cabinet = yaw_transform([0.590, 0.0, 0.815], math.radians(0.4))
    predicted_door = door_pose_from_cabinet(cabinet, 0.0)
    observed_a = predicted_door.copy()
    observed_b = predicted_door.copy()
    observed_b[0, 3] += 0.001
    planning = yaw_transform([0.01, -0.02, -1.0], math.radians(-0.2))
    first, _ = build_alignment_certificate(
        navigation_pass(), cabinet, observed_a, 0.0, 0.0, planning,
        'a' * 64, 100, 100)
    second, _ = build_alignment_certificate(
        navigation_pass(), cabinet, observed_b, 0.0, 0.0, planning,
        'a' * 64, 100, 100)
    assert np.array_equal(
        first['planning_handle_target'],
        second['planning_handle_target'])
    assert first['door_tag_translation_error_m'] == 0.0
    assert second['door_tag_translation_error_m'] == pytest.approx(0.001)
    expected = planning @ handle_pose_from_cabinet(cabinet, 0.0, 0.0)
    assert np.allclose(
        first['planning_handle_target'], expected,
        atol=1.0e-12, rtol=0.0)


def test_tag11_outside_rigid_door_gate_fails_closed():
    cabinet = yaw_transform([0.590, 0.0, 0.815], 0.0)
    observed = door_pose_from_cabinet(cabinet, 0.0)
    observed[1, 3] += 0.005001
    with pytest.raises(ValueError, match='DOOR_TAG_TRANSLATION'):
        build_alignment_certificate(
            navigation_pass(), cabinet, observed, 0.0, 0.0, np.eye(4),
            'a' * 64, 100, 100)


def test_base_residual_changes_planning_target_not_world_handle():
    cabinet = yaw_transform([0.590, 0.0, 0.815], 0.0)
    door = door_pose_from_cabinet(cabinet, 0.0)
    first, _ = build_alignment_certificate(
        navigation_pass(), cabinet, door, 0.0, 0.0, np.eye(4),
        'a' * 64, 100, 100)
    planning = yaw_transform([-0.006, 0.004, 0.0], math.radians(-0.5))
    second, _ = build_alignment_certificate(
        navigation_pass(), cabinet, door, 0.0, 0.0, planning,
        'a' * 64, 100, 100)
    assert np.array_equal(
        first['world_handle_from_cabinet'],
        second['world_handle_from_cabinet'])
    assert not np.allclose(
        first['planning_handle_target'],
        second['planning_handle_target'], atol=1.0e-9, rtol=0.0)
