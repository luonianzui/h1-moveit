"""Pure fail-closed contract for navigation-to-visual alignment handoff."""

from __future__ import annotations

import hashlib
import json
import math

import numpy as np

from .candidate3_alignment import (
    handle_pose_from_cabinet,
    rotation_angle,
    validate_door_tag_cross_check,
    validate_se3,
)
from .candidate3_contract import yaw_transform


SCHEMA = 'h1_candidate3_post_nav_alignment/v1'
PASS_REASON = 'INCUBATOR_STAGING_GOAL_LONG_HOLD'
REQUIRED_CABINET_TAG_IDS = (10, 12)
REQUIRED_DOOR_TAG_IDS = (11,)
MINIMUM_HOLD_S = 30.0
MAXIMUM_GOAL_POSITION_ERROR_M = 0.08
MAXIMUM_GOAL_YAW_ERROR_RAD = 0.20
MINIMUM_UP_Z = 0.98
MINIMUM_HEIGHT_M = 0.95
MAXIMUM_OPEN_BASE_X_M = 0.001
MAXIMUM_OPEN_BASE_Y_M = 0.001
MAXIMUM_OPEN_BASE_Z_M = 0.001
MAXIMUM_OPEN_BASE_YAW_RAD = math.radians(0.1)
MAXIMUM_OPEN_BASE_TILT_RAD = math.radians(0.05)


def canonical_bytes(value):
    """Encode one certificate deterministically and reject NaN/Inf."""
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
        allow_nan=False).encode('utf-8')


def canonical_hash(value):
    """Return the deterministic SHA-256 for a JSON-compatible value."""
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'NAVIGATION_{label}') from error
    if not math.isfinite(result):
        raise ValueError(f'NAVIGATION_{label}')
    return result


def _vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'NAVIGATION_{label}')
    return tuple(_finite(item, label) for item in value)


def _angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def validate_navigation_hold_status(
        payload, expected_goal_pose=None,
        minimum_hold_s=MINIMUM_HOLD_S,
        maximum_position_error_m=MAXIMUM_GOAL_POSITION_ERROR_M,
        maximum_yaw_error_rad=MAXIMUM_GOAL_YAW_ERROR_RAD,
        minimum_up_z=MINIMUM_UP_Z,
        minimum_height_m=MINIMUM_HEIGHT_M):
    """Validate the continuously monitored Nav2 stop-and-hold result."""
    if not isinstance(payload, dict):
        raise ValueError('NAVIGATION_STATUS_TYPE')
    if payload.get('state') != 'PASS':
        raise ValueError('NAVIGATION_NOT_PASS')
    if payload.get('reason') != PASS_REASON:
        raise ValueError('NAVIGATION_PASS_REASON')
    if payload.get('gazebo_only') is not True:
        raise ValueError('NAVIGATION_NOT_GAZEBO_ONLY')

    goal = _vector(payload.get('goal_pose'), 3, 'GOAL_POSE')
    localized = _vector(payload.get('latest_pose'), 3, 'LATEST_POSE')
    root = _vector(
        payload.get('latest_root_position'), 2, 'LATEST_ROOT_POSITION')
    if expected_goal_pose is not None:
        expected = _vector(expected_goal_pose, 3, 'EXPECTED_GOAL_POSE')
        if (math.hypot(goal[0] - expected[0], goal[1] - expected[1]) >
                1.0e-9 or
                abs(_angle_difference(goal[2], expected[2])) > 1.0e-9):
            raise ValueError('NAVIGATION_UNEXPECTED_GOAL')

    localized_error = math.hypot(
        localized[0] - goal[0], localized[1] - goal[1])
    physical_error = math.hypot(root[0] - goal[0], root[1] - goal[1])
    reported_localized_error = _finite(
        payload.get('localized_goal_error_m'), 'LOCALIZED_GOAL_ERROR')
    reported_physical_error = _finite(
        payload.get('physical_goal_error_m'), 'PHYSICAL_GOAL_ERROR')
    if not math.isclose(
            localized_error, reported_localized_error,
            abs_tol=1.0e-9, rel_tol=0.0):
        raise ValueError('NAVIGATION_LOCALIZED_ERROR_INCONSISTENT')
    if not math.isclose(
            physical_error, reported_physical_error,
            abs_tol=1.0e-9, rel_tol=0.0):
        raise ValueError('NAVIGATION_PHYSICAL_ERROR_INCONSISTENT')
    if (localized_error > float(maximum_position_error_m) or
            physical_error > float(maximum_position_error_m)):
        raise ValueError('NAVIGATION_POSITION_ENVELOPE')
    yaw_error = abs(_angle_difference(localized[2], goal[2]))
    if yaw_error > float(maximum_yaw_error_rad):
        raise ValueError('NAVIGATION_YAW_ENVELOPE')

    hold_duration = _finite(
        payload.get('long_hold_duration_s'), 'HOLD_DURATION')
    hold_elapsed = _finite(
        payload.get('long_hold_elapsed_s'), 'HOLD_ELAPSED')
    if (hold_duration < float(minimum_hold_s) or
            hold_elapsed < hold_duration):
        raise ValueError('NAVIGATION_HOLD_DURATION')
    up_z = _finite(payload.get('latest_up_z'), 'UP_Z')
    height = _finite(payload.get('latest_height'), 'HEIGHT')
    if up_z < float(minimum_up_z):
        raise ValueError('NAVIGATION_UPRIGHT')
    if height < float(minimum_height_m):
        raise ValueError('NAVIGATION_HEIGHT')

    lifecycle = payload.get('nav2_lifecycle_states')
    if (not isinstance(lifecycle, dict) or not lifecycle or
            any(int(value) != 3 for value in lifecycle.values())):
        raise ValueError('NAVIGATION_LIFECYCLE')
    normalized = dict(payload)
    normalized['goal_pose'] = list(goal)
    normalized['latest_pose'] = list(localized)
    normalized['latest_root_position'] = list(root)
    return {
        'status_hash': canonical_hash(normalized),
        'goal_pose': goal,
        'localized_pose': localized,
        'root_position': root,
        'localized_error_m': localized_error,
        'physical_error_m': physical_error,
        'yaw_error_rad': yaw_error,
        'hold_duration_s': hold_duration,
        'hold_elapsed_s': hold_elapsed,
        'up_z': up_z,
        'height_m': height,
    }


def validate_tag_roles(cabinet, door, expected_config_hash):
    """Require Tag 10/12 for cabinet authority and Tag 11 for checking."""
    for name, value, expected_ids in (
            ('CABINET', cabinet, REQUIRED_CABINET_TAG_IDS),
            ('DOOR', door, REQUIRED_DOOR_TAG_IDS)):
        if not value.valid or not value.stable:
            raise ValueError(f'{name}_TAG_NOT_STABLE')
        if value.config_hash != expected_config_hash:
            raise ValueError(f'{name}_TAG_CONFIG_HASH')
        actual_ids = tuple(sorted(int(item)
                                  for item in value.contributing_tag_ids))
        if actual_ids != expected_ids:
            raise ValueError(f'{name}_TAG_ROLE')


def validate_open_base_envelope(base_residual):
    """Gate the provisional right-open residual domain before IK/audit."""
    value = validate_se3(base_residual, 'OPEN_BASE_RESIDUAL')
    yaw = math.atan2(float(value[1, 0]), float(value[0, 0]))
    yaw_only = yaw_transform([0.0, 0.0, 0.0], yaw)
    tilt = rotation_angle(
        yaw_only @ np.block([
            [value[:3, :3].T, np.zeros((3, 1))],
            [np.zeros((1, 3)), np.ones((1, 1))],
        ]))
    errors = []
    for axis, limit, label in (
            (0, MAXIMUM_OPEN_BASE_X_M, 'OPEN_BASE_X_ENVELOPE'),
            (1, MAXIMUM_OPEN_BASE_Y_M, 'OPEN_BASE_Y_ENVELOPE'),
            (2, MAXIMUM_OPEN_BASE_Z_M, 'OPEN_BASE_Z_ENVELOPE')):
        if abs(float(value[axis, 3])) > limit + 1.0e-12:
            errors.append(label)
    if abs(yaw) > MAXIMUM_OPEN_BASE_YAW_RAD + 1.0e-12:
        errors.append('OPEN_BASE_YAW_ENVELOPE')
    if tilt > MAXIMUM_OPEN_BASE_TILT_RAD + 1.0e-12:
        errors.append('OPEN_BASE_TILT_ENVELOPE')
    if errors:
        raise ValueError(','.join(errors))
    return {
        'x_m': float(value[0, 3]),
        'y_m': float(value[1, 3]),
        'z_m': float(value[2, 3]),
        'yaw_rad': yaw,
        'tilt_rad': tilt,
        'contract': 'post_nav_right_handle_open/v1',
    }


def build_alignment_certificate(
        navigation_status, world_cabinet, observed_world_door,
        door_hinge_angle_rad, handle_latch_angle_rad, planning_from_world,
        tag_config_hash, cabinet_stamp_ns, door_stamp_ns):
    """Bind navigation, tag check, and the fixed-offset handle target."""
    navigation = validate_navigation_hold_status(navigation_status)
    cabinet = validate_se3(world_cabinet, 'WORLD_CABINET')
    door_check = validate_door_tag_cross_check(
        cabinet, observed_world_door, door_hinge_angle_rad)
    handle = handle_pose_from_cabinet(
        cabinet, door_hinge_angle_rad, handle_latch_angle_rad)
    planning = validate_se3(planning_from_world, 'PLANNING_FROM_WORLD')
    payload = {
        'schema': SCHEMA,
        'status': 'PASS',
        'navigation_status_hash': navigation['status_hash'],
        'tag_config_hash': str(tag_config_hash),
        'cabinet_tag_ids': list(REQUIRED_CABINET_TAG_IDS),
        'door_check_tag_ids': list(REQUIRED_DOOR_TAG_IDS),
        'cabinet_stamp_ns': int(cabinet_stamp_ns),
        'door_stamp_ns': int(door_stamp_ns),
        'door_hinge_angle_rad': float(door_hinge_angle_rad),
        'handle_latch_angle_rad': float(handle_latch_angle_rad),
        'door_tag_translation_error_m':
            door_check['translation_error_m'],
        'door_tag_rotation_error_rad': door_check['rotation_error_rad'],
        'world_cabinet': cabinet.tolist(),
        'world_door_from_cabinet':
            door_check['predicted_world_door'].tolist(),
        'world_door_tag11_observation':
            door_check['observed_world_door'].tolist(),
        'world_handle_from_cabinet': handle.tolist(),
        'planning_from_world': planning.tolist(),
        'planning_handle_target': (planning @ handle).tolist(),
    }
    return payload, canonical_hash(payload)
