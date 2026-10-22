"""Fail-closed post-navigation scene alignment for candidate-3 planning."""

from __future__ import annotations

import math

import numpy as np

from .candidate3_contract import pose_matrix, yaw_transform


REFERENCE_CABINET = yaw_transform([0.590, 0.0, 0.815], 0.0)
REFERENCE_CUBE = yaw_transform([0.334, 0.377, 0.830], math.pi / 2)
REFERENCE_PELVIS_FROM_WORLD = yaw_transform([0.0, 0.0, -1.0], 0.0)
CABINET_DOOR_CLOSED = yaw_transform(
    [-0.140045922147, -0.247963042272, 0.020], 0.0)

MAX_BASE_X_M = 0.5e-6
MAX_BASE_Y_M = 0.5e-6
MAX_BASE_Z_M = 0.5e-6
MAX_BASE_YAW_RAD = math.radians(0.0001)
MAX_BASE_TILT_RAD = math.radians(0.00001)
ENVELOPE_TRANSLATION_EPSILON_M = 1.0e-12
ENVELOPE_ROTATION_EPSILON_RAD = 1.0e-12
RIGID_TRANSLATION_TOLERANCE_M = 1.0e-6
RIGID_ROTATION_TOLERANCE_RAD = 1.0e-6
MAXIMUM_OBJECT_STAMP_SKEW_S = 0.10
DOOR_TAG_TRANSLATION_TOLERANCE_M = 0.005
DOOR_TAG_ROTATION_TOLERANCE_RAD = math.radians(2.0)

DYNAMIC_OBJECT_IDS = (
    'incubator_body_exact',
    'incubator_door_exact',
    'incubator_handle_exact',
    'green_cube_dynamic_exact',
    'green_cube_10mm_guard',
)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation, dtype=float)
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


BODY_VISUAL = _transform(
    _rpy_matrix(1.570796326795, 0.0, -1.570796326795),
    [0.189890542898, 0.250472606667, 0.0])
TRAY_VISUAL = _transform(
    _rpy_matrix(1.570796326795, 0.0, -1.570796326795),
    [0.151890542898, 0.146472606667, 0.160407057555])
DOOR_VISUAL = _transform(
    _rpy_matrix(1.570796326795, 0.0, -1.528168312093),
    [0.005592142864, 0.408609505443, 0.0])
DOOR_HANDLE = _transform(
    _rpy_matrix(0.0, 0.0, 1.613424341497),
    [-0.019105784513, 0.377528763146, 0.189999998514])
HANDLE_VISUAL = _transform(
    np.eye(3), [-0.009999668689, 0.000027284668, -0.119003896682])


def rotation_angle(value):
    """Return the principal rotation angle of an SE(3) matrix."""
    cosine = (float(np.trace(value[:3, :3])) - 1.0) * 0.5
    return math.acos(min(1.0, max(-1.0, cosine)))


def validate_se3(value, label='SE3'):
    """Return a finite, proper homogeneous transform or fail closed."""
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4):
        raise ValueError(f'{label}:SHAPE')
    if not np.isfinite(result).all():
        raise ValueError(f'{label}:NONFINITE')
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0],
                       atol=1.0e-12, rtol=0.0):
        raise ValueError(f'{label}:HOMOGENEOUS_ROW')
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3),
                       atol=1.0e-8, rtol=0.0):
        raise ValueError(f'{label}:ROTATION_ORTHONORMAL')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0,
                        abs_tol=1.0e-8, rel_tol=0.0):
        raise ValueError(f'{label}:ROTATION_DETERMINANT')
    return result


def transform_error(first, second):
    """Return translation and rotation error between two transforms."""
    delta = validate_se3(first, 'FIRST') @ np.linalg.inv(
        validate_se3(second, 'SECOND'))
    return float(np.linalg.norm(delta[:3, 3])), rotation_angle(delta)


def door_pose_from_cabinet(world_cabinet, door_hinge_angle_rad):
    """Derive the door root from the cabinet root and rigid hinge model."""
    cabinet = validate_se3(world_cabinet, 'WORLD_CABINET')
    angle = float(door_hinge_angle_rad)
    if not math.isfinite(angle):
        raise ValueError('DOOR_HINGE_ANGLE:NONFINITE')
    hinge_rotation = yaw_transform([0.0, 0.0, 0.0], angle)
    return validate_se3(
        cabinet @ CABINET_DOOR_CLOSED @ hinge_rotation,
        'WORLD_DOOR_FROM_CABINET')


def handle_pose_from_cabinet(
        world_cabinet, door_hinge_angle_rad, handle_latch_angle_rad):
    """Derive the handle root using only cabinet pose and rigid offsets."""
    handle_angle = float(handle_latch_angle_rad)
    if not math.isfinite(handle_angle):
        raise ValueError('HANDLE_LATCH_ANGLE:NONFINITE')
    return validate_se3(
        door_pose_from_cabinet(world_cabinet, door_hinge_angle_rad) @
        DOOR_HANDLE @
        _transform(
            _rpy_matrix(0.0, handle_angle, 0.0), [0.0, 0.0, 0.0]),
        'WORLD_HANDLE_FROM_CABINET')


def validate_door_tag_cross_check(
        world_cabinet, observed_world_door, door_hinge_angle_rad,
        translation_tolerance_m=DOOR_TAG_TRANSLATION_TOLERANCE_M,
        rotation_tolerance_rad=DOOR_TAG_ROTATION_TOLERANCE_RAD):
    """Check Tag 11 without allowing it to become the door authority."""
    translation_tolerance_m = float(translation_tolerance_m)
    rotation_tolerance_rad = float(rotation_tolerance_rad)
    if (not math.isfinite(translation_tolerance_m) or
            translation_tolerance_m < 0.0 or
            not math.isfinite(rotation_tolerance_rad) or
            rotation_tolerance_rad < 0.0):
        raise ValueError('DOOR_TAG_TOLERANCE')
    predicted = door_pose_from_cabinet(
        world_cabinet, door_hinge_angle_rad)
    observed = validate_se3(observed_world_door, 'WORLD_DOOR_TAG11')
    translation, rotation = transform_error(observed, predicted)
    if translation > translation_tolerance_m:
        raise ValueError(
            f'DOOR_TAG_TRANSLATION:{translation:.9g}')
    if rotation > rotation_tolerance_rad:
        raise ValueError(f'DOOR_TAG_ROTATION:{rotation:.9g}')
    return {
        'predicted_world_door': predicted,
        'observed_world_door': observed,
        'translation_error_m': translation,
        'rotation_error_rad': rotation,
    }


def require_common_rigid_transform(candidates):
    """Select one transform only when every independent chain agrees."""
    if not candidates:
        raise ValueError('SCENE_RIGID:NO_CANDIDATES')
    normalized = {
        name: validate_se3(value, f'SCENE_RIGID:{name}')
        for name, value in candidates.items()
    }
    reference_name = 'body' if 'body' in normalized else next(iter(normalized))
    reference = normalized[reference_name]
    for name, value in normalized.items():
        translation, rotation = transform_error(value, reference)
        if translation > RIGID_TRANSLATION_TOLERANCE_M:
            raise ValueError(
                f'SCENE_EPOCH_RIGID_TRANSLATION:{name}:{translation:.9g}')
        if rotation > RIGID_ROTATION_TOLERANCE_RAD:
            raise ValueError(
                f'SCENE_EPOCH_RIGID_ROTATION:{name}:{rotation:.9g}')
    return reference.copy()


def base_pose_residual(planning_from_world):
    """Recover world-frame base motion from the measured planning transform."""
    planning_from_world = validate_se3(
        planning_from_world, 'PLANNING_FROM_WORLD')
    return validate_se3(
        np.linalg.inv(planning_from_world) @ REFERENCE_PELVIS_FROM_WORLD,
        'BASE_POSE_RESIDUAL')


def planning_from_base_residual(base_residual):
    """Convert a world-frame base residual to pelvis<-world."""
    base_residual = validate_se3(base_residual, 'BASE_POSE_RESIDUAL')
    return validate_se3(
        REFERENCE_PELVIS_FROM_WORLD @ np.linalg.inv(base_residual),
        'PLANNING_FROM_WORLD')


def validate_base_envelope(base_residual):
    """Enforce the symmetric full-plan-certified residual envelope."""
    value = validate_se3(base_residual, 'BASE_POSE_RESIDUAL')
    errors = []
    if abs(float(value[0, 3])) > (
            MAX_BASE_X_M + ENVELOPE_TRANSLATION_EPSILON_M):
        errors.append('BASE_X_ENVELOPE')
    if abs(float(value[1, 3])) > (
            MAX_BASE_Y_M + ENVELOPE_TRANSLATION_EPSILON_M):
        errors.append('BASE_Y_ENVELOPE')
    if abs(float(value[2, 3])) > (
            MAX_BASE_Z_M + ENVELOPE_TRANSLATION_EPSILON_M):
        errors.append('BASE_Z_ENVELOPE')
    rotation = value[:3, :3]
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    yaw_only = yaw_transform([0.0, 0.0, 0.0], yaw)
    tilt = rotation_angle(
        yaw_only @ _transform(rotation.T, [0.0, 0.0, 0.0]))
    if abs(yaw) > MAX_BASE_YAW_RAD + ENVELOPE_ROTATION_EPSILON_RAD:
        errors.append('BASE_YAW_ENVELOPE')
    if tilt > MAX_BASE_TILT_RAD + ENVELOPE_ROTATION_EPSILON_RAD:
        errors.append('BASE_TILT_ENVELOPE')
    if errors:
        raise ValueError(','.join(errors))
    return {
        'x_m': float(value[0, 3]),
        'y_m': float(value[1, 3]),
        'z_m': float(value[2, 3]),
        'yaw_rad': yaw,
        'tilt_rad': tilt,
    }


def validate_world_object_envelope(epoch):
    """Preserve the stage-I world-object movement gates independently."""
    cabinet = validate_se3(
        pose_matrix(epoch.cabinet.pose_world.pose), 'WORLD_CABINET')
    door = validate_se3(
        pose_matrix(epoch.door.pose_world.pose), 'WORLD_DOOR')
    cube = validate_se3(
        pose_matrix(epoch.cube.pose_world.pose), 'WORLD_CUBE')
    cabinet_delta = cabinet @ np.linalg.inv(REFERENCE_CABINET)
    cube_delta = cube @ np.linalg.inv(REFERENCE_CUBE)
    cabinet_translation = cabinet[:3, 3] - REFERENCE_CABINET[:3, 3]
    cube_translation = cube[:3, 3] - REFERENCE_CUBE[:3, 3]
    errors = []
    if abs(cabinet_translation[0]) > 0.035:
        errors.append('CABINET_X_ENVELOPE')
    if abs(cabinet_translation[1]) > 0.035:
        errors.append('CABINET_Y_ENVELOPE')
    if abs(cabinet_translation[2]) > 0.010:
        errors.append('CABINET_Z_ENVELOPE')
    if rotation_angle(cabinet_delta) > math.radians(5.5):
        errors.append('CABINET_ROTATION_ENVELOPE')
    if abs(cube_translation[0]) > 0.060 or abs(cube_translation[1]) > 0.060:
        errors.append('CUBE_XY_ENVELOPE')
    if abs(cube_translation[2]) > 0.015:
        errors.append('CUBE_Z_ENVELOPE')
    if rotation_angle(cube_delta) > math.radians(21.0):
        errors.append('CUBE_ROTATION_ENVELOPE')
    if errors:
        raise ValueError(','.join(errors))
    return {
        'cabinet': cabinet,
        'door': door,
        'cube': cube,
        'cabinet_delta': cabinet_delta,
        'cube_delta': cube_delta,
    }


def _uint8(value):
    if isinstance(value, bytes):
        if len(value) != 1:
            raise ValueError('COLLISION_OBJECT_OPERATION')
        return value[0]
    return int(value)


def _require_topology(value, object_id, meshes, primitives):
    if value.header.frame_id != 'pelvis':
        raise ValueError(f'SCENE_OBJECT_FRAME:{object_id}')
    if _uint8(value.operation) != 0:
        raise ValueError(f'SCENE_OBJECT_OPERATION:{object_id}')
    if any((len(value.meshes) != meshes,
            len(value.mesh_poses) != meshes,
            len(value.primitives) != primitives,
            len(value.primitive_poses) != primitives,
            len(value.planes) != 0,
            len(value.plane_poses) != 0)):
        raise ValueError(f'SCENE_OBJECT_TOPOLOGY:{object_id}')


def _global_shape_pose(value, poses, index=0):
    return validate_se3(
        pose_matrix(value.pose) @ pose_matrix(poses[index]),
        f'SCENE_SHAPE:{value.id}:{index}')


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _validate_epoch_frames_and_stamps(epoch):
    values = (epoch.cabinet, epoch.door, epoch.cube)
    frames = {value.header.frame_id for value in values}
    if frames != {epoch.header.frame_id} or epoch.header.frame_id != 'world':
        raise ValueError('EPOCH_OBJECT_FRAME_BINDING')
    stamps = [_stamp_ns(value.header.stamp) for value in values]
    if min(stamps) < 0 or (
            max(stamps) - min(stamps) >
            int(MAXIMUM_OBJECT_STAMP_SKEW_S * 1.0e9)):
        raise ValueError('EPOCH_OBJECT_STAMP_SKEW')


def _mechanism_position(epoch, name):
    names = list(epoch.mechanism_state.name)
    if names.count(name) != 1:
        raise ValueError(f'MECHANISM_JOINT:{name}')
    index = names.index(name)
    if index >= len(epoch.mechanism_state.position):
        raise ValueError(f'MECHANISM_POSITION:{name}')
    value = float(epoch.mechanism_state.position[index])
    if not math.isfinite(value):
        raise ValueError(f'MECHANISM_NONFINITE:{name}')
    return value


def recover_scene_alignment(
        epoch, scene, base_envelope_validator=validate_base_envelope):
    """Recover pelvis-relative dynamic poses and planning deltas."""
    _validate_epoch_frames_and_stamps(epoch)
    world = validate_world_object_envelope(epoch)
    door_hinge_angle = _mechanism_position(epoch, 'door_hinge_joint')
    door_check = validate_door_tag_cross_check(
        world['cabinet'], world['door'], door_hinge_angle)
    world['door_tag_observation'] = world['door']
    world['door'] = door_check['predicted_world_door']
    objects = {}
    duplicates = []
    for value in scene.world.collision_objects:
        if value.id in DYNAMIC_OBJECT_IDS:
            if value.id in objects:
                duplicates.append(value.id)
            objects[value.id] = value
    if duplicates:
        raise ValueError(f'SCENE_OBJECT_DUPLICATE:{sorted(duplicates)}')
    missing = sorted(set(DYNAMIC_OBJECT_IDS) - set(objects))
    if missing:
        raise ValueError(f'SCENE_OBJECT_MISSING:{missing}')
    attached = {
        value.object.id
        for value in scene.robot_state.attached_collision_objects
    }
    forbidden = sorted(attached.intersection(DYNAMIC_OBJECT_IDS))
    if forbidden:
        raise ValueError(f'SCENE_DYNAMIC_OBJECT_ATTACHED:{forbidden}')

    body = objects['incubator_body_exact']
    door_object = objects['incubator_door_exact']
    handle_object = objects['incubator_handle_exact']
    cube = objects['green_cube_dynamic_exact']
    guard = objects['green_cube_10mm_guard']
    _require_topology(body, body.id, 2, 0)
    _require_topology(door_object, door_object.id, 1, 0)
    _require_topology(handle_object, handle_object.id, 1, 0)
    _require_topology(cube, cube.id, 0, 1)
    _require_topology(guard, guard.id, 0, 1)
    if tuple(float(x) for x in cube.primitives[0].dimensions) != (
            0.080, 0.160, 0.030):
        raise ValueError('SCENE_CUBE_DIMENSIONS')
    if tuple(float(x) for x in guard.primitives[0].dimensions) != (
            0.100, 0.180, 0.050):
        raise ValueError('SCENE_GUARD_DIMENSIONS')
    if _uint8(cube.primitives[0].type) != 1 or (
            _uint8(guard.primitives[0].type) != 1):
        raise ValueError('SCENE_CUBE_PRIMITIVE_TYPE')

    pelvis_cabinet = _global_shape_pose(
        body, body.mesh_poses, 0) @ np.linalg.inv(BODY_VISUAL)
    pelvis_cabinet_tray = _global_shape_pose(
        body, body.mesh_poses, 1) @ np.linalg.inv(TRAY_VISUAL)
    pelvis_door = _global_shape_pose(
        door_object, door_object.mesh_poses) @ np.linalg.inv(DOOR_VISUAL)
    pelvis_handle = _global_shape_pose(
        handle_object, handle_object.mesh_poses) @ np.linalg.inv(HANDLE_VISUAL)
    pelvis_cube = _global_shape_pose(cube, cube.primitive_poses)
    pelvis_guard = _global_shape_pose(guard, guard.primitive_poses)

    candidates = {
        'body': pelvis_cabinet @ np.linalg.inv(world['cabinet']),
        'tray': pelvis_cabinet_tray @ np.linalg.inv(world['cabinet']),
        'door': pelvis_door @ np.linalg.inv(world['door']),
        'cube': pelvis_cube @ np.linalg.inv(world['cube']),
        'guard': pelvis_guard @ np.linalg.inv(world['cube']),
    }
    planning_from_world = require_common_rigid_transform(candidates)
    handle_angle = _mechanism_position(epoch, 'handle_latch_joint')
    expected_handle = planning_from_world @ handle_pose_from_cabinet(
        world['cabinet'], door_hinge_angle, handle_angle)
    handle_translation, handle_rotation = transform_error(
        pelvis_handle, expected_handle)
    if handle_translation > RIGID_TRANSLATION_TOLERANCE_M:
        raise ValueError(
            f'SCENE_HANDLE_RIGID_TRANSLATION:{handle_translation:.9g}')
    if handle_rotation > RIGID_ROTATION_TOLERANCE_RAD:
        raise ValueError(
            f'SCENE_HANDLE_RIGID_ROTATION:{handle_rotation:.9g}')

    base_residual = base_pose_residual(planning_from_world)
    if base_envelope_validator is None:
        raise ValueError('BASE_ENVELOPE_VALIDATOR_REQUIRED')
    base_metrics = base_envelope_validator(base_residual)
    nominal_pelvis_cabinet = (
        REFERENCE_PELVIS_FROM_WORLD @ REFERENCE_CABINET)
    nominal_pelvis_cube = REFERENCE_PELVIS_FROM_WORLD @ REFERENCE_CUBE
    cabinet_delta = validate_se3(
        pelvis_cabinet @ np.linalg.inv(nominal_pelvis_cabinet),
        'CABINET_PLANNING_DELTA')
    cube_delta = validate_se3(
        pelvis_cube @ np.linalg.inv(nominal_pelvis_cube),
        'CUBE_PLANNING_DELTA')
    return {
        'world': world,
        'planning_from_world': planning_from_world,
        'base_residual': base_residual,
        'base_metrics': base_metrics,
        'pelvis_cabinet': pelvis_cabinet,
        'pelvis_door': pelvis_door,
        'pelvis_handle': pelvis_handle,
        'pelvis_cube': pelvis_cube,
        'cabinet_delta': cabinet_delta,
        'cube_delta': cube_delta,
        'rigid_candidates': candidates,
        'door_tag_translation_error_m':
            door_check['translation_error_m'],
        'door_tag_rotation_error_rad': door_check['rotation_error_rad'],
        'handle_translation_error_m': handle_translation,
        'handle_rotation_error_rad': handle_rotation,
    }
