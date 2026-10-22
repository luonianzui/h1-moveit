"""Offline right-hand handle/open replanner derived from sealed v9 inputs."""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .candidate3_alignment import recover_scene_alignment
from .candidate3_contract import canonical_hash, quaternion_from_matrix
from .candidate3_post_nav_contract import validate_open_base_envelope
from .candidate3_worker import moveit_configuration, rotation_angle, rotation_vector


SCHEMA = 'h1_candidate3_post_nav_right_open_plan/v1'
CERTIFICATE_SCHEMA = 'h1_candidate3_post_nav_right_open_certificate/v1'
PLANNER_VERSION = 'h1_candidate3_post_nav_right_open/826-a6'
AUDIT_TOPOLOGY = 'post_nav_right_handle_open_roundtrip_audit/v1'
EXECUTOR_SHA256 = (
    '7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8')
MANIFEST_SHA256 = (
    '6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c')
MIDDLE_PLAN_SHA256 = (
    'cb20f8303cb2e75a5fe45025ea4a119fa5ba81054c8cda55442d320052d1634e')
MIDDLE_PLAN_RELATIVE = Path(
    'runtime_assets/candidate3_middle_e0_828/candidate3_middle_plan_828.json')
DERIVED_INGRESS_RELATIVE = Path(
    'runtime_evidence/826_post_nav_apriltag_alignment_20260914/'
    'right_hand_fixed_offset_open_a1/'
    'ingress_search_clearance10_seed827/search.json')
DERIVED_INGRESS_SHA256 = (
    'c76fb446f16fef81276d396d5d3ad5a4da5d2a23972aacb34d898b1214b112ed')
EXECUTOR_NAME = 'h1_right_handle_executor_828.py'
MANIFEST_NAME = 'controller_execution_authorization.json'
STAGE_NAMES = (
    'right_ingress_open_to_handle',
    'right_handle_close_and_unlock',
    'right_door_open_supported_601',
    'audit_reverse_right_door_close_601',
    'audit_reverse_right_handle_lock_and_open',
    'audit_reverse_right_egress',
)
EXECUTION_STAGE_COUNT = 3
RIGHT_ARM = (
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
)
LEFT_ARM = tuple(value.replace('right_', 'left_', 1) for value in RIGHT_ARM)
RIGHT_GRIPPER = (
    'right_dex1_finger_1_joint', 'right_dex1_finger_2_joint')
LEFT_GRIPPER = (
    'left_dex1_finger_1_joint', 'left_dex1_finger_2_joint')
IK_JOINTS = ('torso_joint', *RIGHT_ARM)
SOLVER_JOINTS = RIGHT_ARM
AUDIT_JOINTS = (
    'torso_joint', *LEFT_ARM, *RIGHT_ARM, *LEFT_GRIPPER, *RIGHT_GRIPPER)
ANGULAR_RESOLUTION_RAD = math.radians(0.49)
MAXIMUM_ADJACENT_ANGULAR_CHANGE_RAD = math.radians(4.9)
MAXIMUM_ADJACENT_GRIPPER_CHANGE_M = 0.001
MAXIMUM_ABSOLUTE_TORSO_RAD = math.radians(45.0)
INGRESS_CLEARANCE_RETREAT_X_M = 0.0
INGRESS_CLEARANCE_RETREAT_Y_M = 0.0
INGRESS_CLEARANCE_RETREAT_Z_M = 0.0
VELOCITY_LIMITS = {
    'torso_joint': 1.0,
    **dict(zip(LEFT_ARM, (1.0, 1.0, 1.2, 1.2, 1.5, 1.5, 1.5))),
    **dict(zip(RIGHT_ARM, (1.0, 1.0, 1.2, 1.2, 1.5, 1.5, 1.5))),
    **{name: 0.10 for name in (*LEFT_GRIPPER, *RIGHT_GRIPPER)},
}
ACCELERATION_LIMITS = {
    'torso_joint': 2.0,
    **dict(zip(LEFT_ARM, (2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0))),
    **dict(zip(RIGHT_ARM, (2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0))),
    **{name: 0.40 for name in (*LEFT_GRIPPER, *RIGHT_GRIPPER)},
}
_MOVEIT_KEEPALIVE = None


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def matrix_pose(value):
    from geometry_msgs.msg import Pose

    result = Pose()
    result.position.x, result.position.y, result.position.z = value[:3, 3]
    quaternion = quaternion_from_matrix(value[:3, :3])
    result.orientation.x, result.orientation.y = quaternion[:2]
    result.orientation.z, result.orientation.w = quaternion[2:]
    return result


def _load_module(path):
    if file_sha256(path) != EXECUTOR_SHA256:
        raise ValueError('V9_EXECUTOR_SHA256')
    name = 'h1_candidate3_sealed_right_handle_executor_828'
    specification = importlib.util.spec_from_file_location(name, path)
    if specification is None or specification.loader is None:
        raise ValueError('V9_EXECUTOR_IMPORT')
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def load_v9_sources(workspace, authority_root):
    """Validate every immutable v9 source before exposing any waypoints."""
    workspace = Path(workspace).resolve(strict=True)
    authority_root = Path(authority_root).resolve(strict=True)
    executor_path = authority_root / EXECUTOR_NAME
    manifest_path = authority_root / MANIFEST_NAME
    if file_sha256(manifest_path) != MANIFEST_SHA256:
        raise ValueError('V9_MANIFEST_SHA256')
    module = _load_module(executor_path)
    bundle = module.validate_bundle(
        workspace, manifest_path, MANIFEST_SHA256)
    prefix = module.build_stages(bundle, 'prefix')
    if tuple(value.name for value in prefix) != module.PREFIX_STAGE_ORDER:
        raise ValueError('V9_PREFIX_TOPOLOGY')

    middle_path = workspace / MIDDLE_PLAN_RELATIVE
    if file_sha256(middle_path) != MIDDLE_PLAN_SHA256:
        raise ValueError('V9_MIDDLE_PLAN_SHA256')
    middle = json.loads(middle_path.read_text(encoding='utf-8'))
    if (middle.get('schema') != 'h1_candidate3_middle_plan/828-e0-v1' or
            middle.get('offline_only') is not True or
            middle.get('controller_commands_issued') != 0):
        raise ValueError('V9_MIDDLE_PLAN_CONTRACT')
    phases = middle.get('phases')
    if (not isinstance(phases, list) or len(phases) != 11 or
            middle.get('phase_names') != [value.get('name') for value in phases]):
        raise ValueError('V9_MIDDLE_PHASES')
    door_phase = phases[1]
    commands = door_phase.get('commands')
    if (door_phase.get('name') != 'door_open_supported' or
            door_phase.get('commanded') is not True or
            door_phase.get('support') != {
                'cube_post': False, 'cube_pre': False,
                'door_post': True, 'door_pre': True} or
            not isinstance(commands, list) or len(commands) != 2):
        raise ValueError('V9_DOOR_OPEN_PHASE')
    torso, arm = commands
    if (torso.get('topic') != '/torso_controller/joint_trajectory' or
            torso.get('joint_names') != ['torso_joint'] or
            arm.get('topic') != '/right_arm_controller/joint_trajectory' or
            tuple(arm.get('joint_names', ())) != RIGHT_ARM or
            len(torso.get('points', ())) != 601 or
            len(arm.get('points', ())) != 601):
        raise ValueError('V9_DOOR_OPEN_COMMANDS')
    for index, (torso_point, arm_point) in enumerate(zip(
            torso['points'], arm['points'])):
        if (torso_point.get('time_from_start_ns') !=
                arm_point.get('time_from_start_ns') or
                len(torso_point.get('positions', ())) != 1 or
                len(arm_point.get('positions', ())) != 7):
            raise ValueError(f'V9_DOOR_OPEN_SYNCHRONIZATION:{index}')
    q0 = prefix[-1].states[0]
    q90 = prefix[-1].states[-1]
    if (abs(float(torso['points'][0]['positions'][0]) -
            float(q90['torso_joint'])) > 1.0e-12 or
            not np.allclose(
                arm['points'][0]['positions'],
                [q90[name] for name in RIGHT_ARM],
                atol=1.0e-12, rtol=0.0)):
        raise ValueError('V9_Q90_DOOR_SEAM')
    derived_path = workspace / DERIVED_INGRESS_RELATIVE
    if file_sha256(derived_path) != DERIVED_INGRESS_SHA256:
        raise ValueError('DERIVED_INGRESS_SHA256')
    derived = json.loads(derived_path.read_text(encoding='utf-8'))
    if (derived.get('schema') != 'h1_right_handle_splice_search/v1' or
            derived.get('status') != 'PASS' or
            derived.get('valid') is not True or
            derived.get('offline_only') is not True or
            derived.get('controller_commands_issued') != 0 or
            derived.get('path_sha256') != bundle.path.sha256 or
            derived.get('scene_cdr_sha256') != bundle.scene_sha256 or
            derived.get('minimum_environment_clearance_m') != 0.010 or
            derived.get('seed') != 827 or
            derived.get('waypoint_count') != 5 or
            tuple(derived.get('configuration_order', ())) != IK_JOINTS):
        raise ValueError('DERIVED_INGRESS_CONTRACT')
    waypoints = derived.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) != 5:
        raise ValueError('DERIVED_INGRESS_WAYPOINTS')
    derived_states = []
    base = dict(prefix[0].states[-1])
    for index, waypoint in enumerate(waypoints):
        if (not isinstance(waypoint, list) or len(waypoint) != 8 or
                not all(math.isfinite(float(value)) for value in waypoint)):
            raise ValueError(f'DERIVED_INGRESS_WAYPOINT:{index}')
        state = dict(base)
        state.update(zip(IK_JOINTS, map(float, waypoint)))
        derived_states.append(state)
    if (abs(derived_states[-1]['torso_joint'] - q0['torso_joint']) >
            1.0e-12 or
            not np.allclose(
                [derived_states[-1][name] for name in RIGHT_ARM],
                [q0[name] for name in RIGHT_ARM],
                atol=1.0e-12, rtol=0.0)):
        raise ValueError('DERIVED_INGRESS_Q0_SEAM')
    return module, bundle, prefix, door_phase, derived_states


def densify_states(states, names, resolution=ANGULAR_RESOLUTION_RAD):
    """Linearly densify source joints and retain a [0,1] path fraction."""
    if len(states) < 2:
        raise ValueError('SOURCE_PATH_TOO_SHORT')
    output = [(dict(states[0]), 0.0)]
    segment_count = len(states) - 1
    for segment, (first, second) in enumerate(zip(states, states[1:])):
        maximum = max(abs(float(second[name]) - float(first[name]))
                      for name in names)
        subdivisions = max(1, int(math.ceil(maximum / resolution)))
        for step in range(1, subdivisions + 1):
            local = step / subdivisions
            value = {
                name: (1.0 - local) * float(first[name]) +
                local * float(second[name])
                for name in first
            }
            output.append((
                value, (segment + local) / segment_count))
    return output


def interpolate_transform(identity, target, fraction):
    """Interpolate the small planar residual with an exact SE(3) result."""
    del identity
    target = np.asarray(target, dtype=float)
    translation = fraction * target[:3, 3]
    angle = rotation_angle(target)
    if angle <= 1.0e-15:
        rotation = np.eye(3)
    else:
        axis = np.asarray([
            target[2, 1] - target[1, 2],
            target[0, 2] - target[2, 0],
            target[1, 0] - target[0, 1],
        ]) / (2.0 * math.sin(angle))
        x, y, z = axis
        skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        partial = angle * fraction
        rotation = (
            np.eye(3) + math.sin(partial) * skew +
            (1.0 - math.cos(partial)) * (skew @ skew))
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


class RightOpenIK:
    """Track the right-arm branch while retaining the v9 torso schedule."""

    def __init__(self, robot_model):
        from moveit.core.robot_state import RobotState

        self.model = robot_model
        self.reference = RobotState(robot_model)
        self.solver = RobotState(robot_model)
        self.group = 'right_arm'
        self.context_group = 'right_arm_torso'
        self.tip = 'right_dex1_tcp'
        self.group_model = robot_model.get_joint_model_group(self.group)
        self.ik_calls = 0
        self.maximum_translation_error_m = 0.0
        self.maximum_rotation_error_rad = 0.0

    def _set_context(self, state, torso, arm):
        state.set_to_default_values()
        state.set_joint_group_active_positions(
            self.context_group, [float(torso), *map(float, arm)])
        state.update()

    def _local_ik(self, torso, seed, target):
        state = self.solver
        self._set_context(state, torso, seed)
        self.ik_calls += 1
        for _ in range(100):
            actual = state.get_global_link_transform(self.tip)
            translation = target[:3, 3] - actual[:3, 3]
            try:
                rotation = rotation_vector(target[:3, :3] @ actual[:3, :3].T)
            except ValueError:
                return None
            if (np.linalg.norm(translation) <= 1.0e-5 and
                    np.linalg.norm(rotation) <= 1.0e-4):
                solved = np.asarray(
                    state.get_joint_group_positions(self.group), dtype=float)
                return solved if np.isfinite(solved).all() else None
            jacobian = np.asarray(state.get_jacobian(
                self.group, self.tip, np.zeros(3), False), dtype=float)
            if jacobian.shape != (6, len(SOLVER_JOINTS)):
                return None
            error = np.concatenate((translation, rotation))
            damping = 2.0e-3
            try:
                step = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping * damping * np.eye(6),
                    error)
            except np.linalg.LinAlgError:
                return None
            largest = float(np.max(np.abs(step)))
            if largest > 0.035:
                step *= 0.035 / largest
            current = np.asarray(
                state.get_joint_group_positions(self.group), dtype=float)
            candidate = current + step
            try:
                if not self.group_model.satisfies_position_bounds(candidate):
                    return None
            except TypeError:
                pass
            self._set_context(state, torso, candidate)
            state.update()
        return None

    def solve(
            self, reference_torso, solver_torso, source, delta, previous,
            predicted, label):
        source = np.asarray(source, dtype=float)
        self._set_context(self.reference, reference_torso, source)
        baseline = self.reference.get_global_link_transform(self.tip)
        target = delta @ baseline
        nominal_is_continuous = (
            abs(float(reference_torso) - float(solver_torso)) <= 1.0e-12 and
            (previous is None or
             float(np.max(np.abs(source - previous))) <=
             MAXIMUM_ADJACENT_ANGULAR_CHANGE_RAD))
        if (nominal_is_continuous and
                np.linalg.norm(target[:3, 3] - baseline[:3, 3]) <= 1.0e-9 and
                rotation_angle(target @ np.linalg.inv(baseline)) <= 1.0e-9):
            return source.copy()
        seeds = []
        for value in (previous, predicted, source):
            if value is not None and not any(np.allclose(
                    value, existing, atol=1.0e-12, rtol=0.0)
                    for existing in seeds):
                seeds.append(np.asarray(value, dtype=float))
        candidates = []
        for seed in seeds:
            solved = self._local_ik(solver_torso, seed, target)
            if solved is None:
                continue
            self._set_context(self.solver, solver_torso, solved)
            actual = self.solver.get_global_link_transform(self.tip)
            translation_error = float(np.linalg.norm(
                actual[:3, 3] - target[:3, 3]))
            rotation_error = rotation_angle(actual @ np.linalg.inv(target))
            reference = previous if previous is not None else source
            candidates.append((
                float(np.max(np.abs(solved - source))),
                float(np.max(np.abs(solved - reference))),
                translation_error, rotation_error, solved.copy()))
        for seed in seeds:
            self._set_context(self.solver, solver_torso, seed)
            self.ik_calls += 1
            if self.solver.set_from_ik(
                    self.group, matrix_pose(target), self.tip, 0.04):
                self.solver.update()
                solved = np.asarray(
                    self.solver.get_joint_group_positions(self.group),
                    dtype=float)
                actual = self.solver.get_global_link_transform(self.tip)
                translation_error = float(np.linalg.norm(
                    actual[:3, 3] - target[:3, 3]))
                rotation_error = rotation_angle(actual @ np.linalg.inv(target))
                reference = previous if previous is not None else source
                if (np.isfinite(solved).all() and
                        translation_error <= 1.0e-5 and
                        rotation_error <= 1.0e-4):
                    candidates.append((
                        float(np.max(np.abs(solved - source))),
                        float(np.max(np.abs(solved - reference))),
                        translation_error, rotation_error, solved))
        if not candidates:
            raise RuntimeError(f'IK_NO_SOLUTION:{label}')
        candidates = [
            value for value in candidates
            if value[1] <= MAXIMUM_ADJACENT_ANGULAR_CHANGE_RAD]
        if not candidates:
            raise RuntimeError(f'IK_CONTINUITY:{label}:NO_LOCAL_BRANCH')
        candidates.sort(key=lambda value: (value[0], value[1]))
        selected = candidates[0]
        self.maximum_translation_error_m = max(
            self.maximum_translation_error_m, selected[2])
        self.maximum_rotation_error_rad = max(
            self.maximum_rotation_error_rad, selected[3])
        return selected[4]

    def solve_path(
            self, sources, deltas, previous, label, torso_offsets=None):
        output = []
        previous_source = None
        if torso_offsets is None:
            torso_offsets = [0.0] * len(sources)
        if len(torso_offsets) != len(sources):
            raise ValueError('TORSO_OFFSET_COUNT')
        for index, (source_state, delta, torso_offset) in enumerate(zip(
                sources, deltas, torso_offsets)):
            reference_torso = float(source_state['torso_joint'])
            solver_torso = reference_torso + float(torso_offset)
            source = np.asarray([source_state[name] for name in SOLVER_JOINTS])
            predicted = (
                previous + source - previous_source
                if previous is not None and previous_source is not None
                else None)
            solved = self.solve(
                reference_torso, solver_torso, source, delta, previous,
                predicted,
                f'{label}:{index}/{len(sources)}')
            state = dict(source_state)
            state['torso_joint'] = solver_torso
            state.update(zip(SOLVER_JOINTS, solved))
            output.append(state)
            previous = solved
            previous_source = source
        return output, previous


def _current_state(epoch):
    if len(epoch.robot_state.name) != len(epoch.robot_state.position):
        raise ValueError('ROBOT_STATE_WIDTH')
    values = {}
    for name, position in zip(
            epoch.robot_state.name, epoch.robot_state.position):
        if name in values or not math.isfinite(float(position)):
            raise ValueError('ROBOT_STATE_VALUE')
        values[name] = float(position)
    missing = sorted(set(AUDIT_JOINTS) - set(values))
    if missing:
        raise ValueError(f'ROBOT_STATE_MISSING:{missing}')
    return {name: values[name] for name in AUDIT_JOINTS}


def _door_source_states(contact_last, phase):
    torso, arm = phase['commands']
    output = []
    for torso_point, arm_point in zip(torso['points'], arm['points']):
        state = dict(contact_last)
        state['torso_joint'] = float(torso_point['positions'][0])
        state.update(zip(RIGHT_ARM, map(float, arm_point['positions'])))
        output.append(state)
    return output


def _state_matrix(states):
    result = np.asarray([
        [float(state[name]) for name in AUDIT_JOINTS]
        for state in states], dtype=float)
    if result.ndim != 2 or result.shape[1] != len(AUDIT_JOINTS) or not (
            np.isfinite(result).all()):
        raise ValueError('TRAJECTORY_MATRIX')
    return result


def retime_states(states, minimum_dt_s):
    """Conservatively time a complete synchronized multi-controller path."""
    positions = _state_matrix(states)
    velocity_limits = np.asarray([VELOCITY_LIMITS[name]
                                  for name in AUDIT_JOINTS])
    acceleration_limits = np.asarray([ACCELERATION_LIMITS[name]
                                      for name in AUDIT_JOINTS])
    dt = float(minimum_dt_s)
    if len(positions) > 1:
        dt = max(dt, float(np.max(
            np.abs(np.diff(positions, axis=0)) / velocity_limits)))
    dt *= 1.001
    for _ in range(4):
        velocities = np.zeros_like(positions)
        if len(positions) > 1:
            velocities[1:-1] = (
                positions[2:] - positions[:-2]) / (2.0 * dt)
        accelerations = np.zeros_like(positions)
        if len(positions) > 1:
            accelerations[1:] = (
                velocities[1:] - velocities[:-1]) / dt
        velocity_ratio = float(np.max(
            np.abs(velocities) / velocity_limits))
        acceleration_ratio = float(np.max(
            np.abs(accelerations) / acceleration_limits))
        scale = max(1.0, velocity_ratio, math.sqrt(acceleration_ratio))
        if scale <= 1.0 + 1.0e-12:
            break
        dt *= scale * 1.001
    points = []
    for index, (position, velocity, acceleration) in enumerate(zip(
            positions, velocities, accelerations)):
        seconds = (index + 1) * dt
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1.0e9))
        if nanosec == 1_000_000_000:
            sec += 1
            nanosec = 0
        points.append({
            'positions': position.tolist(),
            'velocities': velocity.tolist(),
            'accelerations': acceleration.tolist(),
            'time_from_start': {'sec': sec, 'nanosec': nanosec},
        })
    return points, {
        'uniform_dt_s': dt,
        'duration_s': len(points) * dt,
        'maximum_velocity_ratio': velocity_ratio,
        'maximum_acceleration_ratio': acceleration_ratio,
    }


def build_stage(name, states, minimum_dt_s, audit_only=False):
    points, timing = retime_states(states, minimum_dt_s)
    audit_message = {
        'joint_names': list(AUDIT_JOINTS),
        'points': points,
    }
    controller_projection = {}
    controller_groups = {
        '/torso_controller/joint_trajectory': ('torso_joint',),
        '/left_arm_controller/joint_trajectory': LEFT_ARM,
        '/right_arm_controller/joint_trajectory': RIGHT_ARM,
        '/left_gripper_controller/joint_trajectory': LEFT_GRIPPER,
        '/right_gripper_controller/joint_trajectory': RIGHT_GRIPPER,
    }
    for topic, names in controller_groups.items():
        indices = [AUDIT_JOINTS.index(value) for value in names]
        projected = []
        for point in points:
            projected.append({
                'positions': [point['positions'][index] for index in indices],
                'velocities': [point['velocities'][index] for index in indices],
                'accelerations': [
                    point['accelerations'][index] for index in indices],
                'time_from_start': dict(point['time_from_start']),
            })
        controller_projection[topic] = {
            'joint_names': list(names), 'points': projected}
    return {
        'name': name,
        'audit_only': bool(audit_only),
        'messages': [audit_message],
        'controller_projection': controller_projection,
        'timing': timing,
    }


def continuity_metrics(stages):
    maximum_angular = 0.0
    maximum_gripper = 0.0
    previous = None
    for stage in stages:
        positions = np.asarray([
            value['positions'] for value in stage['messages'][0]['points']])
        if previous is not None:
            positions = np.vstack((previous, positions))
        if len(positions) > 1:
            difference = np.abs(np.diff(positions, axis=0))
            angular_indices = [index for index, name in enumerate(AUDIT_JOINTS)
                               if 'finger' not in name]
            finger_indices = [index for index, name in enumerate(AUDIT_JOINTS)
                              if 'finger' in name]
            maximum_angular = max(
                maximum_angular,
                float(np.max(difference[:, angular_indices])))
            maximum_gripper = max(
                maximum_gripper,
                float(np.max(difference[:, finger_indices])))
        previous = positions[-1]
    return maximum_angular, maximum_gripper


def reverse_path_audit(stages):
    """Require every audit-only return stage to mirror its forward peer."""
    if len(stages) != 6:
        return False
    for forward_index, reverse_index in ((0, 5), (1, 4), (2, 3)):
        forward = stages[forward_index]
        reverse = stages[reverse_index]
        if forward.get('audit_only') or not reverse.get('audit_only'):
            return False
        forward_points = forward['messages'][0]['points']
        reverse_points = reverse['messages'][0]['points']
        if len(forward_points) != len(reverse_points):
            return False
        expected = np.asarray([
            value['positions'] for value in reversed(forward_points)])
        actual = np.asarray([
            value['positions'] for value in reverse_points])
        if not np.allclose(expected, actual, atol=1.0e-12, rtol=0.0):
            return False
    return True


def cpp_audit(args, config, epoch, stages):
    run_root = Path(args.output).parent
    run_root.mkdir(parents=True, exist_ok=True)
    audit_plan = {
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'candidate3_only': True,
        'contains_partial': False,
        'door_arc_target_count': 601,
        'audit_topology': AUDIT_TOPOLOGY,
        'stages': [
            {'name': value['name'], 'messages': value['messages']}
            for value in stages],
    }
    audit_hash = canonical_hash(audit_plan)
    input_path = run_root / 'post_nav_open_audit_input.json'
    urdf_path = run_root / 'post_nav_open_robot.urdf'
    srdf_path = run_root / 'post_nav_open_robot.srdf'
    result_path = run_root / 'post_nav_open_audit_result.json'
    input_path.write_text(json.dumps(
        {'plan': audit_plan, 'plan_hash': audit_hash},
        sort_keys=True, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    urdf_path.write_text(config['robot_description'], encoding='utf-8')
    srdf_path.write_text(
        config['robot_description_semantic'], encoding='utf-8')
    completed = subprocess.run([
        args.clearance_auditor,
        '--scene', args.scene,
        '--plan', str(input_path),
        '--urdf', str(urdf_path),
        '--srdf', str(srdf_path),
        '--output', str(result_path),
    ], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
       stderr=subprocess.PIPE, text=True, timeout=args.clearance_timeout_s,
       check=False)
    if completed.returncode not in (0, 2) or not result_path.is_file():
        raise RuntimeError(
            f'CPP_OPEN_AUDITOR:exit={completed.returncode}:'
            f'{completed.stderr[-400:]}')
    result = json.loads(result_path.read_text(encoding='utf-8'))
    if (result.get('schema') != 'h1_candidate3_cpp_clearance_audit/v1' or
            result.get('plan_hash') != audit_hash):
        raise RuntimeError('CPP_OPEN_AUDITOR_CONTRACT')
    return result, audit_hash


def build_result(args):
    global _MOVEIT_KEEPALIVE
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene
    from moveit.planning import MoveItPy
    from rclpy.serialization import deserialize_message

    epoch = deserialize_message(Path(args.epoch).read_bytes(), SceneEpoch)
    scene = deserialize_message(Path(args.scene).read_bytes(), PlanningScene)
    if not epoch.valid or not epoch.stable:
        raise ValueError(f'EPOCH_INVALID:{epoch.status}')
    if (scene.is_diff or scene.name !=
            f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'):
        raise ValueError('SCENE_EPOCH_BINDING')
    _module, bundle, prefix, door_phase, derived_ingress = load_v9_sources(
        args.workspace, args.authority_root)
    alignment = recover_scene_alignment(
        epoch, scene, base_envelope_validator=validate_open_base_envelope)
    current = _current_state(epoch)
    source_launch = prefix[0].states[0]
    start_error = max(abs(current[name] - float(source_launch[name]))
                      for name in AUDIT_JOINTS)
    if start_error > 0.003:
        raise ValueError(f'CURRENT_STATE_SEAM:{start_error:.9g}')

    config = moveit_configuration()
    moveit = MoveItPy(
        node_name=f'candidate3_post_nav_open_{os.getpid()}',
        config_dict=config)
    _MOVEIT_KEEPALIVE = moveit
    solver = RightOpenIK(moveit.get_robot_model())
    started = time.monotonic()

    ingress_sources = densify_states(derived_ingress, IK_JOINTS)
    ingress_values = [value for value, _ in ingress_sources]
    ingress_deltas = []
    ingress_torso_offsets = []
    ingress_retreat = np.asarray([
        args.ingress_clearance_x_m,
        args.ingress_clearance_y_m,
        args.ingress_clearance_z_m,
    ], dtype=float)
    if (not np.isfinite(ingress_retreat).all() or
            float(np.linalg.norm(ingress_retreat)) > 0.005):
        raise ValueError('INGRESS_CLEARANCE_RETREAT_ENVELOPE')
    if (not math.isfinite(args.ingress_torso_bump_deg) or
            abs(args.ingress_torso_bump_deg) > 2.0):
        raise ValueError('INGRESS_TORSO_BUMP_ENVELOPE')
    if (not math.isfinite(args.planning_torso_bias_deg) or
            abs(args.planning_torso_bias_deg) > 2.0):
        raise ValueError('PLANNING_TORSO_BIAS_ENVELOPE')
    torso_bias = math.radians(args.planning_torso_bias_deg)
    for _, fraction in ingress_sources:
        retreat = np.eye(4)
        retreat[:3, 3] = (
            ingress_retreat * math.sin(math.pi * fraction))
        ingress_deltas.append(
            retreat @ interpolate_transform(
                np.eye(4), alignment['cabinet_delta'], fraction))
        ingress_torso_offsets.append(
            torso_bias * fraction +
            math.radians(args.ingress_torso_bump_deg) *
            math.sin(math.pi * fraction))
    previous = np.asarray([
        prefix[0].states[-1][name] for name in SOLVER_JOINTS])
    solved_ingress, previous = solver.solve_path(
        ingress_values, ingress_deltas, previous, 'right_ingress',
        torso_offsets=ingress_torso_offsets)
    stage0_states = [current, dict(prefix[0].states[-1])]
    stage0_states.extend(solved_ingress)

    contact_sources = [
        value for value, _ in densify_states(
            prefix[2].states, RIGHT_GRIPPER,
            MAXIMUM_ADJACENT_GRIPPER_CHANGE_M)]
    contact_sources.extend(dict(value) for value in prefix[3].states[1:])
    contact_deltas = [alignment['cabinet_delta']] * len(contact_sources)
    solved_contact, previous = solver.solve_path(
        contact_sources, contact_deltas, previous, 'right_handle_unlock',
        torso_offsets=[torso_bias] * len(contact_sources))

    door_sources = _door_source_states(solved_contact[-1], door_phase)
    door_deltas = [alignment['cabinet_delta']] * len(door_sources)
    solved_door, previous = solver.solve_path(
        door_sources, door_deltas, previous, 'right_door_open_601',
        torso_offsets=[torso_bias] * len(door_sources))
    del previous

    forward_stages = [
        build_stage(STAGE_NAMES[0], stage0_states, 0.010),
        build_stage(STAGE_NAMES[1], solved_contact, 0.004),
        build_stage(STAGE_NAMES[2], solved_door, 0.100),
    ]
    reverse_stages = [
        build_stage(
            STAGE_NAMES[3], list(reversed(solved_door)), 0.100,
            audit_only=True),
        build_stage(
            STAGE_NAMES[4], list(reversed(solved_contact)), 0.004,
            audit_only=True),
        build_stage(
            STAGE_NAMES[5], list(reversed(stage0_states)), 0.010,
            audit_only=True),
    ]
    stages = forward_stages + reverse_stages
    maximum_angular, maximum_gripper = continuity_metrics(stages)
    torso_index = AUDIT_JOINTS.index('torso_joint')
    maximum_absolute_torso = max(
        abs(float(point['positions'][torso_index]))
        for stage in stages
        for point in stage['messages'][0]['points'])
    continuity_passed = (
        maximum_angular <= MAXIMUM_ADJACENT_ANGULAR_CHANGE_RAD and
        maximum_gripper <= MAXIMUM_ADJACENT_GRIPPER_CHANGE_M)
    timing_passed = all(
        value['timing']['maximum_velocity_ratio'] <= 1.0 + 1.0e-9 and
        value['timing']['maximum_acceleration_ratio'] <= 1.0 + 1.0e-9
        for value in stages)
    reverse_passed = reverse_path_audit(stages)
    torso_passed = maximum_absolute_torso < MAXIMUM_ABSOLUTE_TORSO_RAD
    cpp_result, audit_hash = cpp_audit(args, config, epoch, stages)
    collision_passed = bool(
        cpp_result.get('collision_free') is True and
        cpp_result.get('clearance_passed') is True and
        float(cpp_result.get('minimum_environment_clearance_m', -1.0)) >=
        0.010)
    planning_seconds = time.monotonic() - started
    alignment_hash = (
        epoch.status.split(':', 1)[1]
        if epoch.status.startswith('FROZEN_POST_NAV_ALIGNMENT:') else '')
    source_hashes = {
        'right_handle_executor': EXECUTOR_SHA256,
        'right_handle_manifest': MANIFEST_SHA256,
        'right_handle_contact_path': bundle.path.sha256,
        'right_handle_ingress_path': bundle.ingress.sha256,
        'derived_clearance10_ingress': DERIVED_INGRESS_SHA256,
        'right_handle_exact_audit': bundle.audit.sha256,
        'middle_plan': MIDDLE_PLAN_SHA256,
    }
    plan = {
        'schema': SCHEMA,
        'planner_version': PLANNER_VERSION,
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'source_epoch_status': epoch.status,
        'post_nav_alignment_hash': alignment_hash,
        'action_semantics': (
            'right_hand_only_handle_unlock_and_door_open;'
            'left_hand_never_contacts_handle'),
        'candidate3_only': True,
        'contains_partial': False,
        'audit_topology': AUDIT_TOPOLOGY,
        'door_arc_target_count': 601,
        'execution_stage_count': EXECUTION_STAGE_COUNT,
        'reverse_audit_stage_count': len(reverse_stages),
        'source_hashes': source_hashes,
        'ingress_clearance_retreat_xyz_m': ingress_retreat.tolist(),
        'ingress_torso_bump_deg': float(args.ingress_torso_bump_deg),
        'planning_torso_bias_deg': float(args.planning_torso_bias_deg),
        'base_pose_residual': alignment['base_residual'].tolist(),
        'base_residual_metrics': alignment['base_metrics'],
        'planning_handle_pose': alignment['pelvis_handle'].tolist(),
        'door_tag_translation_error_m':
            alignment['door_tag_translation_error_m'],
        'door_tag_rotation_error_rad':
            alignment['door_tag_rotation_error_rad'],
        'stage_names': list(STAGE_NAMES),
        'stage_waypoint_counts': [
            len(value['messages'][0]['points']) for value in stages],
        'planned_action_duration_s': sum(
            value['timing']['duration_s'] for value in forward_stages),
        'reverse_audit_duration_s': sum(
            value['timing']['duration_s'] for value in reverse_stages),
        'clearance_audited_trajectory_hash': audit_hash,
        'planning_wall_duration_s': planning_seconds,
        'ik_calls': solver.ik_calls,
        'maximum_ik_translation_error_m':
            solver.maximum_translation_error_m,
        'maximum_ik_rotation_error_rad': solver.maximum_rotation_error_rad,
        'maximum_absolute_torso_rad': maximum_absolute_torso,
        'stages': stages,
    }
    plan_hash = canonical_hash(plan)
    passed = (
        collision_passed and continuity_passed and timing_passed and
        reverse_passed and torso_passed)
    if not continuity_passed:
        status = 'CONTINUITY'
    elif not timing_passed:
        status = 'TIMING'
    elif not reverse_passed:
        status = 'REVERSE_PATH'
    elif not torso_passed:
        status = 'TORSO_45_DEGREE_LIMIT'
    elif not collision_passed:
        status = str(cpp_result.get('status', 'COLLISION_CLEARANCE'))
    else:
        status = 'PASS'
    certificate = {
        'schema': CERTIFICATE_SCHEMA,
        'planner_version': PLANNER_VERSION,
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'source_epoch_status': epoch.status,
        'post_nav_alignment_hash': alignment_hash,
        'plan_hash': plan_hash,
        'source_hashes': source_hashes,
        'right_hand_only_handle': True,
        'left_handle_command_count': 0,
        'door_arc_target_count': 601,
        'execution_stage_count': EXECUTION_STAGE_COUNT,
        'reverse_audit_stage_count': len(reverse_stages),
        'ik_complete': True,
        'collision_free': cpp_result.get('collision_free') is True,
        'clearance_passed': cpp_result.get('clearance_passed') is True,
        'minimum_environment_clearance_m':
            cpp_result.get('minimum_environment_clearance_m'),
        'maximum_adjacent_angular_change_rad': maximum_angular,
        'maximum_adjacent_gripper_change_m': maximum_gripper,
        'maximum_absolute_torso_rad': maximum_absolute_torso,
        'torso_below_45_degrees': torso_passed,
        'timing_passed': timing_passed,
        'forward_passed': (
            collision_passed and continuity_passed and timing_passed and
            torso_passed),
        'reverse_passed': reverse_passed,
        'no_partial': True,
        'valid': passed,
        'status': status,
        'audit_details': cpp_result,
    }
    return {
        'plan': plan,
        'plan_hash': plan_hash,
        'certificate': certificate,
        'certificate_hash': canonical_hash(certificate),
    }


def failure_result(error):
    return {
        'plan': None,
        'plan_hash': '',
        'certificate': {
            'schema': CERTIFICATE_SCHEMA,
            'planner_version': PLANNER_VERSION,
            'ik_complete': False,
            'no_partial': True,
            'valid': False,
            'status': f'{type(error).__name__}:{error}',
        },
        'certificate_hash': '',
    }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--epoch', required=True)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--authority-root', required=True)
    parser.add_argument('--clearance-auditor', required=True)
    parser.add_argument('--clearance-timeout-s', type=float, default=300.0)
    parser.add_argument(
        '--ingress-clearance-x-m', type=float,
        default=INGRESS_CLEARANCE_RETREAT_X_M)
    parser.add_argument(
        '--ingress-clearance-y-m', type=float,
        default=INGRESS_CLEARANCE_RETREAT_Y_M)
    parser.add_argument(
        '--ingress-clearance-z-m', type=float,
        default=INGRESS_CLEARANCE_RETREAT_Z_M)
    parser.add_argument('--ingress-torso-bump-deg', type=float, default=0.0)
    parser.add_argument('--planning-torso-bias-deg', type=float, default=0.5)
    parser.add_argument('--output', required=True)
    return parser.parse_args(argv)


def main(argv=None):
    faulthandler.enable()
    args = parse_arguments(argv)
    try:
        result = build_result(args)
        exit_code = 0 if result['certificate']['valid'] else 2
    except Exception as error:
        result = failure_result(error)
        exit_code = 2
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + '.tmp')
    temporary.write_text(json.dumps(
        result, sort_keys=True, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')
    os.replace(temporary, output)
    print(json.dumps({
        'status': result['certificate']['status'],
        'valid': result['certificate']['valid'],
        'output': str(output),
    }, sort_keys=True), flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


if __name__ == '__main__':
    main()
