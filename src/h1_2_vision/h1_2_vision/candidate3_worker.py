"""
Isolated MoveIt worker for candidate-3 planning and plan-only auditing.

This process never creates a trajectory publisher, action client, or controller
client.  It consumes an immutable SceneEpoch/PlanningScene pair and writes one
content-addressed plan-and-certificate document before exiting.
"""

from __future__ import annotations

import argparse
import copy
import faulthandler
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .candidate3_alignment import (
    recover_scene_alignment,
    REFERENCE_CABINET,
    REFERENCE_CUBE,
)
from .candidate3_contract import (
    AUDITOR_VERSION,
    canonical_hash,
    dense_interpolate,
    DENSE_RESOLUTION_RAD,
    DOOR_ARC_TARGET_COUNT,
    interpolate_transform,
    load_sealed_assets,
    MAXIMUM_ADJACENT_CHANGE_RAD,
    PLANNER_VERSION,
    reverse_stage,
    SCHEMA,
    select_knots,
    stage_hash,
    STAGE_NAMES,
    TOPOLOGY,
    yaw_transform,
)


__all__ = ('REFERENCE_CABINET', 'REFERENCE_CUBE')


ARM_VELOCITY = np.asarray([1.0, 1.0, 1.2, 1.2, 1.5, 1.5, 1.5])
ARM_ACCELERATION = np.asarray([2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0])
GRIPPER_VELOCITY = np.asarray([0.20, 0.20])
GRIPPER_ACCELERATION = np.asarray([0.40, 0.40])
MESSAGE_GAP_S = 0.0
_MOVEIT_KEEPALIVE = None


def matrix_pose(value):
    from geometry_msgs.msg import Pose
    from .candidate3_contract import quaternion_from_matrix

    result = Pose()
    result.position.x, result.position.y, result.position.z = value[:3, 3]
    quaternion = quaternion_from_matrix(value[:3, :3])
    result.orientation.x, result.orientation.y = quaternion[:2]
    result.orientation.z, result.orientation.w = quaternion[2:]
    return result


def rotation_angle(value):
    cosine = (float(np.trace(value[:3, :3])) - 1.0) * 0.5
    return math.acos(min(1.0, max(-1.0, cosine)))


def rotation_vector(value):
    """Return the deterministic axis-angle vector of a rotation matrix."""
    rotation = np.asarray(value, dtype=float)[:3, :3]
    angle = math.acos(min(
        1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) * 0.5)))
    skew = np.asarray([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    if angle <= 1e-8:
        return 0.5 * skew
    sine = math.sin(angle)
    if abs(sine) <= 1e-8:
        raise ValueError('rotation vector singularity')
    return angle * skew / (2.0 * sine)


def moveit_configuration():
    from moveit_configs_utils import MoveItConfigsBuilder

    value = (
        MoveItConfigsBuilder('h1_2', package_name='h1_2_moveit_config')
        .robot_description(file_path='config/h1_2.urdf.xacro')
        .robot_description_semantic(file_path='config/h1_2.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True)
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs().to_dict())
    # Jazzy MoveItPy expects this nested form even though the builder emits a
    # flat pipeline-name list.  No plan/execution method is called here.
    value['planning_pipelines'] = {'pipeline_names': ['ompl']}
    value.setdefault('planning_scene_monitor_options', {})[
        'wait_for_initial_state_timeout'] = 0.0
    return value


def target_delta(stage_index, message_index, fraction,
                 cabinet_delta, cube_delta):
    identity = np.eye(4)
    if stage_index == 0:
        if message_index == 0:
            return interpolate_transform(identity, cabinet_delta, fraction)
        return cabinet_delta
    if stage_index == 1:
        return interpolate_transform(cabinet_delta, identity, fraction)
    if stage_index == 2:
        if message_index == 1:
            return interpolate_transform(identity, cabinet_delta, fraction)
        return cabinet_delta
    if stage_index == 3:
        if message_index == 0:
            return interpolate_transform(identity, cube_delta, fraction)
        if message_index == 2:
            return interpolate_transform(cube_delta, cabinet_delta, fraction)
        if message_index == 5:
            return interpolate_transform(cabinet_delta, identity, fraction)
        return cabinet_delta
    raise ValueError(f'unknown forward stage index: {stage_index}')


class Replanner:
    def __init__(self, robot_model, cabinet_delta, cube_delta):
        """Initialize deterministic FK/IK state for the frozen epoch."""
        from moveit.core.robot_state import RobotState

        self.model = robot_model
        self.cabinet_delta = cabinet_delta
        self.cube_delta = cube_delta
        self.reference = RobotState(robot_model)
        self.solver = RobotState(robot_model)
        self.reference.set_to_default_values()
        self.solver.set_to_default_values()
        self.ik_calls = 0
        self.ik_failures = []
        self.maximum_ik_translation_error_m = 0.0
        self.maximum_ik_rotation_error_rad = 0.0
        self.previous_by_group = {}
        self.previous_source_by_group = {}

    @staticmethod
    def group_and_tip(message):
        first = message['joint_names'][0]
        if first.startswith('left_'):
            return 'left_arm', 'left_dex1_tcp'
        if first.startswith('right_'):
            return 'right_arm', 'right_dex1_tcp'
        raise ValueError('unknown arm group')

    def local_ik(self, group, tip, seed, target_tcp):
        """Track the nearby IK branch with deterministic damped least squares."""
        seed = np.asarray(seed, dtype=float)
        state = self.solver
        state.set_to_default_values()
        state.set_joint_group_active_positions(group, seed)
        state.update()
        group_model = self.model.get_joint_model_group(group)
        self.ik_calls += 1
        for _ in range(80):
            actual_tcp = state.get_global_link_transform(tip)
            translation = target_tcp[:3, 3] - actual_tcp[:3, 3]
            try:
                rotation = rotation_vector(
                    target_tcp[:3, :3] @ actual_tcp[:3, :3].T)
            except ValueError:
                return None
            if (float(np.linalg.norm(translation)) <= 1.0e-5 and
                    float(np.linalg.norm(rotation)) <= 1.0e-4):
                solved = np.asarray(
                    state.get_joint_group_positions(group), dtype=float)
                try:
                    bounded = bool(
                        group_model.satisfies_position_bounds(solved))
                except TypeError:
                    bounded = True
                return solved if bounded and np.isfinite(solved).all() else None
            jacobian = np.asarray(state.get_jacobian(
                group, tip, np.zeros(3), False), dtype=float)
            if jacobian.shape != (6, len(seed)) or not np.isfinite(
                    jacobian).all():
                return None
            error = np.concatenate((translation, rotation))
            damping = 2.0e-3
            try:
                step = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T +
                    (damping * damping) * np.eye(6), error)
            except np.linalg.LinAlgError:
                return None
            largest = float(np.max(np.abs(step)))
            if largest > 0.035:
                step *= 0.035 / largest
            current = np.asarray(
                state.get_joint_group_positions(group), dtype=float)
            candidate = current + step
            try:
                if not group_model.satisfies_position_bounds(candidate):
                    return None
            except TypeError:
                pass
            state.set_joint_group_active_positions(group, candidate)
            state.update()
        return None

    def solve(
            self, group, tip, source, delta, previous, label,
            preserve_nominal=True, predicted=None):
        source = np.asarray(source, dtype=float)
        self.reference.set_joint_group_active_positions(group, source)
        self.reference.update()
        baseline_tcp = self.reference.get_global_link_transform(tip)
        target_tcp = delta @ baseline_tcp
        translation_residual = float(np.linalg.norm(
            target_tcp[:3, 3] - baseline_tcp[:3, 3]))
        rotation_residual = rotation_angle(
            target_tcp @ np.linalg.inv(baseline_tcp))
        nominal_branch_is_continuous = (
            previous is None or
            float(np.max(np.abs(source - previous))) <=
            MAXIMUM_ADJACENT_CHANGE_RAD)
        if (preserve_nominal and nominal_branch_is_continuous and
                translation_residual <= 1e-6 and
                rotation_residual <= 1e-5):
            # The relative planner's exact nominal point is already a valid IK
            # solution on the current continuous branch. Preserve it
            # byte-for-byte instead of inviting a KDL null-space branch
            # change at a redundant wrist posture.
            return source.copy()
        candidates = []
        seeds = [previous, predicted, source]
        seeds = [np.asarray(value, dtype=float) for value in seeds
                 if value is not None]
        unique_seeds = []
        for seed in seeds:
            if not any(np.allclose(seed, value, atol=1e-12, rtol=0)
                       for value in unique_seeds):
                unique_seeds.append(seed)
        for seed in unique_seeds:
            self.solver.set_to_default_values()
            self.solver.set_joint_group_active_positions(
                group, np.asarray(seed, dtype=float))
            self.solver.update()
            self.ik_calls += 1
            if self.solver.set_from_ik(
                    group, matrix_pose(target_tcp), tip, 0.035):
                self.solver.update()
                solved = np.asarray(
                    self.solver.get_joint_group_positions(group), dtype=float)
                if np.isfinite(solved).all():
                    actual_tcp = self.solver.get_global_link_transform(tip)
                    translation_error = float(np.linalg.norm(
                        actual_tcp[:3, 3] - target_tcp[:3, 3]))
                    rotation_error = rotation_angle(
                        actual_tcp @ np.linalg.inv(target_tcp))
                    if (translation_error <= 1.0e-5 and
                            rotation_error <= 1.0e-4):
                        reference = previous if previous is not None else source
                        candidates.append((
                            float(np.max(np.abs(solved - reference))),
                            translation_error, rotation_error, solved))
        candidates.sort(key=lambda value: value[0])
        # Prefer MoveIt's bounded KDL solution whenever it already remains on
        # the continuous branch.  The local solver is a deterministic fallback
        # for a failed or branch-jumping KDL result, not a nominal-path
        # replacement; this avoids injecting redundant-joint micro-curvature.
        if not candidates or candidates[0][0] > MAXIMUM_ADJACENT_CHANGE_RAD:
            for seed in unique_seeds:
                local = self.local_ik(group, tip, seed, target_tcp)
                if local is None:
                    continue
                self.solver.set_joint_group_active_positions(group, local)
                self.solver.update()
                actual_tcp = self.solver.get_global_link_transform(tip)
                translation_error = float(np.linalg.norm(
                    actual_tcp[:3, 3] - target_tcp[:3, 3]))
                rotation_error = rotation_angle(
                    actual_tcp @ np.linalg.inv(target_tcp))
                reference = previous if previous is not None else source
                candidates.append((
                    float(np.max(np.abs(local - reference))),
                    translation_error, rotation_error, local.copy()))
        if not candidates:
            self.ik_failures.append(label)
            raise RuntimeError(f'IK_NO_SOLUTION:{label}')
        candidates.sort(key=lambda value: value[0])
        selected = candidates[0]
        self.maximum_ik_translation_error_m = max(
            self.maximum_ik_translation_error_m, selected[1])
        self.maximum_ik_rotation_error_rad = max(
            self.maximum_ik_rotation_error_rad, selected[2])
        return selected[3]

    def solve_source_path(self, message, source_positions, deltas, label,
                          knot_indices=None, preserve_nominal=True):
        group, tip = self.group_and_tip(message)
        source_positions = np.asarray(source_positions, dtype=float)
        if knot_indices is None:
            # The 2-degree asset simplification is sufficient for nominal
            # replay but can cross a narrow IK basin after a 30 mm/5 degree
            # object transform.  Keep IK seeds no farther apart than the same
            # 0.49-degree resolution used by the final dense path.
            knot_indices = select_knots(
                source_positions, DENSE_RESOLUTION_RAD)
        solved = []
        # Preserve one continuous branch across adjacent trajectory-message
        # envelopes and across stages for each arm.  Finger messages do not
        # disturb the corresponding arm seed.
        previous = self.previous_by_group.get(group)
        previous_source = self.previous_source_by_group.get(group)
        denominator = max(1, len(source_positions) - 1)
        for sequence, index in enumerate(knot_indices):
            fraction = index / denominator
            delta = deltas(fraction)
            source = source_positions[index]
            predicted = None
            if previous is not None and previous_source is not None:
                # Transport the nominal joint increment onto the current IK
                # branch.  This is a much better local seed near redundant or
                # singular wrist postures than reusing the last state alone.
                predicted = previous + (source - previous_source)
            value = self.solve(
                group, tip, source, delta, previous,
                f'{label}:{sequence}/{len(knot_indices)}', preserve_nominal,
                predicted)
            if previous is not None:
                jump = float(np.max(np.abs(value - previous)))
                if jump > MAXIMUM_ADJACENT_CHANGE_RAD:
                    raise RuntimeError(
                        f'IK_CONTINUITY:{label}:{sequence}/'
                        f'{len(knot_indices)}:{math.degrees(jump):.6f}')
            solved.append(value)
            previous = value
            previous_source = source
        self.previous_by_group[group] = solved[-1].copy()
        self.previous_source_by_group[group] = previous_source.copy()
        return dense_interpolate(solved, DENSE_RESOLUTION_RAD)

    def normal_message(self, stage_index, message_index, message):
        positions = [point['positions'] for point in message['points']]

        def deltas(fraction):
            return target_delta(
                stage_index, message_index, fraction,
                self.cabinet_delta, self.cube_delta)
        solved = self.solve_source_path(
            message, positions, deltas,
            f'{STAGE_NAMES[stage_index]}/message_{message_index}')
        output = copy.deepcopy(message)
        output['points'] = [point_record(value) for value in solved]
        return output

    def door_arc(self, stage_index, messages):
        source = []
        for message_index, message in enumerate(messages):
            positions = [point['positions'] for point in message['points']]
            if message_index:
                positions = positions[1:]
            source.extend(positions)
        source = np.asarray(source, dtype=float)
        sample_axis = np.linspace(0, len(source) - 1, DOOR_ARC_TARGET_COUNT)
        sampled = []
        for position in sample_axis:
            lower = int(math.floor(position))
            upper = min(len(source) - 1, lower + 1)
            fraction = position - lower
            sampled.append(
                (1 - fraction) * source[lower] + fraction * source[upper])
        template = messages[0]
        solved = self.solve_source_path(
            template, sampled, lambda _: self.cabinet_delta,
            f'{STAGE_NAMES[stage_index]}/door_arc_601',
            knot_indices=list(range(DOOR_ARC_TARGET_COUNT)),
            preserve_nominal=False)
        # Each 10-degree segment contains 101 targets; adjacent segments share
        # their boundary, yielding exactly 601 unique hinge targets.
        output = []
        for segment, envelope in enumerate(messages):
            message = copy.deepcopy(envelope)
            subset = solved[segment * 100:(segment + 1) * 100 + 1]
            message['points'] = [point_record(value) for value in subset]
            output.append(message)
        return output

    def stage(self, stage_index, payload):
        print(json.dumps({
            'event': 'PLAN_STAGE_START',
            'stage': STAGE_NAMES[stage_index],
            'ik_calls': self.ik_calls,
        }, sort_keys=True), flush=True)
        result = {'name': STAGE_NAMES[stage_index], 'messages': []}
        if stage_index == 2:
            for index, message in enumerate(payload['messages'][:7]):
                if len(message['joint_names']) == 7:
                    result['messages'].append(
                        self.normal_message(stage_index, index, message))
                else:
                    copied = copy.deepcopy(message)
                    copied['points'] = [
                        point_record(point['positions'])
                        for point in message['points']]
                    result['messages'].append(copied)
            result['messages'].extend(
                self.door_arc(stage_index, payload['messages'][7:]))
            return result
        for index, message in enumerate(payload['messages']):
            copied = copy.deepcopy(message)
            if len(message['joint_names']) == 7:
                copied = self.normal_message(stage_index, index, message)
            else:
                copied['points'] = [
                    point_record(point['positions'])
                    for point in message['points']]
            result['messages'].append(copied)
        return result


def point_record(positions):
    return {
        'positions': [float(value) for value in positions],
        'velocities': [],
        'accelerations': [],
        'effort': [],
        'time_from_start': {'sec': 0, 'nanosec': 0},
    }


def seconds_duration(seconds):
    whole = int(math.floor(seconds))
    nanoseconds = int(round((seconds - whole) * 1e9))
    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000
    return {'sec': whole, 'nanosec': nanoseconds}


def simplify_collinear_positions(positions, tolerance_rad=1.0e-4):
    """Remove numerically collinear audit samples without changing the path."""
    values = np.asarray(positions, dtype=float)
    if values.ndim != 2 or len(values) == 0:
        raise ValueError('EMPTY_TIMING_PATH')
    if len(values) <= 2:
        return values.copy()
    retained = {0, len(values) - 1}
    pending = [(0, len(values) - 1)]
    while pending:
        begin, end = pending.pop()
        if end <= begin + 1:
            continue
        fractions = (
            np.arange(1, end - begin, dtype=float) / (end - begin))
        interpolated = (
            values[begin][None, :] + fractions[:, None] *
            (values[end] - values[begin])[None, :])
        errors = np.max(
            np.abs(values[begin + 1:end] - interpolated), axis=1)
        local_index = int(np.argmax(errors))
        if float(errors[local_index]) > tolerance_rad:
            index = begin + 1 + local_index
            retained.add(index)
            pending.extend(((begin, index), (index, end)))
    return values[sorted(retained)].copy()


def retime_message(message, robot_model):
    """Apply MoveIt's TOTG and Ruckig without invoking a planning pipeline."""
    from moveit.core.robot_state import RobotState
    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMessage
    from trajectory_msgs.msg import JointTrajectoryPoint

    if not message['points']:
        raise ValueError('EMPTY_MESSAGE')
    first_name = message['joint_names'][0]
    if len(message['joint_names']) == 7:
        group = 'left_arm' if first_name.startswith('left_') else 'right_arm'
    else:
        group = ('left_gripper' if first_name.startswith('left_')
                 else 'right_gripper')
    positions = np.asarray(
        [record['positions'] for record in message['points']], dtype=float)
    positions = simplify_collinear_positions(positions)
    boundaries = [0]
    if len(positions) > 2:
        differences = np.diff(positions, axis=0)
        for index in range(1, len(differences)):
            before, after = differences[index - 1], differences[index]
            denominator = float(np.linalg.norm(before) * np.linalg.norm(after))
            if denominator > 1e-12:
                cosine = float(np.dot(before, after) / denominator)
                if cosine < -0.95:
                    boundaries.append(index)
    boundaries.append(len(positions) - 1)
    boundaries = sorted(set(boundaries))
    output_points = []
    total_duration = 0.0
    for piece_index, (begin, end) in enumerate(
            zip(boundaries[:-1], boundaries[1:])):
        if end <= begin:
            continue
        reference = RobotState(robot_model)
        reference.set_to_default_values()
        reference.set_joint_group_active_positions(group, positions[begin])
        reference.update()
        payload = RobotTrajectoryMessage()
        payload.joint_trajectory.joint_names = list(message['joint_names'])
        for values in positions[begin:end + 1]:
            point = JointTrajectoryPoint()
            point.positions = values.tolist()
            payload.joint_trajectory.points.append(point)
        trajectory = RobotTrajectory(robot_model)
        trajectory.joint_model_group_name = group
        trajectory.set_robot_trajectory_msg(reference, payload)
        if not trajectory.apply_totg_time_parameterization(
                0.90, 0.80, path_tolerance=0.001,
                resample_dt=0.010, min_angle_change=0.00001):
            piece = positions[begin:end + 1]
            span = float(np.max(np.abs(piece[-1] - piece[0])))
            maximum_step = float(np.max(np.abs(np.diff(piece, axis=0))))
            raise RuntimeError(
                f'TOTG:{group}:piece_{piece_index}/{len(boundaries) - 1}:'
                f'points_{len(piece)}:span_{span:.9f}:'
                f'max_step_{maximum_step:.9f}')
        result = trajectory.get_robot_trajectory_msg(
            list(message['joint_names']))
        if list(result.joint_trajectory.joint_names) != message['joint_names']:
            raise RuntimeError(f'TIMING_JOINT_ORDER:{group}')
        sources = result.joint_trajectory.points
        if piece_index:
            sources = sources[1:]
        for source in sources:
            source_seconds = float(source.time_from_start.sec)
            source_seconds += source.time_from_start.nanosec * 1e-9
            shifted = total_duration + source_seconds
            output_points.append({
                'positions': [float(value) for value in source.positions],
                'velocities': [float(value) for value in source.velocities],
                'accelerations': [
                    float(value) for value in source.accelerations],
                'effort': [float(value) for value in source.effort],
                'time_from_start': seconds_duration(shifted),
            })
        total_duration += float(trajectory.duration)
    message['points'] = output_points
    velocity_limit = (
        ARM_VELOCITY if len(message['joint_names']) == 7
        else GRIPPER_VELOCITY)
    acceleration_limit = (
        ARM_ACCELERATION if len(message['joint_names']) == 7
        else GRIPPER_ACCELERATION)
    velocity_ratio = acceleration_ratio = 0.0
    for point in message['points']:
        velocity = np.asarray(point['velocities'], dtype=float)
        acceleration = np.asarray(point['accelerations'], dtype=float)
        if len(velocity):
            velocity_ratio = max(
                velocity_ratio,
                float(np.max(np.abs(velocity) / velocity_limit)))
        if len(acceleration):
            acceleration_ratio = max(
                acceleration_ratio,
                float(np.max(np.abs(acceleration) / acceleration_limit)))
    factor = max(1.0, velocity_ratio, math.sqrt(acceleration_ratio))
    factor *= 1.000001
    if factor > 1.0:
        for point in message['points']:
            seconds = duration_seconds(point['time_from_start']) * factor
            point['time_from_start'] = seconds_duration(seconds)
            point['velocities'] = [
                value / factor for value in point['velocities']]
            point['accelerations'] = [
                value / (factor * factor)
                for value in point['accelerations']]
        total_duration *= factor
    return total_duration


def merge_contiguous_messages(stage):
    """Merge adjacent commands for one controller before retiming."""
    merged = []
    for source in stage['messages']:
        same_controller = all((
            bool(merged),
            not merged or merged[-1]['joint_names'] == source['joint_names'],
            not merged or merged[-1]['topic'] == source['topic'],
        ))
        if same_controller:
            points = copy.deepcopy(source['points'])
            if (points and merged[-1]['points'] and np.allclose(
                    points[0]['positions'],
                    merged[-1]['points'][-1]['positions'],
                    atol=1e-12, rtol=0)):
                points = points[1:]
            merged[-1]['points'].extend(points)
        else:
            merged.append(copy.deepcopy(source))
    for sequence, message in enumerate(merged):
        message['sequence'] = sequence
    stage['messages'] = merged


def retime_stage(stage, robot_model):
    merge_contiguous_messages(stage)
    offset = 0.0
    for message in stage['messages']:
        message['sequence'] = int(message.get('sequence', 0))
        message['wall_offset_ns'] = int(round(offset * 1e9))
        try:
            duration = retime_message(message, robot_model)
        except RuntimeError as error:
            raise RuntimeError(
                f'{error}:{stage["name"]}:message_{message["sequence"]}') \
                from error
        offset += duration + MESSAGE_GAP_S
    stage['duration_s'] = max(0.0, offset - MESSAGE_GAP_S)
    return stage['duration_s']


def duration_seconds(value):
    return value['sec'] + value['nanosec'] * 1e-9


def mirror_reverse_timing(forward, reverse):
    """Mirror a retimed stage exactly, including velocity and acceleration."""
    forward_by_width = {
        width: [message for message in forward['messages']
                if len(message['joint_names']) == width]
        for width in (2, 7)}
    reverse_by_width = {
        width: [message for message in reverse['messages']
                if len(message['joint_names']) == width]
        for width in (2, 7)}
    for width in (2, 7):
        sources = list(reversed(forward_by_width[width]))
        targets = reverse_by_width[width]
        if len(sources) != len(targets):
            raise RuntimeError('REVERSE_TIMING_TOPOLOGY')
        for source, target in zip(sources, targets):
            duration = duration_seconds(
                source['points'][-1]['time_from_start'])
            if len(source['points']) != len(target['points']):
                raise RuntimeError('REVERSE_TIMING_POINT_COUNT')
            for source_point, target_point in zip(
                    reversed(source['points']), target['points']):
                mirrored_time = duration - duration_seconds(
                    source_point['time_from_start'])
                target_point['time_from_start'] = seconds_duration(
                    mirrored_time)
                target_point['velocities'] = [
                    -float(value) for value in source_point['velocities']]
                target_point['accelerations'] = [
                    float(value) for value in source_point['accelerations']]
                target_point['effort'] = [
                    float(value) for value in source_point['effort']]
    offset = 0.0
    for message in reverse['messages']:
        message['wall_offset_ns'] = int(round(offset * 1e9))
        offset += duration_seconds(
            message['points'][-1]['time_from_start']) + MESSAGE_GAP_S
    reverse['duration_s'] = max(0.0, offset - MESSAGE_GAP_S)
    return reverse['duration_s']


def maximum_change(stages):
    maximum = 0.0
    previous = {}
    for stage in stages:
        for message in stage['messages']:
            key = tuple(message['joint_names'])
            for point in message['points']:
                value = np.asarray(point['positions'], dtype=float)
                if key in previous:
                    maximum = max(
                        maximum, float(np.max(np.abs(value - previous[key]))))
                previous[key] = value
    return maximum


def timing_audit(stages):
    maximum_velocity_ratio = maximum_acceleration_ratio = 0.0
    valid = True
    for stage in stages:
        for message in stage['messages']:
            width = len(message['joint_names'])
            velocity_limit = ARM_VELOCITY if width == 7 else GRIPPER_VELOCITY
            acceleration_limit = (
                ARM_ACCELERATION if width == 7 else GRIPPER_ACCELERATION)
            last_time = -1.0
            for point in message['points']:
                duration = point['time_from_start']
                current = duration['sec'] + duration['nanosec'] * 1e-9
                valid = valid and current >= last_time and current >= 0
                last_time = current
                positions = np.asarray(point['positions'], dtype=float)
                valid = valid and np.isfinite(positions).all()
                if width == 2:
                    valid = valid and bool(np.all(positions >= -0.0205))
                    valid = valid and bool(np.all(positions <= 0.0255))
                else:
                    valid = valid and bool(np.all(np.abs(positions) < math.pi))
                velocity = np.asarray(point['velocities'], dtype=float)
                acceleration = np.asarray(point['accelerations'], dtype=float)
                if len(velocity):
                    maximum_velocity_ratio = max(
                        maximum_velocity_ratio,
                        float(np.max(np.abs(velocity) / velocity_limit)))
                if len(acceleration):
                    maximum_acceleration_ratio = max(
                        maximum_acceleration_ratio,
                        float(np.max(
                            np.abs(acceleration) / acceleration_limit)))
    valid = valid and maximum_velocity_ratio <= 1.0 + 1e-6
    valid = valid and maximum_acceleration_ratio <= 1.0 + 1e-6
    return valid, maximum_velocity_ratio, maximum_acceleration_ratio


def reverse_audit(stages):
    pairs = ((0, 6), (1, 5), (2, 4))
    for forward_index, reverse_index in pairs:
        forward = list(stages[forward_index]['messages'])
        reverse = list(stages[reverse_index]['messages'])
        forward_by_width = {
            width: [message for message in forward
                    if len(message['joint_names']) == width]
            for width in (2, 7)}
        reverse_by_width = {
            width: [message for message in reverse
                    if len(message['joint_names']) == width]
            for width in (2, 7)}
        for width in (2, 7):
            expected = [
                point['positions']
                for message in forward_by_width[width]
                for point in message['points']
            ]
            actual = [
                point['positions']
                for message in reverse_by_width[width]
                for point in message['points']
            ]
            if len(expected) != len(actual):
                return False
            if expected and not np.allclose(
                    np.asarray(expected), np.asarray(actual)[::-1],
                    atol=1e-12, rtol=0):
                return False
    return True


def hinge_audit(epoch, cabinet, door):
    relative = np.linalg.inv(cabinet) @ door
    nominal_translation = np.asarray([-0.140045922147, -0.247963042272, 0.020])
    translation_error = float(np.linalg.norm(
        relative[:3, 3] - nominal_translation))
    try:
        index = epoch.mechanism_state.name.index('door_hinge_joint')
        hinge = float(epoch.mechanism_state.position[index])
    except (ValueError, IndexError):
        return False, translation_error, math.inf
    expected = yaw_transform(nominal_translation, hinge)
    rotation_error = rotation_angle(expected @ np.linalg.inv(relative))
    passed = all((
        translation_error <= 0.005,
        rotation_error <= math.radians(2.0),
    ))
    return passed, translation_error, rotation_error


def cpp_clearance_audit(args, moveit_config, epoch, stages):
    """Run the file-only C++ world-distance and collision replay."""
    run_directory = Path(args.output).parent
    run_directory.mkdir(parents=True, exist_ok=True)
    audit_plan = {
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'candidate3_only': True,
        'contains_partial': False,
        'door_arc_target_count': DOOR_ARC_TARGET_COUNT,
        'stages': stages,
    }
    audit_hash = canonical_hash(audit_plan)
    input_path = run_directory / 'clearance_audit_input.json'
    urdf_path = run_directory / 'clearance_audit_robot.urdf'
    srdf_path = run_directory / 'clearance_audit_robot.srdf'
    result_path = run_directory / 'clearance_audit_result.json'
    stdout_path = run_directory / 'clearance_auditor.stdout.log'
    stderr_path = run_directory / 'clearance_auditor.stderr.log'
    input_path.write_text(
        json.dumps(
            {'plan': audit_plan, 'plan_hash': audit_hash},
            sort_keys=True, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')
    urdf_path.write_text(moveit_config['robot_description'], encoding='utf-8')
    srdf_path.write_text(
        moveit_config['robot_description_semantic'], encoding='utf-8')
    command = [
        args.clearance_auditor,
        '--scene', args.scene,
        '--plan', str(input_path),
        '--urdf', str(urdf_path),
        '--srdf', str(srdf_path),
        '--output', str(result_path),
    ]
    with stdout_path.open('wb') as stdout_file, (
            stderr_path.open('wb')) as stderr_file:
        completed = subprocess.run(
            command, stdin=subprocess.DEVNULL,
            stdout=stdout_file, stderr=stderr_file,
            timeout=args.clearance_timeout_s, check=False)
    if completed.returncode not in (0, 2) or not result_path.is_file():
        raise RuntimeError(
            f'CPP_CLEARANCE_AUDITOR:exit={completed.returncode}')
    result = json.loads(result_path.read_text(encoding='utf-8'))
    expected_scene = f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'
    if any((
            result.get('schema') !=
            'h1_candidate3_cpp_clearance_audit/v1',
            result.get('scene_name') != expected_scene,
            result.get('plan_hash') != audit_hash,
            not isinstance(result.get('samples'), int),
            int(result.get('samples', 0)) <= 0,
            not isinstance(result.get('collision_free'), bool),
            not isinstance(result.get('clearance_passed'), bool),
            not math.isfinite(float(
                result.get('minimum_environment_clearance_m', math.nan))),
    )):
        raise RuntimeError('CPP_CLEARANCE_AUDITOR_CONTRACT')
    return result, audit_hash


def build_result(args):
    global _MOVEIT_KEEPALIVE
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene as PlanningSceneMessage
    from rclpy.serialization import deserialize_message

    epoch = deserialize_message(Path(args.epoch).read_bytes(), SceneEpoch)
    scene_message = deserialize_message(
        Path(args.scene).read_bytes(), PlanningSceneMessage)
    expected_scene_name = f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'
    if not epoch.valid or not epoch.stable:
        raise ValueError(f'EPOCH_INVALID:{epoch.status}')
    alignment_prefix = 'FROZEN_POST_NAV_ALIGNMENT:'
    post_nav_alignment_hash = ''
    if epoch.status.startswith(alignment_prefix):
        post_nav_alignment_hash = epoch.status[len(alignment_prefix):]
        if (len(post_nav_alignment_hash) != 64 or any(
                value not in '0123456789abcdef'
                for value in post_nav_alignment_hash)):
            raise ValueError('POST_NAV_ALIGNMENT_HASH')
    if scene_message.name != expected_scene_name or scene_message.is_diff:
        raise ValueError('SCENE_EPOCH_BINDING')
    assets, cache_hash = load_sealed_assets(Path(args.asset_root))
    alignment = recover_scene_alignment(epoch, scene_message)
    cabinet = alignment['world']['cabinet']
    cube = alignment['world']['cube']
    door = alignment['world']['door']
    cabinet_delta = alignment['cabinet_delta']
    cube_delta = alignment['cube_delta']

    from moveit.planning import MoveItPy

    moveit_config = moveit_configuration()
    moveit = MoveItPy(
        node_name=f'candidate3_worker_{os.getpid()}',
        config_dict=moveit_config)
    # Retain the instance until os._exit(), including fail-closed paths.  Jazzy
    # MoveItPy can fault if its executor is destroyed during exception unwind.
    _MOVEIT_KEEPALIVE = moveit
    planner = Replanner(moveit.get_robot_model(), cabinet_delta, cube_delta)
    start = time.monotonic()
    handle = planner.stage(0, assets[0]['payload'])
    left_clear = planner.stage(1, assets[1]['payload'])
    door_open = planner.stage(2, assets[2]['payload'])
    cube_stage = planner.stage(3, assets[3]['payload'])
    forward_stages = [handle, left_clear, door_open, cube_stage]
    forward_duration = sum(
        retime_stage(stage, moveit.get_robot_model())
        for stage in forward_stages)
    door_close = reverse_stage(
        door_open, assets[4]['payload'], STAGE_NAMES[4])
    left_regrasp = reverse_stage(
        left_clear, assets[5]['payload'], STAGE_NAMES[5])
    handle_relock = reverse_stage(
        handle, assets[6]['payload'], STAGE_NAMES[6])
    reverse_duration = sum((
        mirror_reverse_timing(door_open, door_close),
        mirror_reverse_timing(left_clear, left_regrasp),
        mirror_reverse_timing(handle, handle_relock),
    ))
    stages = [handle, left_clear, door_open, cube_stage,
              door_close, left_regrasp, handle_relock]
    total_duration = forward_duration + reverse_duration
    plan_seconds = time.monotonic() - start
    adjacent_change = maximum_change(stages)
    timing_passed, velocity_ratio, acceleration_ratio = timing_audit(stages)
    reverse_passed = reverse_audit(stages)
    hinge_passed, hinge_translation_error, hinge_rotation_error = hinge_audit(
        epoch, cabinet, door)

    cpp_audit, clearance_audit_hash = cpp_clearance_audit(
        args, moveit_config, epoch, stages)
    collision_free = bool(cpp_audit['collision_free'])
    clearance_passed = bool(cpp_audit['clearance_passed'])
    clearance = float(cpp_audit['minimum_environment_clearance_m'])
    collision_samples = int(cpp_audit['samples'])
    collision_status = str(cpp_audit['status'])
    support_passed = all((
        epoch.latch_state.strip().lower() == 'locked',
        epoch.right_door_support_state.startswith('DETACHED'),
        epoch.left_cube_support_state.startswith('DETACHED'),
    ))
    continuity_passed = adjacent_change <= MAXIMUM_ADJACENT_CHANGE_RAD
    ik_complete = not planner.ik_failures
    original_stl_contact_passed = all((
        len(epoch.stl_hash) == 64,
        cache_hash == assets[3]['payload'][
            'cube_cache_contract']['cache_sha256'],
    ))
    action_time_passed = total_duration <= 90.0
    all_passed = all((
        ik_complete, continuity_passed, collision_free, clearance_passed,
        original_stl_contact_passed, hinge_passed, reverse_passed,
        timing_passed, support_passed, action_time_passed,
    ))
    if not ik_complete:
        final_status = 'IK_INCOMPLETE'
    elif not continuity_passed:
        final_status = 'IK_CONTINUITY'
    elif not collision_free or not clearance_passed:
        final_status = collision_status
    elif not original_stl_contact_passed:
        final_status = 'ORIGINAL_STL_CONTACT'
    elif not hinge_passed:
        final_status = 'HINGE_CONTRACT'
    elif not reverse_passed:
        final_status = 'REVERSE_PATH'
    elif not timing_passed:
        final_status = 'TIMING_LIMITS'
    elif not action_time_passed:
        final_status = f'ACTION_TIME:{total_duration:.6f}s'
    elif not support_passed:
        final_status = 'SUPPORT_TOPOLOGY'
    else:
        final_status = 'PASS'

    stage_hashes = [stage_hash(stage) for stage in stages]
    stage_waypoints = [sum(len(message['points'])
                           for message in stage['messages'])
                       for stage in stages]
    plan = {
        'schema': SCHEMA,
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'source_epoch_status': epoch.status,
        'post_nav_alignment_hash': post_nav_alignment_hash,
        'planner_version': PLANNER_VERSION,
        'topology': TOPOLOGY,
        'candidate3_only': True,
        'contains_partial': False,
        'door_arc_target_count': DOOR_ARC_TARGET_COUNT,
        'source_asset_hashes': {
            value['name']: value['payload_hash'] for value in assets},
        'source_cache_hash': cache_hash,
        'current_cabinet_pose': cabinet.tolist(),
        'current_cube_pose': cube.tolist(),
        'current_pelvis_cabinet_pose':
            alignment['pelvis_cabinet'].tolist(),
        'current_pelvis_door_pose': alignment['pelvis_door'].tolist(),
        'current_pelvis_handle_pose': alignment['pelvis_handle'].tolist(),
        'current_pelvis_cube_pose': alignment['pelvis_cube'].tolist(),
        'planning_from_world': alignment['planning_from_world'].tolist(),
        'base_pose_residual': alignment['base_residual'].tolist(),
        'base_residual_metrics': alignment['base_metrics'],
        'world_cabinet_delta':
            alignment['world']['cabinet_delta'].tolist(),
        'world_cube_delta': alignment['world']['cube_delta'].tolist(),
        'stage_names': list(STAGE_NAMES),
        'stage_hashes': stage_hashes,
        'stage_waypoint_counts': stage_waypoints,
        'clearance_audited_trajectory_hash': clearance_audit_hash,
        'planned_action_duration_s': total_duration,
        'planning_wall_duration_s': plan_seconds,
        'ik_calls': planner.ik_calls,
        'maximum_ik_translation_error_m':
            planner.maximum_ik_translation_error_m,
        'maximum_ik_rotation_error_rad':
            planner.maximum_ik_rotation_error_rad,
        'stages': stages,
    }
    plan_hash = canonical_hash(plan)
    certificate_payload = {
        'schema': 'h1_candidate3_plan_certificate/v1',
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'source_epoch_status': epoch.status,
        'post_nav_alignment_hash': post_nav_alignment_hash,
        'plan_hash': plan_hash,
        'clearance_audited_trajectory_hash': clearance_audit_hash,
        'robot_model_hash': epoch.robot_model_hash,
        'stl_hash': epoch.stl_hash,
        'tag_config_hash': epoch.tag_config_hash,
        'planner_version': PLANNER_VERSION,
        'auditor_version': AUDITOR_VERSION,
        'door_arc_target_count': DOOR_ARC_TARGET_COUNT,
        'forward_dense_samples': sum(stage_waypoints[:4]),
        'reverse_dense_samples': sum(stage_waypoints[4:]),
        'minimum_environment_clearance_m': clearance,
        'maximum_adjacent_joint_change_rad': adjacent_change,
        'candidate3_only': True,
        'ik_complete': ik_complete,
        'collision_free': collision_free,
        'original_stl_contact_passed': original_stl_contact_passed,
        'hinge_contract_passed': hinge_passed,
        'forward_passed': all_passed,
        'reverse_passed': reverse_passed,
        'timing_passed': timing_passed and action_time_passed,
        'support_topology_passed': support_passed,
        'no_partial': True,
        'validity_s': 0.50,
        'valid': all_passed,
        'status': final_status,
        'audit_details': {
            'collision_samples': collision_samples,
            'cpp_clearance_audit': cpp_audit,
            'ik_calls': planner.ik_calls,
            'maximum_ik_translation_error_m':
                planner.maximum_ik_translation_error_m,
            'maximum_ik_rotation_error_rad':
                planner.maximum_ik_rotation_error_rad,
            'planning_wall_duration_s': plan_seconds,
            'planned_action_duration_s': total_duration,
            'maximum_velocity_ratio': velocity_ratio,
            'maximum_acceleration_ratio': acceleration_ratio,
            'hinge_translation_error_m': hinge_translation_error,
            'hinge_rotation_error_rad': hinge_rotation_error,
            'scene_handle_translation_error_m':
                alignment['handle_translation_error_m'],
            'scene_handle_rotation_error_rad':
                alignment['handle_rotation_error_rad'],
            'scoped_clearance_exclusions': [
                'incubator_handle_exact_after_handle_contact',
                'incubator_door_exact_while_right_hand_supports_door',
                'green_cube_dynamic_exact_while_left_hand_supports_cube',
                'green_cube_10mm_guard_while_left_hand_supports_cube',
                ],
            'original_stl_contact_method': (
                'sealed exact-STL/cache hashes plus object-rigid TCP target '
                'transform invariance'),
        },
    }
    certificate_hash = canonical_hash(certificate_payload)
    return {
        'plan': plan,
        'plan_hash': plan_hash,
        'certificate': certificate_payload,
        'certificate_hash': certificate_hash,
    }


def failure_result(status):
    return {
        'plan': None,
        'plan_hash': '',
        'certificate': {
            'schema': 'h1_candidate3_plan_certificate/v1',
            'planner_version': PLANNER_VERSION,
            'auditor_version': AUDITOR_VERSION,
            'candidate3_only': True,
            'no_partial': True,
            'valid': False,
            'status': status,
        },
        'certificate_hash': '',
    }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--epoch', required=True)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--asset-root', required=True)
    parser.add_argument('--clearance-auditor', required=True)
    parser.add_argument('--clearance-timeout-s', type=float, default=300.0)
    parser.add_argument('--output', required=True)
    return parser.parse_args(argv)


def main(argv=None):
    faulthandler.enable()
    args = parse_arguments(argv)
    exit_code = 0
    try:
        result = build_result(args)
    except Exception as error:
        result = failure_result(f'{type(error).__name__}:{error}')
        exit_code = 2
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, sort_keys=True, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')
    print(json.dumps({
        'valid': result['certificate']['valid'],
        'status': result['certificate']['status'],
        'plan_hash': result['plan_hash'],
        'certificate_hash': result['certificate_hash'],
        'output': str(output),
    }, sort_keys=True), flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    # MoveItPy Jazzy can fault during interpreter teardown after its rclcpp
    # executor is destroyed.  The worker has no controller clients and exits
    # only after the immutable output has been flushed.
    os._exit(exit_code)


if __name__ == '__main__':
    main()
