"""
File-only stage-I perturbation runner for candidate-3.

The runner derives immutable SceneEpoch/PlanningScene pairs from one audited
baseline and invokes the plan-only worker in isolated subprocesses.  It does
not create ROS nodes, publishers, action clients, or controller clients.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

from .candidate3_contract import pose_matrix, quaternion_from_matrix, yaw_transform
from .candidate3_worker import REFERENCE_CABINET, REFERENCE_CUBE


SCHEMA = 'h1_candidate3_stage_i_positive_matrix/v1'
CABINET_OBJECT_IDS = frozenset((
    'incubator_body_exact',
    'incubator_door_exact',
    'incubator_handle_exact',
))
CUBE_OBJECT_IDS = frozenset((
    'green_cube_dynamic_exact',
    'green_cube_10mm_guard',
))


def case_offsets(entry):
    """Normalize one stage-I matrix entry to SI-unit perturbations."""
    parameter = entry['parameter']
    value = entry['value']
    offsets = {
        'cabinet_x_m': 0.0,
        'cabinet_y_m': 0.0,
        'cabinet_yaw_rad': 0.0,
        'cube_x_m': 0.0,
        'cube_y_m': 0.0,
        'cube_yaw_rad': 0.0,
    }
    if parameter == 'cabinet_x_mm':
        offsets['cabinet_x_m'] = float(value) / 1000.0
    elif parameter == 'cabinet_y_mm':
        offsets['cabinet_y_m'] = float(value) / 1000.0
    elif parameter == 'cabinet_yaw_deg':
        offsets['cabinet_yaw_rad'] = math.radians(float(value))
    elif parameter == 'cube_xy_mm':
        offsets['cube_x_m'] = float(value[0]) / 1000.0
        offsets['cube_y_m'] = float(value[1]) / 1000.0
    elif parameter == 'cube_yaw_deg':
        offsets['cube_yaw_rad'] = math.radians(float(value))
    elif parameter == 'cabinet_xy_yaw_cube_xy_yaw':
        offsets.update({
            'cabinet_x_m': float(value['cabinet_x_mm']) / 1000.0,
            'cabinet_y_m': float(value['cabinet_y_mm']) / 1000.0,
            'cabinet_yaw_rad': math.radians(
                float(value['cabinet_yaw_deg'])),
            'cube_x_m': float(value['cube_x_mm']) / 1000.0,
            'cube_y_m': float(value['cube_y_mm']) / 1000.0,
            'cube_yaw_rad': math.radians(float(value['cube_yaw_deg'])),
        })
    else:
        raise ValueError(f'unknown stage-I parameter: {parameter}')
    return offsets


def assign_pose(message, value):
    """Assign one finite SE(3) matrix to a geometry pose message."""
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('invalid pose matrix')
    quaternion = quaternion_from_matrix(value[:3, :3])
    message.position.x = float(value[0, 3])
    message.position.y = float(value[1, 3])
    message.position.z = float(value[2, 3])
    message.orientation.x = float(quaternion[0])
    message.orientation.y = float(quaternion[1])
    message.orientation.z = float(quaternion[2])
    message.orientation.w = float(quaternion[3])


def object_pose_matrix(collision_object):
    return pose_matrix(collision_object.pose)


def target_poses(offsets):
    cabinet = yaw_transform([
        REFERENCE_CABINET[0, 3] + offsets['cabinet_x_m'],
        REFERENCE_CABINET[1, 3] + offsets['cabinet_y_m'],
        REFERENCE_CABINET[2, 3],
    ], offsets['cabinet_yaw_rad'])
    cube = yaw_transform([
        REFERENCE_CUBE[0, 3] + offsets['cube_x_m'],
        REFERENCE_CUBE[1, 3] + offsets['cube_y_m'],
        REFERENCE_CUBE[2, 3],
    ], math.pi / 2 + offsets['cube_yaw_rad'])
    return cabinet, cube


def derive_case(baseline_epoch, baseline_scene, entry, epoch_id):
    """Derive a consistent positive-case epoch and exact collision scene."""
    from rclpy.serialization import serialize_message

    epoch = copy.deepcopy(baseline_epoch)
    scene = copy.deepcopy(baseline_scene)
    offsets = case_offsets(entry)
    cabinet_target, cube_target = target_poses(offsets)
    cabinet_source = pose_matrix(baseline_epoch.cabinet.pose_world.pose)
    cube_source = pose_matrix(baseline_epoch.cube.pose_world.pose)
    door_source = pose_matrix(baseline_epoch.door.pose_world.pose)
    cabinet_world_delta = cabinet_target @ np.linalg.inv(cabinet_source)
    cube_world_delta = cube_target @ np.linalg.inv(cube_source)

    assign_pose(epoch.cabinet.pose_world.pose, cabinet_target)
    assign_pose(
        epoch.door.pose_world.pose, cabinet_world_delta @ door_source)
    assign_pose(epoch.cube.pose_world.pose, cube_target)
    epoch.epoch_id = int(epoch_id)
    epoch.stable = True
    epoch.valid = True
    epoch.status = 'FROZEN_PLAN_ONLY_STAGE_I_DERIVED'

    objects = {value.id: value for value in scene.world.collision_objects}
    missing = sorted((CABINET_OBJECT_IDS | CUBE_OBJECT_IDS) - set(objects))
    if missing:
        raise ValueError(f'missing collision objects: {missing}')

    # Infer the exact fixed planning<-world transform from the baseline cube.
    # The frozen GetPlanningScene response canonicalizes shape poses into the
    # CollisionObject pose, so the rigid delta belongs on that object pose.
    planning_from_world = (
        object_pose_matrix(objects['green_cube_dynamic_exact']) @
        np.linalg.inv(cube_source))
    world_from_planning = np.linalg.inv(planning_from_world)
    cabinet_planning_delta = (
        planning_from_world @ cabinet_world_delta @ world_from_planning)
    cube_planning_delta = (
        planning_from_world @ cube_world_delta @ world_from_planning)
    for object_id in CABINET_OBJECT_IDS:
        value = objects[object_id]
        assign_pose(
            value.pose, cabinet_planning_delta @ object_pose_matrix(value))
    for object_id in CUBE_OBJECT_IDS:
        value = objects[object_id]
        assign_pose(
            value.pose, cube_planning_delta @ object_pose_matrix(value))

    # DynamicScene hashes the complete pre-name PlanningScene CDR plus the
    # immutable robot/mechanism state.  Emptying the prior epoch name makes the
    # offline derivation deterministic and avoids a circular hash dependency.
    scene.name = ''
    scene.is_diff = False
    canonical = json.dumps({
        'epoch_id': epoch.epoch_id,
        'tag_config_hash': epoch.tag_config_hash,
        'stl_hash': epoch.stl_hash,
        'robot_model_hash': epoch.robot_model_hash,
        'pointcloud_revision': epoch.pointcloud_revision,
        'latch': epoch.latch_state,
        'right_support': epoch.right_door_support_state,
        'left_support': epoch.left_cube_support_state,
    }, sort_keys=True, separators=(',', ':')).encode('utf-8')
    digest = hashlib.sha256()
    digest.update(canonical)
    digest.update(serialize_message(epoch.robot_state))
    digest.update(serialize_message(epoch.mechanism_state))
    digest.update(serialize_message(scene))
    epoch.scene_hash = digest.hexdigest()
    scene.name = f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'
    return epoch, scene, offsets


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def compact_result(entry, offsets, epoch, case_directory, completed):
    result_path = case_directory / 'result.json'
    record = {
        'case': entry['case'],
        'parameter': entry['parameter'],
        'value': entry['value'],
        'offsets_si': offsets,
        'executed': True,
        'worker_exit_code': int(completed.returncode),
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'epoch_cdr_sha256': file_sha256(case_directory / 'epoch.cdr'),
        'scene_cdr_sha256': file_sha256(case_directory / 'scene.cdr'),
        'result_sha256': file_sha256(result_path) if result_path.is_file() else '',
        'plan_hash': '',
        'certificate_hash': '',
        'full_path_generated': False,
        'valid': False,
        'status': f'WORKER_EXIT:{completed.returncode}',
    }
    if not result_path.is_file():
        return record
    result = json.loads(result_path.read_text(encoding='utf-8'))
    certificate = result.get('certificate') or {}
    plan = result.get('plan')
    record.update({
        'plan_hash': result.get('plan_hash', ''),
        'certificate_hash': result.get('certificate_hash', ''),
        'full_path_generated': bool(
            plan and plan.get('candidate3_only') is True and
            plan.get('contains_partial') is False and
            len(plan.get('stages', ())) == 7),
        'valid': certificate.get('valid') is True,
        'status': certificate.get('status', 'MISSING_CERTIFICATE_STATUS'),
    })
    if plan:
        record.update({
            'planned_action_duration_s':
                plan.get('planned_action_duration_s'),
            'planning_wall_duration_s':
                plan.get('planning_wall_duration_s'),
            'maximum_ik_translation_error_m':
                plan.get('maximum_ik_translation_error_m'),
            'maximum_ik_rotation_error_rad':
                plan.get('maximum_ik_rotation_error_rad'),
        })
    details = certificate.get('audit_details') or {}
    cpp = details.get('cpp_clearance_audit') or {}
    if details:
        record.update({
            'collision_samples': details.get('collision_samples'),
            'minimum_environment_clearance_m':
                certificate.get('minimum_environment_clearance_m'),
            'maximum_adjacent_joint_change_rad':
                certificate.get('maximum_adjacent_joint_change_rad'),
            'maximum_velocity_ratio': details.get('maximum_velocity_ratio'),
            'maximum_acceleration_ratio':
                details.get('maximum_acceleration_ratio'),
            'collision_free': cpp.get('collision_free'),
            'clearance_passed': cpp.get('clearance_passed'),
        })
    return record


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-epoch', required=True)
    parser.add_argument('--baseline-scene', required=True)
    parser.add_argument('--matrix', required=True)
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--asset-root', required=True)
    parser.add_argument('--clearance-auditor', required=True)
    parser.add_argument('--worker-timeout-s', type=float, default=360.0)
    parser.add_argument('--clearance-timeout-s', type=float, default=300.0)
    return parser.parse_args(argv)


def main(argv=None):
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message

    args = parse_arguments(argv)
    baseline_epoch_path = Path(args.baseline_epoch).resolve()
    baseline_scene_path = Path(args.baseline_scene).resolve()
    output_directory = Path(args.output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    matrix = json.loads(Path(args.matrix).read_text(encoding='utf-8'))
    entries = matrix['positive_matrix']
    if len(entries) != 37:
        raise ValueError(f'expected 37 positive cases, got {len(entries)}')
    baseline_epoch = deserialize_message(
        baseline_epoch_path.read_bytes(), SceneEpoch)
    baseline_scene = deserialize_message(
        baseline_scene_path.read_bytes(), PlanningScene)
    results = []
    for index, entry in enumerate(entries, start=1):
        case_directory = output_directory / entry['case']
        case_directory.mkdir(parents=True, exist_ok=True)
        epoch, scene, offsets = derive_case(
            baseline_epoch, baseline_scene, entry,
            int(baseline_epoch.epoch_id) + index)
        epoch_path = case_directory / 'epoch.cdr'
        scene_path = case_directory / 'scene.cdr'
        epoch_path.write_bytes(serialize_message(epoch))
        scene_path.write_bytes(serialize_message(scene))
        command = [
            sys.executable,
            '-m', 'h1_2_vision.candidate3_worker',
            '--epoch', str(epoch_path),
            '--scene', str(scene_path),
            '--asset-root', str(Path(args.asset_root).resolve()),
            '--clearance-auditor',
            str(Path(args.clearance_auditor).resolve()),
            '--clearance-timeout-s', str(args.clearance_timeout_s),
            '--output', str(case_directory / 'result.json'),
        ]
        with (case_directory / 'worker.stdout.log').open('wb') as stdout, (
                case_directory / 'worker.stderr.log').open('wb') as stderr:
            try:
                completed = subprocess.run(
                    command, stdin=subprocess.DEVNULL, stdout=stdout,
                    stderr=stderr, check=False,
                    timeout=args.worker_timeout_s)
            except subprocess.TimeoutExpired:
                completed = subprocess.CompletedProcess(command, 124)
        record = compact_result(
            entry, offsets, epoch, case_directory, completed)
        results.append(record)
        summary = {
            'schema': SCHEMA,
            'baseline_epoch_cdr_sha256': file_sha256(baseline_epoch_path),
            'baseline_scene_cdr_sha256': file_sha256(baseline_scene_path),
            'robot_motion_allowed': False,
            'robot_trajectory_commands_sent': 0,
            'case_count': len(entries),
            'executed_count': len(results),
            'pass_count': sum(
                value['valid'] and value['full_path_generated']
                for value in results),
            'failure_count': sum(
                not (value['valid'] and value['full_path_generated'])
                for value in results),
            'complete': len(results) == len(entries),
            'all_passed': (
                len(results) == len(entries) and
                all(value['valid'] and value['full_path_generated']
                    for value in results)),
            'cases': results,
        }
        (output_directory / 'summary.json').write_text(
            json.dumps(summary, sort_keys=True, indent=2, allow_nan=False) +
            '\n', encoding='utf-8')
        print(json.dumps({
            'case': record['case'],
            'index': index,
            'count': len(entries),
            'valid': record['valid'],
            'full_path_generated': record['full_path_generated'],
            'status': record['status'],
        }, sort_keys=True), flush=True)
    return 0 if summary['all_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
