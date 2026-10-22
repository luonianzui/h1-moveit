"""Derive and run an offline post-navigation base-residual matrix."""

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

from .candidate3_alignment import (
    planning_from_base_residual,
    recover_scene_alignment,
    validate_se3,
)
from .candidate3_contract import pose_matrix, quaternion_from_matrix


SCHEMA = 'h1_candidate3_base_residual_matrix/v1'


def file_sha256(path):
    """Return the SHA-256 digest of one file."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


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


def case_base_residual(entry):
    """Convert one human-readable case to a proper base SE(3)."""
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(
        math.radians(float(entry.get('base_roll_deg', 0.0))),
        math.radians(float(entry.get('base_pitch_deg', 0.0))),
        math.radians(float(entry.get('base_yaw_deg', 0.0))))
    result[:3, 3] = [
        float(entry.get('base_x_mm', 0.0)) / 1000.0,
        float(entry.get('base_y_mm', 0.0)) / 1000.0,
        float(entry.get('base_z_mm', 0.0)) / 1000.0,
    ]
    return validate_se3(result, f"MATRIX_CASE:{entry['case']}")


def assign_pose(message, value):
    """Assign an SE(3) matrix to a geometry Pose."""
    value = validate_se3(value, 'ASSIGN_POSE')
    quaternion = quaternion_from_matrix(value[:3, :3])
    message.position.x = float(value[0, 3])
    message.position.y = float(value[1, 3])
    message.position.z = float(value[2, 3])
    message.orientation.x = float(quaternion[0])
    message.orientation.y = float(quaternion[1])
    message.orientation.z = float(quaternion[2])
    message.orientation.w = float(quaternion[3])


def assign_transform(message, value):
    """Assign an SE(3) matrix to a geometry Transform."""
    value = validate_se3(value, 'ASSIGN_TRANSFORM')
    quaternion = quaternion_from_matrix(value[:3, :3])
    message.translation.x = float(value[0, 3])
    message.translation.y = float(value[1, 3])
    message.translation.z = float(value[2, 3])
    message.rotation.x = float(quaternion[0])
    message.rotation.y = float(quaternion[1])
    message.rotation.z = float(quaternion[2])
    message.rotation.w = float(quaternion[3])


def _seal_pair(epoch, scene, serialize_message):
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
        'epoch_status': epoch.status,
    }, sort_keys=True, separators=(',', ':')).encode('utf-8')
    digest = hashlib.sha256()
    digest.update(canonical)
    digest.update(serialize_message(epoch.robot_state))
    digest.update(serialize_message(epoch.mechanism_state))
    digest.update(serialize_message(scene))
    epoch.scene_hash = digest.hexdigest()
    scene.name = f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'


def derive_case(baseline_epoch, baseline_scene, entry, epoch_id,
                serialize_message):
    """Move the entire frozen world relative to an unchanged epoch."""
    epoch = copy.deepcopy(baseline_epoch)
    scene = copy.deepcopy(baseline_scene)
    baseline = recover_scene_alignment(baseline_epoch, baseline_scene)
    requested_base = case_base_residual(entry)
    planning_from_world = planning_from_base_residual(requested_base)
    frame_delta = validate_se3(
        planning_from_world @ np.linalg.inv(
            baseline['planning_from_world']),
        f"MATRIX_FRAME_DELTA:{entry['case']}")

    for value in scene.world.collision_objects:
        if value.header.frame_id != 'pelvis':
            raise ValueError(f'MATRIX_WORLD_OBJECT_FRAME:{value.id}')
        assign_pose(value.pose, frame_delta @ pose_matrix(value.pose))
    assign_pose(
        scene.world.octomap.origin,
        frame_delta @ pose_matrix(scene.world.octomap.origin))
    world_links = [
        value for value in scene.fixed_frame_transforms
        if value.header.frame_id == 'world' and
        value.child_frame_id == 'pelvis'
    ]
    if len(world_links) != 1:
        raise ValueError('MATRIX_FIXED_WORLD_PELVIS')
    assign_transform(world_links[0].transform, planning_from_world)

    epoch.epoch_id = int(epoch_id)
    epoch.valid = True
    epoch.stable = True
    epoch.status = 'FROZEN_PLAN_ONLY_BASE_RESIDUAL_DERIVED'
    _seal_pair(epoch, scene, serialize_message)
    return epoch, scene, requested_base, frame_delta


def _full_audit(result):
    plan = result.get('plan')
    certificate = result.get('certificate') or {}
    details = certificate.get('audit_details') or {}
    cpp = details.get('cpp_clearance_audit') or {}
    return bool(
        plan and certificate.get('valid') is True and
        certificate.get('status') == 'PASS' and
        plan.get('contains_partial') is False and
        plan.get('door_arc_target_count') == 601 and
        len(plan.get('stages', ())) == 7 and
        float(certificate.get('minimum_environment_clearance_m', -1.0)) >=
        0.010 and
        float(certificate.get('maximum_adjacent_joint_change_rad', 99.0)) <=
        math.radians(4.9) and
        certificate.get('reverse_passed') is True and
        certificate.get('timing_passed') is True and
        certificate.get('no_partial') is True and
        cpp.get('collision_free') is True and
        cpp.get('clearance_passed') is True and
        int(cpp.get('samples', 0)) == sum(
            int(value) for value in plan.get('stage_waypoint_counts', ())))


def compact_result(entry, epoch, requested_base, frame_delta,
                   case_directory, completed):
    """Reduce a full worker result to matrix audit facts."""
    result_path = case_directory / 'result.json'
    result = (json.loads(result_path.read_text(encoding='utf-8'))
              if result_path.is_file() else {})
    plan = result.get('plan')
    certificate = result.get('certificate') or {}
    details = certificate.get('audit_details') or {}
    cpp = details.get('cpp_clearance_audit') or {}
    status = str(certificate.get(
        'status', f'WORKER_EXIT:{completed.returncode}'))
    expected = entry['expected']
    if expected == 'PASS':
        expectation_matched = completed.returncode == 0 and _full_audit(result)
    elif expected == 'REJECT':
        expectation_matched = bool(
            certificate.get('valid') is False and
            certificate.get('no_partial') is True and
            (plan is None or plan.get('contains_partial') is False) and
            not _full_audit(result) and
            entry['status_contains'] in status)
    else:
        raise ValueError(f'MATRIX_EXPECTED:{expected}')
    return {
        'case': entry['case'],
        'requested': {
            key: entry[key] for key in (
                'base_x_mm', 'base_y_mm', 'base_z_mm',
                'base_roll_deg', 'base_pitch_deg', 'base_yaw_deg')
            if key in entry
        },
        'expected': expected,
        'status_contains': entry.get('status_contains', ''),
        'expectation_matched': expectation_matched,
        'worker_exit_code': int(completed.returncode),
        'status': status,
        'valid': certificate.get('valid') is True,
        'no_partial': certificate.get('no_partial') is True,
        'full_audit_passed': _full_audit(result),
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'requested_base_residual': requested_base.tolist(),
        'scene_frame_delta': frame_delta.tolist(),
        'epoch_cdr_sha256': file_sha256(case_directory / 'epoch.cdr'),
        'scene_cdr_sha256': file_sha256(case_directory / 'scene.cdr'),
        'result_sha256': file_sha256(result_path)
        if result_path.is_file() else '',
        'plan_hash': result.get('plan_hash', ''),
        'certificate_hash': result.get('certificate_hash', ''),
        'planned_action_duration_s':
            plan.get('planned_action_duration_s') if plan else None,
        'planning_wall_duration_s':
            plan.get('planning_wall_duration_s') if plan else None,
        'ik_calls': plan.get('ik_calls') if plan else None,
        'door_arc_target_count':
            plan.get('door_arc_target_count') if plan else None,
        'trajectory_samples': cpp.get('samples'),
        'minimum_environment_clearance_m':
            certificate.get('minimum_environment_clearance_m'),
        'maximum_adjacent_joint_change_rad':
            certificate.get('maximum_adjacent_joint_change_rad'),
        'reverse_passed': certificate.get('reverse_passed'),
        'timing_passed': certificate.get('timing_passed'),
    }


def parse_arguments(argv=None):
    """Parse the file-only matrix runner arguments."""
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
    """Run every immutable CDR-derived case without control authority."""
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message

    args = parse_arguments(argv)
    epoch_path = Path(args.baseline_epoch).resolve()
    scene_path = Path(args.baseline_scene).resolve()
    matrix_path = Path(args.matrix).resolve()
    output = Path(args.output_directory).resolve()
    matrix = json.loads(matrix_path.read_text(encoding='utf-8'))
    if matrix.get('schema') != SCHEMA:
        raise ValueError('MATRIX_SCHEMA')
    expected_hashes = matrix['baseline_cdr_sha256']
    if file_sha256(epoch_path) != expected_hashes['epoch']:
        raise ValueError('BASELINE_EPOCH_SHA256')
    if file_sha256(scene_path) != expected_hashes['scene']:
        raise ValueError('BASELINE_SCENE_SHA256')
    output.mkdir(parents=True, exist_ok=False)
    baseline_epoch = deserialize_message(epoch_path.read_bytes(), SceneEpoch)
    baseline_scene = deserialize_message(
        scene_path.read_bytes(), PlanningScene)
    results = []
    for index, entry in enumerate(matrix['cases'], start=1):
        case_directory = output / entry['case']
        case_directory.mkdir(exist_ok=False)
        epoch, scene, requested_base, frame_delta = derive_case(
            baseline_epoch, baseline_scene, entry,
            int(baseline_epoch.epoch_id) + index, serialize_message)
        derived_epoch = case_directory / 'epoch.cdr'
        derived_scene = case_directory / 'scene.cdr'
        derived_epoch.write_bytes(serialize_message(epoch))
        derived_scene.write_bytes(serialize_message(scene))
        command = [
            sys.executable, '-m', 'h1_2_vision.candidate3_worker',
            '--epoch', str(derived_epoch),
            '--scene', str(derived_scene),
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
            entry, epoch, requested_base, frame_delta,
            case_directory, completed)
        results.append(record)
        summary = {
            'schema': SCHEMA,
            'matrix_sha256': file_sha256(matrix_path),
            'baseline_epoch_path': str(epoch_path),
            'baseline_scene_path': str(scene_path),
            'baseline_epoch_cdr_sha256': file_sha256(epoch_path),
            'baseline_scene_cdr_sha256': file_sha256(scene_path),
            'baseline_world_poses_changed': False,
            'robot_motion_allowed': False,
            'robot_trajectory_commands_sent': 0,
            'gazebo_started': False,
            'case_count': len(matrix['cases']),
            'executed_count': len(results),
            'full_audit_pass_count': sum(
                value['full_audit_passed'] for value in results),
            'expected_rejection_count': sum(
                value['expected'] == 'REJECT' for value in results),
            'expectation_match_count': sum(
                value['expectation_matched'] for value in results),
            'complete': len(results) == len(matrix['cases']),
            'all_expectations_matched': bool(
                len(results) == len(matrix['cases']) and
                all(value['expectation_matched'] for value in results)),
            'cases': results,
        }
        (output / 'summary.json').write_text(
            json.dumps(summary, sort_keys=True, indent=2,
                       allow_nan=False) + '\n',
            encoding='utf-8')
        print(json.dumps({
            'case': entry['case'],
            'index': index,
            'count': len(matrix['cases']),
            'status': record['status'],
            'expectation_matched': record['expectation_matched'],
        }, sort_keys=True), flush=True)
    return 0 if summary['all_expectations_matched'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
