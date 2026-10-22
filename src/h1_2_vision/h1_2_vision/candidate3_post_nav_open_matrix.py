"""Run a file-only attempt11 matrix for the post-navigation open plan."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

from .candidate3_base_residual_matrix import derive_case, file_sha256


SCHEMA = 'h1_candidate3_post_nav_open_matrix/v1'
WORKER_MODULE = 'h1_2_vision.candidate3_post_nav_open_worker'


def full_audit(result):
    """Check every safety result required by the plan-only D1 contract."""
    plan = result.get('plan') or {}
    certificate = result.get('certificate') or {}
    cpp = certificate.get('audit_details') or {}
    counts = plan.get('stage_waypoint_counts') or []
    return bool(
        certificate.get('valid') is True and
        certificate.get('status') == 'PASS' and
        certificate.get('ik_complete') is True and
        certificate.get('collision_free') is True and
        certificate.get('clearance_passed') is True and
        float(certificate.get(
            'minimum_environment_clearance_m', -1.0)) >= 0.010 and
        float(certificate.get(
            'maximum_adjacent_angular_change_rad', math.inf)) <=
        math.radians(4.9) and
        float(certificate.get(
            'maximum_adjacent_gripper_change_m', math.inf)) <= 0.001 and
        certificate.get('forward_passed') is True and
        certificate.get('reverse_passed') is True and
        certificate.get('timing_passed') is True and
        certificate.get('torso_below_45_degrees') is True and
        float(certificate.get(
            'maximum_absolute_torso_rad', math.inf)) < math.radians(45.0) and
        certificate.get('no_partial') is True and
        plan.get('contains_partial') is False and
        plan.get('door_arc_target_count') == 601 and
        plan.get('execution_stage_count') == 3 and
        plan.get('reverse_audit_stage_count') == 3 and
        len(plan.get('stages', ())) == 6 and
        len(counts) == 6 and
        cpp.get('valid') is True and
        cpp.get('collision_free') is True and
        cpp.get('clearance_passed') is True and
        int(cpp.get('samples', 0)) == sum(int(value) for value in counts))


def compact_result(entry, epoch, requested_base, frame_delta,
                   case_directory, completed):
    """Reduce the large plan to immutable boundary and audit facts."""
    result_path = case_directory / 'result.json'
    result = (json.loads(result_path.read_text(encoding='utf-8'))
              if result_path.is_file() else {})
    plan = result.get('plan') or {}
    certificate = result.get('certificate') or {}
    cpp = certificate.get('audit_details') or {}
    status = str(certificate.get(
        'status', f'WORKER_EXIT:{completed.returncode}'))
    complete_pass = full_audit(result)
    expected = entry.get('expected', 'OBSERVE')
    if expected == 'PASS':
        expectation_matched = completed.returncode == 0 and complete_pass
    elif expected == 'REJECT':
        expected_status = str(entry.get('status_contains', ''))
        expectation_matched = bool(
            completed.returncode != 0 and
            certificate.get('valid') is False and
            certificate.get('no_partial') is True and
            not complete_pass and
            expected_status in status)
    elif expected == 'OBSERVE':
        expectation_matched = True
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
        'full_audit_passed': complete_pass,
        'scene_epoch_id': int(epoch.epoch_id),
        'scene_hash': epoch.scene_hash,
        'requested_base_residual': requested_base.tolist(),
        'scene_frame_delta': frame_delta.tolist(),
        'epoch_cdr_sha256': file_sha256(case_directory / 'epoch.cdr'),
        'scene_cdr_sha256': file_sha256(case_directory / 'scene.cdr'),
        'result_sha256': (
            file_sha256(result_path) if result_path.is_file() else ''),
        'plan_hash': result.get('plan_hash', ''),
        'certificate_hash': result.get('certificate_hash', ''),
        'planning_wall_duration_s': plan.get('planning_wall_duration_s'),
        'planned_action_duration_s': plan.get('planned_action_duration_s'),
        'reverse_audit_duration_s': plan.get('reverse_audit_duration_s'),
        'ik_calls': plan.get('ik_calls'),
        'maximum_ik_translation_error_m':
            plan.get('maximum_ik_translation_error_m'),
        'maximum_ik_rotation_error_rad':
            plan.get('maximum_ik_rotation_error_rad'),
        'door_arc_target_count': plan.get('door_arc_target_count'),
        'stage_waypoint_counts': plan.get('stage_waypoint_counts'),
        'trajectory_samples': cpp.get('samples'),
        'minimum_environment_clearance_m':
            certificate.get('minimum_environment_clearance_m'),
        'maximum_adjacent_angular_change_rad':
            certificate.get('maximum_adjacent_angular_change_rad'),
        'maximum_adjacent_gripper_change_m':
            certificate.get('maximum_adjacent_gripper_change_m'),
        'maximum_absolute_torso_rad':
            certificate.get('maximum_absolute_torso_rad'),
        'torso_below_45_degrees':
            certificate.get('torso_below_45_degrees'),
        'forward_passed': certificate.get('forward_passed'),
        'reverse_passed': certificate.get('reverse_passed'),
        'timing_passed': certificate.get('timing_passed'),
        'audit_failure': {
            key: cpp.get(key) for key in (
                'status', 'stage', 'stage_index', 'message_index',
                'point_index', 'nearest_first_body', 'nearest_second_body')
        },
    }


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-epoch', required=True)
    parser.add_argument('--baseline-scene', required=True)
    parser.add_argument('--matrix', required=True)
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--authority-root', required=True)
    parser.add_argument('--clearance-auditor', required=True)
    parser.add_argument('--worker-timeout-s', type=float, default=360.0)
    parser.add_argument('--clearance-timeout-s', type=float, default=300.0)
    return parser.parse_args(argv)


def main(argv=None):
    """Derive every case from unmodified CDRs and invoke the file worker."""
    from h1_2_perception_msgs.msg import SceneEpoch
    from moveit_msgs.msg import PlanningScene
    from rclpy.serialization import deserialize_message, serialize_message

    args = parse_arguments(argv)
    epoch_path = Path(args.baseline_epoch).resolve(strict=True)
    scene_path = Path(args.baseline_scene).resolve(strict=True)
    matrix_path = Path(args.matrix).resolve(strict=True)
    output = Path(args.output_directory).resolve()
    workspace = Path(args.workspace).resolve(strict=True)
    authority_root = Path(args.authority_root).resolve(strict=True)
    auditor = Path(args.clearance_auditor).resolve(strict=True)
    matrix = json.loads(matrix_path.read_text(encoding='utf-8'))
    if matrix.get('schema') != SCHEMA:
        raise ValueError('MATRIX_SCHEMA')
    hashes = matrix.get('baseline_cdr_sha256') or {}
    if file_sha256(epoch_path) != hashes.get('epoch'):
        raise ValueError('BASELINE_EPOCH_SHA256')
    if file_sha256(scene_path) != hashes.get('scene'):
        raise ValueError('BASELINE_SCENE_SHA256')
    cases = matrix.get('cases')
    if not isinstance(cases, list) or not cases:
        raise ValueError('MATRIX_CASES')
    names = [entry.get('case') for entry in cases]
    if any(not isinstance(value, str) or not value for value in names):
        raise ValueError('MATRIX_CASE_NAME')
    if len(names) != len(set(names)):
        raise ValueError('MATRIX_CASE_DUPLICATE')

    output.mkdir(parents=True, exist_ok=False)
    baseline_epoch = deserialize_message(epoch_path.read_bytes(), SceneEpoch)
    baseline_scene = deserialize_message(
        scene_path.read_bytes(), PlanningScene)
    results = []
    for index, entry in enumerate(cases, start=1):
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
            sys.executable, '-m', WORKER_MODULE,
            '--epoch', str(derived_epoch),
            '--scene', str(derived_scene),
            '--workspace', str(workspace),
            '--authority-root', str(authority_root),
            '--clearance-auditor', str(auditor),
            '--clearance-timeout-s', str(args.clearance_timeout_s),
            '--output', str(case_directory / 'result.json'),
        ]
        with (case_directory / 'worker.stdout.log').open('wb') as stdout, (
                case_directory / 'worker.stderr.log').open('wb') as stderr:
            try:
                completed = subprocess.run(
                    command, stdin=subprocess.DEVNULL, stdout=stdout,
                    stderr=stderr, check=False, timeout=args.worker_timeout_s)
            except subprocess.TimeoutExpired:
                completed = subprocess.CompletedProcess(command, 124)
        record = compact_result(
            entry, epoch, requested_base, frame_delta,
            case_directory, completed)
        results.append(record)
        summary = {
            'schema': SCHEMA,
            'matrix_path': str(matrix_path),
            'matrix_sha256': file_sha256(matrix_path),
            'baseline_epoch_path': str(epoch_path),
            'baseline_scene_path': str(scene_path),
            'baseline_epoch_cdr_sha256': file_sha256(epoch_path),
            'baseline_scene_cdr_sha256': file_sha256(scene_path),
            'baseline_cdr_modified': False,
            'fixed_authority_modified': False,
            'robot_motion_allowed': False,
            'robot_trajectory_commands_sent': 0,
            'navigation_commands_sent': 0,
            'gazebo_started': False,
            'case_count': len(cases),
            'executed_count': len(results),
            'full_audit_pass_count': sum(
                value['full_audit_passed'] for value in results),
            'expected_rejection_count': sum(
                value['expected'] == 'REJECT' for value in results),
            'expectation_match_count': sum(
                value['expectation_matched'] for value in results),
            'complete': len(results) == len(cases),
            'all_expectations_matched': bool(
                len(results) == len(cases) and
                all(value['expectation_matched'] for value in results)),
            'cases': results,
        }
        (output / 'summary.json').write_text(
            json.dumps(summary, sort_keys=True, indent=2,
                       allow_nan=False) + '\n', encoding='utf-8')
        print(json.dumps({
            'case': entry['case'],
            'index': index,
            'count': len(cases),
            'status': record['status'],
            'full_audit_passed': record['full_audit_passed'],
            'expectation_matched': record['expectation_matched'],
        }, sort_keys=True), flush=True)
    return 0 if summary['all_expectations_matched'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
