"""
Fail-closed, pure-data admission contract for candidate-3 execution.

This module deliberately imports no ROS execution surface.  It validates the
complete worker result, including the hashes of the nested plan/certificate,
before an execution supervisor is allowed to translate any point into a ROS
trajectory message.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import stat

from .candidate3_contract import (
    AUDITOR_VERSION,
    canonical_hash,
    DEFAULT_CLEARANCE_M,
    DOOR_ARC_TARGET_COUNT,
    MAXIMUM_ADJACENT_CHANGE_RAD,
    PLANNER_VERSION,
    SCHEMA,
    stage_hash,
    STAGE_NAMES,
    TOPOLOGY,
)


CERTIFICATE_SCHEMA = 'h1_candidate3_plan_certificate/v1'
MAXIMUM_RESULT_BYTES = 32 * 1024 * 1024
MAXIMUM_ACTION_DURATION_S = 90.0

CONTROLLER_JOINTS = {
    '/left_arm_controller/joint_trajectory': (
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint', 'left_elbow_joint',
        'left_wrist_roll_joint', 'left_wrist_pitch_joint',
        'left_wrist_yaw_joint',
    ),
    '/right_arm_controller/joint_trajectory': (
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint', 'right_elbow_joint',
        'right_wrist_roll_joint', 'right_wrist_pitch_joint',
        'right_wrist_yaw_joint',
    ),
    '/left_gripper_controller/joint_trajectory': (
        'left_dex1_finger_1_joint', 'left_dex1_finger_2_joint',
    ),
    '/right_gripper_controller/joint_trajectory': (
        'right_dex1_finger_1_joint', 'right_dex1_finger_2_joint',
    ),
}

STAGE_TOPICS = (
    ('/left_arm_controller/joint_trajectory',
     '/left_gripper_controller/joint_trajectory',
     '/left_arm_controller/joint_trajectory'),
    ('/left_gripper_controller/joint_trajectory',
     '/left_arm_controller/joint_trajectory'),
    ('/right_gripper_controller/joint_trajectory',
     '/right_arm_controller/joint_trajectory',
     '/right_gripper_controller/joint_trajectory',
     '/right_arm_controller/joint_trajectory'),
    ('/left_arm_controller/joint_trajectory',
     '/left_gripper_controller/joint_trajectory',
     '/left_arm_controller/joint_trajectory',
     '/left_gripper_controller/joint_trajectory',
     '/left_arm_controller/joint_trajectory'),
    ('/right_arm_controller/joint_trajectory',
     '/right_gripper_controller/joint_trajectory',
     '/right_arm_controller/joint_trajectory',
     '/right_gripper_controller/joint_trajectory'),
    ('/left_arm_controller/joint_trajectory',
     '/left_gripper_controller/joint_trajectory'),
    ('/left_arm_controller/joint_trajectory',
     '/left_gripper_controller/joint_trajectory',
     '/left_arm_controller/joint_trajectory'),
)

SOURCE_ASSET_HASHES = {
    'handle_position_only.trajectory.json':
        'c03df7c51daac6d2059ab5316d17dd9d486191d1bc2aadd0395cfe73044adc6b',
    'left_clear.trajectory.json':
        '066b2437fa320d2db895a927e24f0d5044ff2a9786638e335f8117695e105b88',
    'door.trajectory.json':
        '867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19',
    'cube.parallel90.neutral.trajectory.json':
        'acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705',
    'door_close.trajectory.json':
        '5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038',
    'left_regrasp.trajectory.json':
        '5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d',
    'handle_relock.trajectory.json':
        'b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3',
}
SOURCE_CACHE_HASH = (
    'dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02')


class ExecutionContractError(ValueError):
    """The candidate-3 result is not admissible for execution."""


def _require(condition, status):
    if not condition:
        raise ExecutionContractError(status)


def duration_seconds(value):
    _require(isinstance(value, dict), 'POINT_DURATION_TYPE')
    _require(set(value) == {'sec', 'nanosec'}, 'POINT_DURATION_FIELDS')
    seconds = value['sec']
    nanoseconds = value['nanosec']
    _require(isinstance(seconds, int) and seconds >= 0, 'POINT_DURATION_SEC')
    _require(isinstance(nanoseconds, int) and 0 <= nanoseconds < 1_000_000_000,
             'POINT_DURATION_NANOSEC')
    return float(seconds) + float(nanoseconds) * 1e-9


def _finite_vector(value, width, status):
    _require(isinstance(value, list) and len(value) in (0, width), status)
    result = []
    for item in value:
        _require(isinstance(item, (int, float)) and math.isfinite(float(item)),
                 status)
        result.append(float(item))
    return result


def _validate_message(message, sequence, expected_topic):
    _require(isinstance(message, dict), 'MESSAGE_TYPE')
    _require(message.get('sequence') == sequence, 'MESSAGE_SEQUENCE')
    _require(message.get('topic') == expected_topic, 'MESSAGE_TOPIC')
    names = message.get('joint_names')
    _require(tuple(names or ()) == CONTROLLER_JOINTS[expected_topic],
             'MESSAGE_JOINTS')
    _require(isinstance(message.get('wall_offset_ns'), int) and
             message['wall_offset_ns'] >= 0, 'MESSAGE_WALL_OFFSET')
    points = message.get('points')
    _require(isinstance(points, list) and len(points) >= 2,
             'MESSAGE_POINTS')
    width = len(names)
    previous_time = -1.0
    for point in points:
        _require(isinstance(point, dict), 'POINT_TYPE')
        positions = _finite_vector(
            point.get('positions'), width, 'POINT_POSITIONS')
        _require(len(positions) == width, 'POINT_POSITIONS')
        _finite_vector(point.get('velocities'), width, 'POINT_VELOCITIES')
        _finite_vector(
            point.get('accelerations'), width, 'POINT_ACCELERATIONS')
        _finite_vector(point.get('effort'), width, 'POINT_EFFORT')
        current_time = duration_seconds(point.get('time_from_start'))
        _require(current_time >= previous_time, 'POINT_TIME_ORDER')
        previous_time = current_time
        if width == 2:
            _require(all(-0.0205 <= value <= 0.0255 for value in positions),
                     'GRIPPER_POSITION_LIMIT')
        else:
            _require(all(abs(value) < math.pi for value in positions),
                     'ARM_POSITION_LIMIT')
    _require(duration_seconds(points[0]['time_from_start']) == 0.0,
             'MESSAGE_NONZERO_START')
    _require(previous_time > 0.0, 'MESSAGE_ZERO_DURATION')
    return len(points)


def _positions(plan, stage_index, message_index, point_index=-1):
    return plan['stages'][stage_index]['messages'][message_index][
        'points'][point_index]['positions']


def _close(left, right, tolerance=1e-9):
    return (len(left) == len(right) and
            all(abs(float(a) - float(b)) <= tolerance
                for a, b in zip(left, right)))


def _validate_reverse(stages):
    for forward_index, reverse_index in ((0, 6), (1, 5), (2, 4)):
        for width in (2, 7):
            forward = [
                point['positions']
                for message in stages[forward_index]['messages']
                if len(message['joint_names']) == width
                for point in message['points']
            ]
            reverse = [
                point['positions']
                for message in stages[reverse_index]['messages']
                if len(message['joint_names']) == width
                for point in message['points']
            ]
            _require(len(forward) == len(reverse), 'REVERSE_POINT_COUNT')
            _require(all(_close(left, right, 1e-12) for left, right in
                         zip(forward, reversed(reverse))),
                     'REVERSE_TRAJECTORY')


def _validate_support_boundaries(plan):
    # These endpoints are the immutable physical hand-off checkpoints used by
    # the mature 8251/8253 Gazebo support helpers.  They bind the dynamically
    # retimed g5 plan to the already-audited attach/detach implementation.
    door_arm = [
        -0.3707748311326624, 0.19999817326302374,
        -2.137161466097166, 2.2367722841146924,
        -0.5394033215363917, -0.3952997106791191,
        0.9178323851958335,
    ]
    door_fingers = [0.011330528925639306, 0.012330528925639305]
    cube_contact_arm = [
        0.0556149341645926, 0.3092704510139955,
        0.06281950935406858, 0.9313910867123811,
        -0.2490038822279106, -0.382353015485734,
        -0.3157470756782848,
    ]
    cube_rack_arm = [
        -0.9444567973289557, -0.3164012808821606,
        -0.7213708495033594, 1.4567808664752016,
        -0.8690669730336473, -0.0901222244068371,
        0.5066048079955359,
    ]
    _require(_close(_positions(plan, 2, 1), door_arm),
             'DOOR_ATTACH_ARM_ENDPOINT')
    _require(_close(_positions(plan, 2, 2), door_fingers),
             'DOOR_ATTACH_GRIPPER_ENDPOINT')
    _require(_close(_positions(plan, 4, 0), door_arm),
             'DOOR_DETACH_ARM_ENDPOINT')
    _require(_close(_positions(plan, 4, 1, 0), door_fingers),
             'DOOR_DETACH_GRIPPER_ENDPOINT')
    _require(_close(_positions(plan, 3, 0), cube_contact_arm),
             'CUBE_ATTACH_ARM_ENDPOINT')
    _require(_close(_positions(plan, 3, 1), [-0.01472, -0.01472]),
             'CUBE_ATTACH_GRIPPER_ENDPOINT')
    _require(_close(_positions(plan, 3, 2), cube_rack_arm),
             'CUBE_DETACH_ARM_ENDPOINT')
    _require(_close(_positions(plan, 3, 3, 0), [-0.014942, -0.014942]),
             'CUBE_DETACH_GRIPPER_ENDPOINT')


def validate_result(result):
    """Validate and return the complete plan, or raise fail closed."""
    _require(isinstance(result, dict), 'RESULT_TYPE')
    plan = result.get('plan')
    certificate = result.get('certificate')
    _require(isinstance(plan, dict), 'MISSING_PLAN')
    _require(isinstance(certificate, dict), 'MISSING_CERTIFICATE')
    _require(result.get('plan_hash') == canonical_hash(plan), 'PLAN_HASH')
    _require(result.get('certificate_hash') == canonical_hash(certificate),
             'CERTIFICATE_HASH')

    _require(plan.get('schema') == SCHEMA, 'PLAN_SCHEMA')
    _require(plan.get('planner_version') == PLANNER_VERSION,
             'PLAN_VERSION')
    _require(plan.get('topology') == TOPOLOGY, 'PLAN_TOPOLOGY')
    _require(plan.get('candidate3_only') is True, 'PLAN_CANDIDATE3')
    _require(plan.get('contains_partial') is False, 'PLAN_PARTIAL')
    epoch_status = plan.get('source_epoch_status', '')
    alignment_hash = plan.get('post_nav_alignment_hash', '')
    if epoch_status.startswith('FROZEN_POST_NAV_ALIGNMENT:'):
        _require(
            epoch_status == f'FROZEN_POST_NAV_ALIGNMENT:{alignment_hash}' and
            isinstance(alignment_hash, str) and len(alignment_hash) == 64 and
            all(value in '0123456789abcdef' for value in alignment_hash),
            'PLAN_POST_NAV_ALIGNMENT')
    else:
        _require(not alignment_hash, 'PLAN_POST_NAV_ALIGNMENT_ORPHAN')
    _require(plan.get('door_arc_target_count') == DOOR_ARC_TARGET_COUNT,
             'PLAN_DOOR_TARGET_COUNT')
    _require(plan.get('source_asset_hashes') == SOURCE_ASSET_HASHES,
             'PLAN_SOURCE_ASSETS')
    _require(plan.get('source_cache_hash') == SOURCE_CACHE_HASH,
             'PLAN_SOURCE_CACHE')
    _require(plan.get('stage_names') == list(STAGE_NAMES),
             'PLAN_STAGE_NAMES')
    stages = plan.get('stages')
    _require(isinstance(stages, list) and len(stages) == len(STAGE_NAMES),
             'PLAN_STAGE_COUNT')
    _require(len(STAGE_TOPICS) == len(stages), 'INTERNAL_STAGE_TOPOLOGY')
    actual_hashes = []
    actual_counts = []
    for stage_index, (stage, topics) in enumerate(zip(stages, STAGE_TOPICS)):
        _require(stage.get('name') == STAGE_NAMES[stage_index], 'STAGE_NAME')
        messages = stage.get('messages')
        _require(isinstance(messages, list) and len(messages) == len(topics),
                 'STAGE_MESSAGE_COUNT')
        previous_offset = -1
        waypoint_count = 0
        for sequence, (message, topic) in enumerate(zip(messages, topics)):
            _require(message.get('wall_offset_ns', -1) >= previous_offset,
                     'MESSAGE_OFFSET_ORDER')
            previous_offset = message.get('wall_offset_ns', -1)
            waypoint_count += _validate_message(message, sequence, topic)
        duration = stage.get('duration_s')
        _require(isinstance(duration, (int, float)) and
                 math.isfinite(float(duration)) and float(duration) > 0.0,
                 'STAGE_DURATION')
        actual_hashes.append(stage_hash(stage))
        actual_counts.append(waypoint_count)
    _require(plan.get('stage_hashes') == actual_hashes, 'STAGE_HASHES')
    _require(plan.get('stage_waypoint_counts') == actual_counts,
             'STAGE_WAYPOINT_COUNTS')
    planned_duration = plan.get('planned_action_duration_s')
    _require(isinstance(planned_duration, (int, float)) and
             0.0 < float(planned_duration) <= MAXIMUM_ACTION_DURATION_S,
             'PLAN_ACTION_DURATION')
    _require(abs(sum(float(stage['duration_s']) for stage in stages) -
                 float(planned_duration)) <= 1e-6,
             'PLAN_DURATION_SUM')
    _validate_reverse(stages)
    _validate_support_boundaries(plan)

    _require(certificate.get('schema') == CERTIFICATE_SCHEMA,
             'CERTIFICATE_SCHEMA')
    _require(certificate.get('planner_version') == PLANNER_VERSION,
             'CERTIFICATE_PLANNER_VERSION')
    _require(certificate.get('auditor_version') == AUDITOR_VERSION,
             'CERTIFICATE_AUDITOR_VERSION')
    for field in ('scene_epoch_id', 'scene_hash'):
        _require(certificate.get(field) == plan.get(field),
                 f'CERTIFICATE_{field.upper()}')
    for field in ('source_epoch_status', 'post_nav_alignment_hash'):
        _require(certificate.get(field, '') == plan.get(field, ''),
                 f'CERTIFICATE_{field.upper()}')
    _require(certificate.get('plan_hash') == result.get('plan_hash'),
             'CERTIFICATE_PLAN_HASH')
    _require(certificate.get('door_arc_target_count') == DOOR_ARC_TARGET_COUNT,
             'CERTIFICATE_DOOR_TARGET_COUNT')
    for field in (
            'candidate3_only', 'ik_complete', 'collision_free',
            'original_stl_contact_passed', 'hinge_contract_passed',
            'forward_passed', 'reverse_passed', 'timing_passed',
            'support_topology_passed', 'no_partial', 'valid'):
        _require(certificate.get(field) is True,
                 f'CERTIFICATE_{field.upper()}')
    _require(certificate.get('status') == 'PASS', 'CERTIFICATE_STATUS')
    _require(float(certificate.get('minimum_environment_clearance_m', 0.0))
             >= DEFAULT_CLEARANCE_M, 'CERTIFICATE_CLEARANCE')
    _require(float(certificate.get('maximum_adjacent_joint_change_rad', math.inf))
             <= MAXIMUM_ADJACENT_CHANGE_RAD,
             'CERTIFICATE_CONTINUITY')
    _require(certificate.get('forward_dense_samples') == sum(actual_counts[:4]),
             'CERTIFICATE_FORWARD_SAMPLES')
    _require(certificate.get('reverse_dense_samples') == sum(actual_counts[4:]),
             'CERTIFICATE_REVERSE_SAMPLES')
    validity = certificate.get('validity_s')
    _require(isinstance(validity, (int, float)) and
             0.0 < float(validity) <= 2.0, 'CERTIFICATE_VALIDITY')
    return plan


def load_result_file(path, allowed_root):
    """Load one owner-local, non-symlink result below an explicit root."""
    path = Path(path)
    allowed_root = Path(allowed_root).resolve(strict=True)
    resolved = path.resolve(strict=True)
    _require(resolved == allowed_root or allowed_root in resolved.parents,
             'RESULT_PATH_SCOPE')
    metadata = path.lstat()
    _require(stat.S_ISREG(metadata.st_mode) and not stat.S_ISLNK(metadata.st_mode),
             'RESULT_PATH_TYPE')
    _require(metadata.st_uid == os.geteuid() and metadata.st_nlink == 1,
             'RESULT_PATH_OWNER')
    _require(0 < metadata.st_size <= MAXIMUM_RESULT_BYTES,
             'RESULT_PATH_SIZE')
    with path.open('r', encoding='utf-8') as stream:
        result = json.load(stream)
    validate_result(result)
    return result
