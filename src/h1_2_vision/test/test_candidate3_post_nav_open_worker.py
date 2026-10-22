"""Pure/source-binding tests for the v9-derived open-only worker."""

import math
from pathlib import Path

from h1_2_vision.candidate3_contract import yaw_transform
from h1_2_vision.candidate3_post_nav_contract import (
    validate_open_base_envelope,
)
from h1_2_vision.candidate3_post_nav_open_worker import (
    AUDIT_JOINTS,
    build_stage,
    densify_states,
    IK_JOINTS,
    load_v9_sources,
    reverse_path_audit,
)
import numpy as np
import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
AUTHORITY = WORKSPACE / 'runtime_assets/candidate3_roundtrip_831_authority_v9'
SOURCE = (
    WORKSPACE / 'src/h1_2_vision/h1_2_vision/'
    'candidate3_post_nav_open_worker.py').read_text(encoding='utf-8')


def test_v9_sources_are_pinned_and_right_hand_seam_is_exact():
    module, bundle, prefix, door_phase, derived_ingress = load_v9_sources(
        WORKSPACE, AUTHORITY)
    assert bundle.path.sha256 == module.EXPECTED_PATH_SHA256
    assert len(prefix[-1].states) == 921
    assert len(door_phase['commands'][0]['points']) == 601
    assert len(derived_ingress) == 5
    q90 = prefix[-1].states[-1]
    assert door_phase['commands'][0]['points'][0]['positions'][0] == (
        q90['torso_joint'])


def test_densification_and_projection_preserve_synchronized_states():
    first = {name: 0.0 for name in AUDIT_JOINTS}
    second = dict(first)
    second['torso_joint'] = math.radians(1.2)
    second['right_elbow_joint'] = math.radians(-0.7)
    dense = densify_states([first, second], IK_JOINTS)
    values = [value for value, _ in dense]
    matrix = np.asarray([[value[name] for name in IK_JOINTS]
                         for value in values])
    assert float(np.max(np.abs(np.diff(matrix, axis=0)))) <= (
        math.radians(0.49) + 1.0e-12)
    stage = build_stage('test', values, 0.01)
    audit = stage['messages'][0]
    right = stage['controller_projection'][
        '/right_arm_controller/joint_trajectory']
    indices = [AUDIT_JOINTS.index(name)
               for name in right['joint_names']]
    for audit_point, right_point in zip(audit['points'], right['points']):
        assert right_point['positions'] == [
            audit_point['positions'][index] for index in indices]
        assert right_point['time_from_start'] == (
            audit_point['time_from_start'])
    assert stage['timing']['maximum_velocity_ratio'] <= 1.0 + 1.0e-9
    assert stage['timing']['maximum_acceleration_ratio'] <= 1.0 + 1.0e-9


def test_reverse_audit_requires_exact_audit_only_mirrors():
    first = {name: 0.0 for name in AUDIT_JOINTS}
    second = dict(first)
    second['right_elbow_joint'] = 0.01
    forward = [
        build_stage(f'forward_{index}', [first, second], 0.01)
        for index in range(3)
    ]
    reverse = [
        build_stage(
            f'reverse_{index}', [second, first], 0.01,
            audit_only=True)
        for index in range(3)
    ]
    stages = forward + reverse
    assert reverse_path_audit(stages)
    stages[3]['messages'][0]['points'][0]['positions'][0] += 0.001
    assert not reverse_path_audit(stages)


@pytest.mark.parametrize('base', [
    yaw_transform([0.001, 0.0, 0.0], 0.0),
    yaw_transform([-0.001, 0.0, 0.0], 0.0),
    yaw_transform([0.0, 0.001, 0.0], 0.0),
    yaw_transform([0.0, -0.001, 0.0], 0.0),
    yaw_transform([0.0, 0.0, 0.0], math.radians(0.1)),
    yaw_transform([0.0, 0.0, 0.0], math.radians(-0.1)),
])
def test_open_envelope_boundary_is_symmetric(base):
    validate_open_base_envelope(base)


@pytest.mark.parametrize(('base', 'status'), [
    (yaw_transform([0.001001, 0.0, 0.0], 0.0),
     'OPEN_BASE_X_ENVELOPE'),
    (yaw_transform([0.0, -0.001001, 0.0], 0.0),
     'OPEN_BASE_Y_ENVELOPE'),
    (yaw_transform([0.0, 0.0, 0.001001], 0.0),
     'OPEN_BASE_Z_ENVELOPE'),
    (yaw_transform([0.0, 0.0, 0.0], math.radians(0.1001)),
     'OPEN_BASE_YAW_ENVELOPE'),
])
def test_open_envelope_excess_fails_closed(base, status):
    with pytest.raises(ValueError, match=status):
        validate_open_base_envelope(base)


def test_worker_has_no_ros_control_or_navigation_surface():
    assert 'create_publisher' not in SOURCE
    assert 'ActionClient' not in SOURCE
    assert 'NavigateToPose' not in SOURCE
    assert 'FollowJointTrajectory' not in SOURCE
    assert '/joint_trajectory' in SOURCE
    assert 'controller_projection' in SOURCE
    assert 'subprocess.run' in SOURCE
    assert "'--scene'" in SOURCE
