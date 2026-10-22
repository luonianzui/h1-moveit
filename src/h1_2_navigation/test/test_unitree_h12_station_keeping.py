import math

import numpy as np

from h1_2_navigation.unitree_h12_gazebo_controller import (
    _advance_motion_reference)
from h1_2_navigation.unitree_h12_gazebo_controller import (
    _capture_station_anchor)
from h1_2_navigation.unitree_h12_gazebo_controller import (
    _leash_motion_reference)
from h1_2_navigation.unitree_h12_gazebo_controller import (
    _pose_tracking_command)
from h1_2_navigation.unitree_h12_gazebo_controller import (
    _station_keeping_command)


def test_station_keeping_rotates_world_error_into_body_frame():
    command, world_error, yaw_error = _station_keeping_command(
        anchor=(1.0, 0.0, math.pi / 2.0),
        position=(0.0, 0.0),
        yaw=math.pi / 2.0,
        position_gain=0.05,
        yaw_gain=1.0,
        limits=(0.1, 0.08, 0.05))

    np.testing.assert_allclose(world_error, [1.0, 0.0], atol=1.0e-12)
    np.testing.assert_allclose(command, [0.0, -0.05, 0.0], atol=1.0e-7)
    assert yaw_error == 0.0


def test_station_keeping_clips_policy_command_and_wraps_yaw():
    command, _, yaw_error = _station_keeping_command(
        anchor=(1.0, -1.0, -math.pi + 0.02),
        position=(0.0, 0.0),
        yaw=math.pi - 0.02,
        position_gain=2.0,
        yaw_gain=2.0,
        limits=(0.1, 0.08, 0.05))

    np.testing.assert_allclose(command, [-0.1, 0.08, 0.05], atol=1.0e-7)
    assert math.isclose(yaw_error, 0.04, abs_tol=1.0e-12)


def test_motion_reference_integrates_body_twist_in_reference_frame():
    reference = _advance_motion_reference(
        (1.0, 2.0, math.pi / 2.0),
        (0.10, 0.05, -0.20),
        0.02)

    np.testing.assert_allclose(
        reference,
        [0.999, 2.002, math.pi / 2.0 - 0.004],
        atol=1.0e-12)


def test_motion_reference_leash_bounds_position_and_yaw_error():
    reference = _leash_motion_reference(
        (1.0, 0.0, 1.0),
        (0.0, 0.0),
        0.0,
        maximum_position_error=0.10,
        maximum_yaw_error=0.20)

    np.testing.assert_allclose(reference, [0.10, 0.0, 0.20], atol=1.0e-12)


def test_pose_tracking_adds_feedback_to_policy_feedforward():
    command, world_error, yaw_error = _pose_tracking_command(
        reference=(0.10, -0.02, -0.03),
        position=(0.0, 0.0),
        yaw=0.0,
        feedforward=(0.04, 0.0, 0.01),
        position_gain=2.0,
        yaw_gain=1.0,
        limits=(0.10, 0.08, 0.05))

    np.testing.assert_allclose(world_error, [0.10, -0.02], atol=1.0e-12)
    np.testing.assert_allclose(command, [0.10, -0.04, -0.02], atol=1.0e-7)
    assert math.isclose(yaw_error, -0.03, abs_tol=1.0e-12)


def test_station_anchor_preserves_final_motion_reference():
    anchor = _capture_station_anchor(
        motion_reference=(-0.45, 0.01, -0.02),
        position=(-0.53, 0.03),
        yaw=0.04)

    np.testing.assert_allclose(anchor, [-0.45, 0.01, -0.02], atol=1.0e-12)


def test_station_anchor_uses_current_pose_without_motion_reference():
    anchor = _capture_station_anchor(
        motion_reference=None,
        position=(-0.60, 0.00),
        yaw=0.03)

    np.testing.assert_allclose(anchor, [-0.60, 0.00, 0.03], atol=1.0e-12)


def test_station_keeping_adds_body_velocity_damping():
    command, world_error, yaw_error = _station_keeping_command(
        anchor=(0.02, -0.01, 0.03),
        position=(0.0, 0.0),
        yaw=0.0,
        position_gain=1.0,
        yaw_gain=1.0,
        limits=(0.10, 0.08, 0.05),
        body_linear_velocity=(0.05, -0.02),
        yaw_rate=0.04,
        linear_velocity_gain=0.8,
        yaw_rate_gain=0.5)

    np.testing.assert_allclose(world_error, [0.02, -0.01], atol=1.0e-12)
    np.testing.assert_allclose(command, [-0.02, 0.006, 0.01], atol=1.0e-7)
    assert math.isclose(yaw_error, 0.03, abs_tol=1.0e-12)
