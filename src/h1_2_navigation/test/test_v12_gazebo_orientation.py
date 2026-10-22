import math

from h1_2_navigation.v12_gazebo_controller import _imu_up_world
from h1_2_navigation.v12_gazebo_controller import _quaternion_multiply
from h1_2_navigation.v12_gazebo_controller import _yaw_from_quaternion
import numpy as np


def test_imu_up_vector_is_local_z_expressed_in_world_coordinates():
    half_angle = math.sqrt(0.5)

    up_world = _imu_up_world((half_angle, 0.0, 0.0, half_angle))

    np.testing.assert_allclose(
        up_world, (0.0, -1.0, 0.0), rtol=0.0, atol=1.0e-12)


def test_imu_up_vector_normalizes_input_quaternion():
    up_world = _imu_up_world((0.0, 0.0, 0.0, 2.0))

    np.testing.assert_allclose(
        up_world, (0.0, 0.0, 1.0), rtol=0.0, atol=1.0e-12)


def test_planar_yaw_and_relative_tilt_reconstruct_body_orientation():
    roll = math.radians(7.0)
    pitch = math.radians(-4.0)
    yaw = math.radians(31.0)
    cosine_roll, sine_roll = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cosine_pitch = math.cos(pitch / 2.0)
    sine_pitch = math.sin(pitch / 2.0)
    cosine_yaw, sine_yaw = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    body = _quaternion_multiply(
        (0.0, 0.0, sine_yaw, cosine_yaw),
        _quaternion_multiply(
            (0.0, sine_pitch, 0.0, cosine_pitch),
            (sine_roll, 0.0, 0.0, cosine_roll),
        ),
    )

    planar_yaw = _yaw_from_quaternion(body)
    planar = (0.0, 0.0, math.sin(planar_yaw / 2.0),
              math.cos(planar_yaw / 2.0))
    relative = _quaternion_multiply(
        (0.0, 0.0, -planar[2], planar[3]), body)

    np.testing.assert_allclose(
        _quaternion_multiply(planar, relative), body,
        rtol=0.0, atol=1.0e-12)
