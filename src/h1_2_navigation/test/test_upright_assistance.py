import math

from h1_2_navigation.upright_assistance import compute_upright_assistance
from h1_2_navigation.upright_assistance import UprightAssistanceConfig
import numpy as np
import pytest


def _config(**overrides):
    values = {
        'nominal_height': 1.0,
        'mass': 60.0,
        'gravity': 10.0,
        'vertical_stiffness': 1000.0,
        'vertical_damping': 100.0,
        'maximum_vertical_force_fraction': 0.25,
        'planar_damping': 10.0,
        'maximum_planar_damping_force': 4.0,
        'attitude_stiffness': 200.0,
        'attitude_damping': 20.0,
        'maximum_attitude_torque': 50.0,
        'minimum_up_z': 0.60,
    }
    values.update(overrides)
    return UprightAssistanceConfig(**values)


def test_planar_assistance_is_strictly_dissipative_and_yaw_torque_is_zero():
    half_angle = math.sin(math.radians(10.0) / 2.0)
    cosine = math.cos(math.radians(10.0) / 2.0)

    result = compute_upright_assistance(
        0.95,
        (half_angle, 0.0, 0.0, cosine),
        (0.4, -0.3, -0.1),
        (0.2, -0.1, 0.7),
        _config(),
    )

    assert result.active
    assert result.torque_world[2] == 0.0
    assert result.force_world[2] > 0.0
    assert np.linalg.norm(result.torque_world[:2]) > 0.0
    planar_velocity = result.linear_velocity_world[:2]
    assert np.dot(result.force_world[:2], planar_velocity) <= 0.0
    assert np.linalg.norm(result.force_world[:2]) == pytest.approx(4.0)


def test_vertical_assistance_is_one_sided_and_fraction_limited():
    config = _config()

    high = compute_upright_assistance(
        1.1, (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.1), (0.0, 0.0, 0.0), config)
    low = compute_upright_assistance(
        0.0, (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, -10.0), (0.0, 0.0, 0.0), config)

    assert high.force_world[2] == 0.0
    assert low.force_world[2] == pytest.approx(150.0)


def test_attitude_torque_points_toward_upright_and_is_limited():
    angle = math.radians(30.0)
    result = compute_upright_assistance(
        1.0,
        (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0)),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        _config(),
    )

    assert result.torque_world[1] < 0.0
    assert np.linalg.norm(result.torque_world[:2]) == pytest.approx(50.0)


def test_assistance_disables_after_fall_gate():
    angle = math.radians(60.0)
    result = compute_upright_assistance(
        0.5,
        (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0)),
        (0.0, 0.0, -1.0),
        (1.0, 1.0, 0.0),
        _config(),
    )

    assert not result.active
    np.testing.assert_array_equal(result.force_world, np.zeros(3))
    np.testing.assert_array_equal(result.torque_world, np.zeros(3))


@pytest.mark.parametrize('quaternion', [
    (0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, math.nan, 1.0),
])
def test_invalid_orientation_is_rejected(quaternion):
    with pytest.raises(ValueError, match='quaternion'):
        compute_upright_assistance(
            1.0, quaternion, (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0), _config())
