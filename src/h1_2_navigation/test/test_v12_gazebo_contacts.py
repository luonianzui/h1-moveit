from types import SimpleNamespace

from h1_2_navigation.v12_gazebo_controller import _contact_force
from h1_2_navigation.v12_gazebo_controller import _rate_limit_targets
from h1_2_navigation.v12_gazebo_controller import _wrench_force
import numpy as np


def _force(x_value, y_value, z_value):
    return SimpleNamespace(x=x_value, y=y_value, z=z_value)


def test_contact_force_uses_largest_body_wrench_magnitude():
    contact = SimpleNamespace(wrenches=[SimpleNamespace(
        body_1_wrench=SimpleNamespace(force=_force(3.0, 4.0, 0.0)),
        body_2_wrench=SimpleNamespace(force=_force(0.0, 0.0, -12.0)),
    )])

    assert _contact_force(SimpleNamespace(contacts=[contact])) == 12.0


def test_contact_force_is_zero_without_collision_contacts():
    assert _contact_force(SimpleNamespace(contacts=[])) == 0.0


def test_wrench_force_uses_continuous_ankle_load_magnitude():
    message = SimpleNamespace(wrench=SimpleNamespace(
        force=_force(3.0, 4.0, 12.0)))

    assert _wrench_force(message) == 13.0


def test_position_targets_are_rate_limited_for_gazebo_servo():
    actual = _rate_limit_targets(
        np.asarray([0.0, 0.1, -0.1]),
        np.asarray([0.2, 0.12, -0.3]),
        0.04,
    )

    np.testing.assert_allclose(actual, [0.04, 0.12, -0.14])
