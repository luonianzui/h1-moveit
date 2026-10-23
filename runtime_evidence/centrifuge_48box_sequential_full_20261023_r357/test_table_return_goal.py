import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from table_return_goal import nearest_table_yaw, table_orientation_error


@pytest.mark.parametrize('yaw,expected', [(0, 0), (17, 0), (89, 0), (-89, 0), (91, 180), (162, 180), (-162, 180), (180, 180)])
def test_aligned_goal_uses_nearest_rectangular_axis_not_after_release(yaw, expected):
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
    assert nearest_table_yaw(body) == math.radians(expected)


@pytest.mark.parametrize('yaw', [0.0, math.pi])
@pytest.mark.parametrize('tilt', [0, .5, 1.99, 2.01, 5, 90, 180])
def test_original_two_degree_target_error_rejects_tilt_and_inversion(yaw, tilt):
    body = np.eye(4)
    body[:3, :3] = (Rotation.from_euler('z', yaw) * Rotation.from_euler('x', tilt, degrees=True)).as_matrix()
    assert (table_orientation_error(body, yaw) < math.radians(2)) == (tilt < 2)


@pytest.mark.parametrize('yaw_error', [1.99, 2.01, 7, 90])
def test_yaw_error_is_not_unrestricted(yaw_error):
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('z', math.pi + math.radians(yaw_error)).as_matrix()
    assert (table_orientation_error(body, math.pi) < math.radians(2)) == (yaw_error < 2)


@pytest.mark.parametrize('yaw', [1.0, -math.pi, float('nan'), float('inf')])
def test_invalid_goals_cannot_hide_a_failed_delivery(yaw):
    with pytest.raises(ValueError):
        table_orientation_error(np.eye(4), yaw)

