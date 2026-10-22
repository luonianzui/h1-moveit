import math

from h1_2_vision.candidate3_matrix import case_offsets, target_poses
from h1_2_vision.candidate3_worker import REFERENCE_CABINET, REFERENCE_CUBE
import numpy as np


def test_stage_i_scalar_and_combined_offsets_use_si_units():
    scalar = case_offsets({
        'parameter': 'cabinet_x_mm',
        'value': -30,
    })
    assert scalar['cabinet_x_m'] == -0.030
    assert all(value == 0.0 for key, value in scalar.items()
               if key != 'cabinet_x_m')

    combined = case_offsets({
        'parameter': 'cabinet_xy_yaw_cube_xy_yaw',
        'value': {
            'cabinet_x_mm': 30,
            'cabinet_y_mm': -30,
            'cabinet_yaw_deg': 5,
            'cube_x_mm': -60,
            'cube_y_mm': 60,
            'cube_yaw_deg': -20,
        },
    })
    assert combined['cabinet_x_m'] == 0.030
    assert combined['cabinet_y_m'] == -0.030
    assert math.isclose(combined['cabinet_yaw_rad'], math.radians(5))
    assert combined['cube_x_m'] == -0.060
    assert combined['cube_y_m'] == 0.060
    assert math.isclose(combined['cube_yaw_rad'], math.radians(-20))


def test_zero_perturbation_targets_are_exact_reference_poses():
    offsets = case_offsets({
        'parameter': 'cube_xy_mm',
        'value': [0, 0],
    })
    cabinet, cube = target_poses(offsets)
    assert np.allclose(cabinet, REFERENCE_CABINET, atol=0, rtol=0)
    assert np.allclose(cube, REFERENCE_CUBE, atol=1e-15, rtol=0)


def test_boundary_targets_preserve_reference_height():
    offsets = case_offsets({
        'parameter': 'cabinet_xy_yaw_cube_xy_yaw',
        'value': {
            'cabinet_x_mm': -30,
            'cabinet_y_mm': 30,
            'cabinet_yaw_deg': -5,
            'cube_x_mm': 60,
            'cube_y_mm': -60,
            'cube_yaw_deg': 20,
        },
    })
    cabinet, cube = target_poses(offsets)
    assert np.allclose(
        cabinet[:3, 3], REFERENCE_CABINET[:3, 3] + [-0.03, 0.03, 0])
    assert np.allclose(
        cube[:3, 3], REFERENCE_CUBE[:3, 3] + [0.06, -0.06, 0])
    assert math.isclose(
        math.atan2(cabinet[1, 0], cabinet[0, 0]), math.radians(-5))
    assert math.isclose(
        math.atan2(cube[1, 0], cube[0, 0]),
        math.pi / 2 + math.radians(20))
