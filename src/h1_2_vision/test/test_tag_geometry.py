import math

import cv2
from h1_2_vision.tag_geometry import (
    matrix_quaternion,
    quaternion_matrix,
    rotation_distance,
    rpy_matrix,
    solve_square_tag,
    transform,
    weighted_transform_mean,
)
import numpy as np


def test_quaternion_round_trip_and_rotation_distance():
    rotation = rpy_matrix(0.4, -0.2, 1.1)
    quaternion = matrix_quaternion(rotation)
    recovered = quaternion_matrix(*quaternion)
    assert rotation_distance(transform(rotation, [0, 0, 0]),
                             transform(recovered, [0, 0, 0])) < 1e-12


def test_weighted_transform_mean_handles_quaternion_sign():
    first = transform(rpy_matrix(0, 0, 0.1), [1.0, 2.0, 3.0])
    second = transform(rpy_matrix(0, 0, 0.1), [1.2, 2.0, 3.0])
    mean = weighted_transform_mean([first, second], [3.0, 1.0])
    assert np.allclose(mean[:3, 3], [1.05, 2.0, 3.0])
    assert rotation_distance(first, mean) < 1e-12


def test_ippe_square_recovers_synthetic_pose():
    camera = np.array([[462.2, 0, 320], [0, 462.2, 240], [0, 0, 1.0]])
    distortion = np.zeros(5)
    size = 0.08
    half = 0.5 * size
    points = np.array([
        [-half, half, 0], [half, half, 0],
        [half, -half, 0], [-half, -half, 0],
    ], dtype=np.float64)
    expected_rotation = rpy_matrix(math.pi, 0.10, -0.15)
    rvec, _ = cv2.Rodrigues(expected_rotation)
    expected_translation = np.array([0.03, -0.02, 0.75])
    pixels, _ = cv2.projectPoints(
        points, rvec, expected_translation, camera, distortion)
    solved = solve_square_tag(pixels, size, camera, distortion)
    assert solved is not None
    pose, _, error = solved
    assert error < 1e-6
    assert np.linalg.norm(pose[:3, 3] - expected_translation) < 1e-6
    assert rotation_distance(
        pose, transform(expected_rotation, expected_translation)) < 1e-6
