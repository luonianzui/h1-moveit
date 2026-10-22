import math

from h1_2_navigation.square_localization import build_square_occupancy
from h1_2_navigation.square_localization import evaluate_square_pose
from h1_2_navigation.square_localization import filter_navigation_points
from h1_2_navigation.square_localization import map_to_odom_pose
from h1_2_navigation.square_localization import normalize_angle
from h1_2_navigation.square_localization import points_to_laser_scan
from h1_2_navigation.square_localization import refine_square_pose
from h1_2_navigation.square_localization import select_square_wall_candidates
from h1_2_navigation.square_localization import square_quality_passes
from h1_2_navigation.square_localization import transform_planar_points
from h1_2_navigation.square_localization import transform_points
import numpy as np
import pytest


def _synthetic_square_points(pose, seed=7):
    coordinates = np.linspace(-2.45, 2.45, 160)
    walls = np.vstack((
        np.column_stack((np.full_like(coordinates, 2.5), coordinates)),
        np.column_stack((np.full_like(coordinates, -2.5), coordinates)),
        np.column_stack((coordinates, np.full_like(coordinates, 2.5))),
        np.column_stack((coordinates, np.full_like(coordinates, -2.5))),
    ))
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    base_points = (walls - np.asarray(pose[:2])) @ rotation
    random = np.random.default_rng(seed)
    base_points += random.normal(0.0, 0.004, base_points.shape)
    outliers = random.uniform(-1.0, 1.0, size=(40, 2))
    return np.vstack((base_points, outliers))


def test_refine_square_pose_recovers_nearby_pose():
    expected = np.asarray([0.23, -0.17, math.radians(6.0)])
    points = _synthetic_square_points(expected)
    initial = np.asarray([0.12, -0.08, math.radians(3.0)])
    pose, cost = refine_square_pose(points, initial, 2.5)
    assert np.linalg.norm(pose[:2] - expected[:2]) <= 0.012
    assert abs(pose[2] - expected[2]) <= math.radians(0.25)
    assert cost <= 0.02
    fraction, rms, sides = evaluate_square_pose(
        points, pose, 2.5, 0.08)
    assert fraction >= 0.93
    assert rms <= 0.015
    assert min(sides) >= 100


def test_wall_candidate_prefilter_rejects_interior_workcell_clutter():
    expected = np.asarray([-0.52, 0.03, math.radians(0.5)])
    wall_points = _synthetic_square_points(expected)
    random = np.random.default_rng(19)
    clutter = random.uniform(
        low=(-0.1, -0.7), high=(1.8, 0.7), size=(1600, 2))
    points = np.vstack((wall_points, clutter))
    initial = np.asarray([-0.60, 0.0, 0.0])

    candidates = select_square_wall_candidates(
        points, initial, 2.5, 0.50)
    pose, _ = refine_square_pose(candidates, initial, 2.5)
    fraction, rms, sides = evaluate_square_pose(
        candidates, pose, 2.5, 0.08)

    assert len(candidates) < 0.40 * len(points)
    assert np.linalg.norm(pose[:2] - expected[:2]) <= 0.015
    assert abs(normalize_angle(pose[2] - expected[2])) <= math.radians(0.3)
    assert fraction >= 0.90
    assert rms <= 0.02
    assert sum(count >= 20 for count in sides) >= 3


def test_filter_and_scan_projection_keep_nearest_return():
    points = np.asarray([
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [0.0, 1.5, 0.0],
        [0.0, 0.0, -1.0],
        [math.inf, 0.0, 0.0],
    ])
    filtered = filter_navigation_points(points, -0.2, 0.2, 0.5, 3.0)
    assert filtered.shape == (3, 3)
    ranges, increment = points_to_laser_scan(
        filtered, -math.pi, math.pi, 360, 0.5, 3.0)
    assert math.isclose(increment, math.radians(1.0))
    zero_index = int(math.floor(math.pi / increment))
    quarter_index = int(math.floor((1.5 * math.pi) / increment))
    assert math.isclose(ranges[zero_index], 1.0)
    assert math.isclose(ranges[quarter_index], 1.5)


def test_transform_points_applies_pitch_and_translation():
    half_angle = math.radians(15.0) / 2.0
    transformed = transform_points(
        np.asarray([[1.0, 0.0, 0.0]]),
        (0.2, -0.1, 0.4),
        (0.0, math.sin(half_angle), 0.0, math.cos(half_angle)),
    )
    assert np.allclose(
        transformed[0],
        [0.2 + math.cos(math.radians(15.0)), -0.1,
         0.4 - math.sin(math.radians(15.0))],
    )


def test_map_to_odom_pose_recomposes_localized_base_pose():
    localized_pose = np.asarray([4.0, 6.0, math.radians(80.0)])
    odom_pose = np.asarray([1.0, 2.0, math.radians(30.0)])

    map_odom = map_to_odom_pose(localized_pose, odom_pose)
    recomposed_position = transform_planar_points(
        odom_pose[None, :2], map_odom)[0]
    recomposed_yaw = normalize_angle(map_odom[2] + odom_pose[2])

    assert np.allclose(recomposed_position, localized_pose[:2])
    assert math.isclose(recomposed_yaw, localized_pose[2])


def test_square_occupancy_has_free_interior_and_occupied_ring():
    occupancy, origin = build_square_occupancy(2.5, 0.1, 0.5, 0.05)
    assert occupancy.shape == (124, 124)
    center = occupancy.shape[0] // 2
    assert occupancy[center, center] == 0
    wall_index = int(math.floor((2.525 - origin) / 0.05))
    assert occupancy[center, wall_index] == 100
    assert occupancy[0, 0] == -1


def test_square_occupancy_adds_known_incubator_workcell():
    occupancy, origin = build_square_occupancy(
        2.5,
        0.1,
        0.5,
        0.05,
        ((0.20, 1.00, -0.60, 0.60),),
    )

    def cell(x_value, y_value):
        column = int(math.floor((x_value - origin) / 0.05))
        row = int(math.floor((y_value - origin) / 0.05))
        return occupancy[row, column]

    assert cell(0.60, 0.0) == 100
    assert cell(-0.45, 0.0) == 0
    with pytest.raises(ValueError, match='inside the room'):
        build_square_occupancy(
            2.5, 0.1, 0.5, 0.05, ((2.40, 2.60, -0.2, 0.2),))


def test_quality_gate_accepts_three_visible_non_collinear_walls():
    valid, visible_count = square_quality_passes(
        0.993,
        0.018,
        (513, 0, 101, 101),
        0.80,
        0.05,
        20,
        3,
    )
    assert valid
    assert visible_count == 3
    four_required, _ = square_quality_passes(
        0.993,
        0.018,
        (513, 0, 101, 101),
        0.80,
        0.05,
        20,
        4,
    )
    assert not four_required
