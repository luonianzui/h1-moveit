"""Pure geometry helpers for Mid-360 square-wall localization."""

import math

import numpy as np


def normalize_angle(angle):
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_rotation_matrix(x, y, z, w):
    """Return a finite 3-D rotation matrix for a quaternion."""
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise ValueError('quaternion must have a finite non-zero norm')
    x, y, z, w = quaternion / norm
    return np.asarray([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ], dtype=np.float64)


def transform_points(points, translation, quaternion):
    """Transform row-vector points into a target frame."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('translation must contain three finite values')
    rotation = quaternion_rotation_matrix(*quaternion)
    return points @ rotation.T + translation


def filter_navigation_points(
        points, minimum_height, maximum_height, minimum_range,
        maximum_range):
    """Select finite points inside the navigation height and range gates."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if minimum_height >= maximum_height:
        raise ValueError('minimum_height must be below maximum_height')
    if minimum_range < 0.0 or minimum_range >= maximum_range:
        raise ValueError('range limits are invalid')
    finite = np.isfinite(points).all(axis=1)
    planar_range = np.linalg.norm(points[:, :2], axis=1)
    selected = (
        finite
        & (points[:, 2] >= minimum_height)
        & (points[:, 2] <= maximum_height)
        & (planar_range >= minimum_range)
        & (planar_range <= maximum_range)
    )
    return points[selected]


def points_to_laser_scan(
        points, angle_minimum, angle_maximum, bin_count, minimum_range,
        maximum_range):
    """Project filtered 3-D points into a virtual horizontal laser scan."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    if bin_count < 2 or angle_minimum >= angle_maximum:
        raise ValueError('scan angular limits are invalid')
    angle_increment = (angle_maximum - angle_minimum) / float(bin_count)
    ranges = np.full(bin_count, np.inf, dtype=np.float64)
    if not len(points):
        return ranges, angle_increment
    planar_range = np.linalg.norm(points[:, :2], axis=1)
    angles = np.arctan2(points[:, 1], points[:, 0])
    indices = np.floor(
        (angles - angle_minimum) / angle_increment).astype(np.int64)
    valid = (
        np.isfinite(planar_range)
        & (planar_range >= minimum_range)
        & (planar_range <= maximum_range)
        & (indices >= 0)
        & (indices < bin_count)
    )
    np.minimum.at(ranges, indices[valid], planar_range[valid])
    return ranges, angle_increment


def transform_planar_points(points, pose):
    """Transform planar base-frame points into the square map frame."""
    points = np.asarray(points, dtype=np.float64)
    pose = np.asarray(pose, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('planar points must have shape (N, 2)')
    if pose.shape != (3,) or not np.isfinite(pose).all():
        raise ValueError('pose must contain finite x, y, yaw values')
    cosine = math.cos(pose[2])
    sine = math.sin(pose[2])
    world_x = cosine * points[:, 0] - sine * points[:, 1] + pose[0]
    world_y = sine * points[:, 0] + cosine * points[:, 1] + pose[1]
    return np.column_stack((world_x, world_y))


def map_to_odom_pose(localized_pose, odom_pose):
    """Compose map-to-base with the inverse of odom-to-base."""
    localized_pose = np.asarray(localized_pose, dtype=np.float64)
    odom_pose = np.asarray(odom_pose, dtype=np.float64)
    if localized_pose.shape != (3,) or not np.isfinite(
            localized_pose).all():
        raise ValueError('localized_pose must contain finite x, y, yaw')
    if odom_pose.shape != (3,) or not np.isfinite(odom_pose).all():
        raise ValueError('odom_pose must contain finite x, y, yaw')
    yaw = normalize_angle(localized_pose[2] - odom_pose[2])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translated_odom_x = cosine * odom_pose[0] - sine * odom_pose[1]
    translated_odom_y = sine * odom_pose[0] + cosine * odom_pose[1]
    return np.asarray([
        localized_pose[0] - translated_odom_x,
        localized_pose[1] - translated_odom_y,
        yaw,
    ])


def square_wall_residuals(points, pose, wall_half_extent):
    """Return distances to the nearest inner face of a square enclosure."""
    if not math.isfinite(wall_half_extent) or wall_half_extent <= 0.0:
        raise ValueError('wall_half_extent must be positive')
    world_points = transform_planar_points(points, pose)
    boundary_coordinate = np.maximum(
        np.abs(world_points[:, 0]), np.abs(world_points[:, 1]))
    return np.abs(boundary_coordinate - wall_half_extent)


def select_square_wall_candidates(
        points, pose, wall_half_extent, maximum_residual):
    """Select points near the expected square boundary from a prior pose."""
    if not math.isfinite(maximum_residual) or maximum_residual <= 0.0:
        raise ValueError('maximum_residual must be positive')
    points = np.asarray(points, dtype=np.float64)
    residuals = square_wall_residuals(points, pose, wall_half_extent)
    return points[residuals <= maximum_residual]


def square_pose_cost(
        points, pose, wall_half_extent, trim_fraction=0.90,
        outlier_cap=0.30):
    """Return a robust RMS distance from points to the square boundary."""
    if not 0.5 <= trim_fraction <= 1.0:
        raise ValueError('trim_fraction must be between 0.5 and 1.0')
    if not math.isfinite(outlier_cap) or outlier_cap <= 0.0:
        raise ValueError('outlier_cap must be positive')
    residuals = square_wall_residuals(
        points, pose, wall_half_extent)
    if not len(residuals):
        return math.inf
    residuals = np.minimum(residuals, outlier_cap)
    retained_count = max(1, int(math.ceil(len(residuals) * trim_fraction)))
    retained = np.partition(residuals, retained_count - 1)[:retained_count]
    return float(math.sqrt(np.mean(retained * retained)))


def refine_square_pose(
        points, initial_pose, wall_half_extent,
        translation_steps=(0.10, 0.02, 0.005),
        yaw_steps=(math.radians(2.0), math.radians(0.5),
                   math.radians(0.1)),
        maximum_translation_offset=0.40,
        maximum_yaw_offset=math.radians(15.0),
        maximum_iterations=12):
    """Refine a nearby pose with deterministic coarse-to-fine search."""
    points = np.asarray(points, dtype=np.float64)
    initial_pose = np.asarray(initial_pose, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        raise ValueError('points must contain at least one planar point')
    if initial_pose.shape != (3,) or not np.isfinite(initial_pose).all():
        raise ValueError('initial_pose must contain finite x, y, yaw')
    if len(translation_steps) != len(yaw_steps):
        raise ValueError('translation and yaw levels must match')
    current = initial_pose.copy()
    current[2] = normalize_angle(current[2])
    current_cost = square_pose_cost(points, current, wall_half_extent)
    for translation_step, yaw_step in zip(
            translation_steps, yaw_steps):
        for _ in range(maximum_iterations):
            best_pose = current
            best_cost = current_cost
            for x_direction in (-1, 0, 1):
                for y_direction in (-1, 0, 1):
                    for yaw_direction in (-1, 0, 1):
                        if not (x_direction or y_direction or yaw_direction):
                            continue
                        candidate = current + np.asarray([
                            x_direction * translation_step,
                            y_direction * translation_step,
                            yaw_direction * yaw_step,
                        ])
                        candidate[2] = normalize_angle(candidate[2])
                        translation_offset = np.linalg.norm(
                            candidate[:2] - initial_pose[:2])
                        yaw_offset = abs(normalize_angle(
                            candidate[2] - initial_pose[2]))
                        if translation_offset > maximum_translation_offset:
                            continue
                        if yaw_offset > maximum_yaw_offset:
                            continue
                        candidate_cost = square_pose_cost(
                            points, candidate, wall_half_extent)
                        if candidate_cost + 1.0e-12 < best_cost:
                            best_pose = candidate
                            best_cost = candidate_cost
            if best_cost + 1.0e-12 >= current_cost:
                break
            current = best_pose
            current_cost = best_cost
    return current, current_cost


def evaluate_square_pose(
        points, pose, wall_half_extent, inlier_threshold):
    """Return inlier fraction, inlier RMS and per-wall point counts."""
    if not math.isfinite(inlier_threshold) or inlier_threshold <= 0.0:
        raise ValueError('inlier_threshold must be positive')
    world_points = transform_planar_points(points, pose)
    residuals = square_wall_residuals(
        points, pose, wall_half_extent)
    inliers = residuals <= inlier_threshold
    inlier_count = int(np.count_nonzero(inliers))
    fraction = inlier_count / float(len(points)) if len(points) else 0.0
    rms = math.inf
    if inlier_count:
        rms = float(math.sqrt(np.mean(residuals[inliers] ** 2)))
    side_counts = [0, 0, 0, 0]
    if inlier_count:
        selected = world_points[inliers]
        x_side = np.abs(selected[:, 0]) >= np.abs(selected[:, 1])
        side_counts[0] = int(np.count_nonzero(
            x_side & (selected[:, 0] >= 0.0)))
        side_counts[1] = int(np.count_nonzero(
            x_side & (selected[:, 0] < 0.0)))
        side_counts[2] = int(np.count_nonzero(
            ~x_side & (selected[:, 1] >= 0.0)))
        side_counts[3] = int(np.count_nonzero(
            ~x_side & (selected[:, 1] < 0.0)))
    return fraction, rms, tuple(side_counts)


def square_quality_passes(
        inlier_fraction, inlier_rms, side_counts,
        minimum_inlier_fraction, maximum_inlier_rms,
        minimum_points_per_wall, minimum_visible_walls):
    """Evaluate localization quality while allowing sensor self-occlusion."""
    if len(side_counts) != 4:
        raise ValueError('side_counts must contain four wall counts')
    if not 1 <= minimum_visible_walls <= 4:
        raise ValueError('minimum_visible_walls must be between one and four')
    visible_wall_count = sum(
        int(count) >= minimum_points_per_wall for count in side_counts)
    valid = (
        math.isfinite(inlier_rms)
        and inlier_fraction >= minimum_inlier_fraction
        and inlier_rms <= maximum_inlier_rms
        and visible_wall_count >= minimum_visible_walls
    )
    return bool(valid), int(visible_wall_count)


def build_square_occupancy(
        wall_half_extent, wall_thickness, outside_margin, resolution,
        occupied_rectangles=()):
    """Build a free interior, occupied wall ring and unknown exterior map."""
    values = (
        wall_half_extent, wall_thickness, outside_margin, resolution)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('map dimensions and resolution must be positive')
    map_half_extent = wall_half_extent + wall_thickness + outside_margin
    cell_count = int(math.ceil(2.0 * map_half_extent / resolution))
    map_half_extent = 0.5 * cell_count * resolution
    origin = -map_half_extent
    coordinates = origin + (
        np.arange(cell_count, dtype=np.float64) + 0.5) * resolution
    x_grid, y_grid = np.meshgrid(coordinates, coordinates)
    boundary_coordinate = np.maximum(np.abs(x_grid), np.abs(y_grid))
    occupancy = np.full((cell_count, cell_count), -1, dtype=np.int8)
    occupancy[boundary_coordinate < wall_half_extent] = 0
    wall = (
        (boundary_coordinate >= wall_half_extent)
        & (boundary_coordinate <= wall_half_extent + wall_thickness)
    )
    occupancy[wall] = 100
    for rectangle in occupied_rectangles:
        if len(rectangle) != 4:
            raise ValueError('occupied rectangles require four bounds')
        minimum_x, maximum_x, minimum_y, maximum_y = (
            float(value) for value in rectangle)
        if (
                not all(math.isfinite(value) for value in (
                    minimum_x, maximum_x, minimum_y, maximum_y))
                or minimum_x >= maximum_x
                or minimum_y >= maximum_y
                or minimum_x <= -wall_half_extent
                or maximum_x >= wall_half_extent
                or minimum_y <= -wall_half_extent
                or maximum_y >= wall_half_extent):
            raise ValueError(
                'occupied rectangle must be finite and inside the room')
        occupied = (
            (x_grid >= minimum_x)
            & (x_grid <= maximum_x)
            & (y_grid >= minimum_y)
            & (y_grid <= maximum_y)
        )
        occupancy[occupied] = 100
    return occupancy, origin
