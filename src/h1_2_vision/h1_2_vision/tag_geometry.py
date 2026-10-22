"""Small, dependency-light SE(3) helpers for the H1-2 tag pipeline."""

from __future__ import annotations

import math

import cv2
import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def transform_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    return transform(rpy_matrix(*rpy), xyz)


def transform_from_ros(value) -> np.ndarray:
    translation = value.transform.translation
    rotation = value.transform.rotation
    return transform(
        quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        [translation.x, translation.y, translation.z],
    )


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-15:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            result = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            result = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            result = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    result /= np.linalg.norm(result)
    return result


def rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    relative = np.asarray(first)[:3, :3].T @ np.asarray(second)[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def weighted_transform_mean(transforms, weights) -> np.ndarray:
    if not transforms or len(transforms) != len(weights):
        raise ValueError('transforms and weights must be non-empty and aligned')
    normalized = np.asarray(weights, dtype=np.float64)
    normalized /= np.sum(normalized)
    translation = sum(
        weight * value[:3, 3] for value, weight in zip(transforms, normalized))
    quaternions = [matrix_quaternion(value[:3, :3]) for value in transforms]
    reference = quaternions[0]
    aligned = [q if np.dot(q, reference) >= 0.0 else -q for q in quaternions]
    accumulator = sum(weight * np.outer(q, q) for q, weight in zip(aligned, normalized))
    values, vectors = np.linalg.eigh(accumulator)
    quaternion = vectors[:, int(np.argmax(values))]
    if np.dot(quaternion, reference) < 0.0:
        quaternion = -quaternion
    return transform(quaternion_matrix(*quaternion), translation)


def solve_square_tag(corners, size_m: float, camera_matrix, distortion):
    half = 0.5 * float(size_m)
    object_points = np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    solved, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if (not solved or not np.all(np.isfinite(tvec)) or
            float(tvec.reshape(3)[2]) <= 0.0):
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)
    error = float(np.sqrt(np.mean(np.sum(
        (projected.reshape(4, 2) - image_points) ** 2, axis=1))))
    return transform(rotation, tvec.reshape(3)), object_points, error


def project_points(points, camera_matrix):
    values = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    valid = values[:, 2] > 1e-6
    if not np.all(valid):
        return None
    pixels = np.empty((values.shape[0], 2), dtype=np.float64)
    pixels[:, 0] = camera_matrix[0, 0] * values[:, 0] / values[:, 2] + camera_matrix[0, 2]
    pixels[:, 1] = camera_matrix[1, 1] * values[:, 1] / values[:, 2] + camera_matrix[1, 2]
    return pixels
