"""Pure Gazebo upright-assistance calculations."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class UprightAssistanceConfig:
    nominal_height: float
    mass: float
    gravity: float
    vertical_stiffness: float
    vertical_damping: float
    maximum_vertical_force_fraction: float
    planar_damping: float
    maximum_planar_damping_force: float
    attitude_stiffness: float
    attitude_damping: float
    maximum_attitude_torque: float
    minimum_up_z: float

    def validate(self):
        positive = (
            self.nominal_height,
            self.mass,
            self.gravity,
            self.vertical_stiffness,
            self.vertical_damping,
            self.planar_damping,
            self.maximum_planar_damping_force,
            self.attitude_stiffness,
            self.attitude_damping,
            self.maximum_attitude_torque,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in positive):
            raise ValueError('upright assistance gains must be positive')
        if not 0.0 < self.maximum_vertical_force_fraction < 1.0:
            raise ValueError('vertical force fraction must be between 0 and 1')
        if not 0.0 < self.minimum_up_z <= 1.0:
            raise ValueError('minimum up-z must be between 0 and 1')


@dataclass(frozen=True)
class UprightAssistanceWrench:
    force_world: np.ndarray
    torque_world: np.ndarray
    up_world: np.ndarray
    linear_velocity_world: np.ndarray
    angular_velocity_world: np.ndarray
    active: bool


def quaternion_rotation_matrix(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError('quaternion must contain four finite values')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise ValueError('quaternion norm is zero')
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


def _finite_vector(values, name):
    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f'{name} must contain three finite values')
    return vector


def compute_upright_assistance(
        position_z,
        quaternion,
        linear_velocity_body,
        angular_velocity_body,
        config):
    config.validate()
    if not math.isfinite(position_z):
        raise ValueError('position_z must be finite')
    rotation = quaternion_rotation_matrix(quaternion)
    linear_world = rotation @ _finite_vector(
        linear_velocity_body, 'linear velocity')
    angular_world = rotation @ _finite_vector(
        angular_velocity_body, 'angular velocity')
    up_world = rotation[:, 2]
    force = np.zeros(3, dtype=np.float64)
    torque = np.zeros(3, dtype=np.float64)
    if up_world[2] < config.minimum_up_z:
        return UprightAssistanceWrench(
            force, torque, up_world, linear_world, angular_world, False)

    force[:2] = -config.planar_damping * linear_world[:2]
    planar_force_norm = float(np.linalg.norm(force[:2]))
    if planar_force_norm > config.maximum_planar_damping_force:
        force[:2] *= (
            config.maximum_planar_damping_force / planar_force_norm)

    height_error = config.nominal_height - float(position_z)
    vertical_force = (
        config.vertical_stiffness * height_error
        - config.vertical_damping * linear_world[2])
    maximum_force = (
        config.maximum_vertical_force_fraction
        * config.mass * config.gravity)
    force[2] = float(np.clip(vertical_force, 0.0, maximum_force))

    alignment_axis = np.cross(up_world, (0.0, 0.0, 1.0))
    torque[:2] = (
        config.attitude_stiffness * alignment_axis[:2]
        - config.attitude_damping * angular_world[:2])
    torque_norm = float(np.linalg.norm(torque[:2]))
    if torque_norm > config.maximum_attitude_torque:
        torque[:2] *= config.maximum_attitude_torque / torque_norm
    return UprightAssistanceWrench(
        force, torque, up_world, linear_world, angular_world, True)
