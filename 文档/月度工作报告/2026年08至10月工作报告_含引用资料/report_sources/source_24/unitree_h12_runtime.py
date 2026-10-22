"""Official Unitree H1_2 observation and direct-target runtime."""

from dataclasses import dataclass
import math

import numpy as np

from .unitree_h12_contract import build_observation


@dataclass(frozen=True)
class UnitreeH12SensorState:
    """One synchronous policy input sample."""

    base_quaternion_xyzw: tuple
    base_angular_velocity: tuple
    leg_positions: np.ndarray
    leg_velocities: np.ndarray


@dataclass(frozen=True)
class UnitreeH12Output:
    """One policy action and its direct PD position target."""

    observation: np.ndarray
    policy_action: np.ndarray
    leg_position_targets: np.ndarray
    phase: float


def projected_gravity(quaternion_xyzw):
    """Return world gravity expressed in the robot body frame."""
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(-1)
    if quaternion.size != 4 or not np.isfinite(quaternion).all():
        raise ValueError('quaternion must contain four finite values')
    norm = np.linalg.norm(quaternion)
    if norm < 1.0e-9:
        raise ValueError('quaternion norm is zero')
    x_value, y_value, z_value, w_value = quaternion / norm
    return np.asarray([
        2.0 * (-z_value * x_value + w_value * y_value),
        -2.0 * (z_value * y_value + w_value * x_value),
        1.0 - 2.0 * (w_value * w_value + z_value * z_value),
    ], dtype=np.float32)


class UnitreeH12Runtime:
    """Preserve the official 50 Hz recurrent observation/action semantics."""

    def __init__(self, contract, policy):
        self.contract = contract
        self.policy = policy
        control = contract['control']
        self.control_period = float(control['control_period'])
        self.action_scale = np.float32(control['action_scale'])
        self.action_clip = np.float32(contract['policy']['action_clip'])
        self.gait_period = float(contract['policy']['gait_period'])
        self.scales = {
            name: np.asarray(values, dtype=np.float32)
            for name, values in control['scales'].items()
        }
        defaults = contract['training_default_positions']
        self.default_positions = np.asarray(
            list(defaults.values()), dtype=np.float32)
        limits = contract['joint_position_limits']
        self.minimum_positions = np.asarray(
            [limits[name][0] for name in defaults], dtype=np.float32)
        self.maximum_positions = np.asarray(
            [limits[name][1] for name in defaults], dtype=np.float32)
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.step_index = 0

    def reset(self):
        """Reset phase, previous action, and recurrent memory."""
        self.previous_action.fill(0.0)
        self.step_index = 0
        self.policy.reset()

    def step(self, sensor_state, command):
        """Advance one official policy period."""
        command = np.asarray(command, dtype=np.float32).reshape(-1)
        positions = np.asarray(
            sensor_state.leg_positions, dtype=np.float32).reshape(-1)
        velocities = np.asarray(
            sensor_state.leg_velocities, dtype=np.float32).reshape(-1)
        angular_velocity = np.asarray(
            sensor_state.base_angular_velocity,
            dtype=np.float32).reshape(-1)
        if command.size != 3 or positions.size != 12 or velocities.size != 12:
            raise ValueError('invalid Unitree H1_2 runtime vector size')
        if angular_velocity.size != 3:
            raise ValueError('base angular velocity must have three values')
        if not all(np.isfinite(values).all() for values in (
                command, positions, velocities, angular_velocity)):
            raise ValueError('Unitree H1_2 runtime input is non-finite')
        self.step_index += 1
        elapsed = self.step_index * self.control_period
        phase = math.fmod(elapsed, self.gait_period) / self.gait_period
        phase_angle = 2.0 * math.pi * phase
        observation = build_observation({
            'base_angular_velocity': (
                angular_velocity * self.scales['angular_velocity']),
            'projected_gravity': projected_gravity(
                sensor_state.base_quaternion_xyzw),
            'velocity_command': command * self.scales['velocity_command'],
            'joint_position': (
                (positions - self.default_positions)
                * self.scales['joint_position']),
            'joint_velocity': velocities * self.scales['joint_velocity'],
            'previous_action': self.previous_action,
            'gait_phase': [math.sin(phase_angle), math.cos(phase_angle)],
        }, self.contract)
        action = np.clip(
            self.policy.infer(observation),
            -self.action_clip,
            self.action_clip).astype(np.float32, copy=False)
        targets = np.clip(
            self.default_positions + self.action_scale * action,
            self.minimum_positions,
            self.maximum_positions).astype(np.float32, copy=False)
        self.previous_action[:] = action
        return UnitreeH12Output(
            observation=observation,
            policy_action=action.copy(),
            leg_position_targets=targets.copy(),
            phase=phase,
        )
