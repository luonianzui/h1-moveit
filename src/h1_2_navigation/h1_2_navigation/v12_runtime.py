"""Stateful, command-free V12 locomotion runtime core."""

from dataclasses import dataclass
import math

import numpy as np

from .locomotion_contract import build_observation
from .locomotion_contract import EXPECTED_LEG_JOINTS
from .locomotion_contract import validate_contract


@dataclass(frozen=True)
class V12SensorState:
    """One synchronized 50 Hz sensor sample in the V12 training frames."""

    step_index: int
    imu_gyro: object
    imu_up_vector: object
    root_linear_velocity_body: object
    root_angular_velocity_body: object
    torso_up_z: float
    leg_positions: object
    leg_velocities: object
    shoulder_pitch_positions: object
    foot_force_magnitudes: object


@dataclass(frozen=True)
class V12RuntimeOutput:
    """Pure result; publishing remains the responsibility of a gated ROS node."""

    mode: str
    observation: np.ndarray
    policy_action: np.ndarray
    leg_position_targets: np.ndarray
    shoulder_pitch_targets: np.ndarray
    capture_latched: bool
    capture_stable_steps: int
    stop_age: int


def _vector(values, size, name):
    result = np.asarray(values, dtype=np.float32).reshape(-1)
    if result.size != size:
        raise ValueError(f'{name} must contain {size} values')
    if not np.isfinite(result).all():
        raise ValueError(f'{name} contains non-finite values')
    return result


class V12LocomotionRuntime:
    """Port the V12 observation, MOVE, BRAKE, HOLD, and recovery semantics."""

    MOVE = 0
    STOP = 1

    def __init__(self, contract, policy):
        validate_contract(contract)
        self.contract = contract
        self.policy = policy
        defaults = contract['training_default_positions']
        self.default_leg_positions = np.asarray(
            [defaults[name] for name in EXPECTED_LEG_JOINTS],
            dtype=np.float32)
        physical_limits = np.asarray([
            contract['joint_position_limits'][name]
            for name in EXPECTED_LEG_JOINTS
        ], dtype=np.float32)
        center = np.mean(physical_limits, axis=1)
        half_range = (
            0.5 * contract['control']['leg_limit_fraction']
            * (physical_limits[:, 1] - physical_limits[:, 0]))
        self.leg_minimum = center - half_range
        self.leg_maximum = center + half_range
        shoulder = contract['shoulder_pitch']
        self.shoulder_defaults = np.asarray(
            shoulder['default_positions'], dtype=np.float32)
        shoulder_limits = np.asarray(
            shoulder['position_limits'], dtype=np.float32)
        self.shoulder_minimum = shoulder_limits[:, 0]
        self.shoulder_maximum = shoulder_limits[:, 1]
        self.action_scale = np.float32(contract['control']['action_scale'])
        self.action_clip = np.float32(contract['policy']['action_clip'])
        stop = contract['stop_controller']
        self.stop_residual_scale = np.asarray(
            stop['stop_residual_scale'], dtype=np.float32)
        self.stop_rate_limit = np.asarray(
            stop['stop_target_rate_limit'], dtype=np.float32)
        self.stop_shoulder_rate_limit = np.float32(
            stop['stop_shoulder_rate_limit'])
        self.reset()

    def reset(self):
        """Reset all policy-visible and controller state without publishing."""
        self.motor_targets = self.default_leg_positions.copy()
        self.shoulder_targets = self.shoulder_defaults.copy()
        self.stop_targets = self.motor_targets.copy()
        self.stop_shoulder_targets = self.shoulder_targets.copy()
        self.capture_targets = self.motor_targets.copy()
        self.capture_shoulder_targets = self.shoulder_targets.copy()
        self.previous_action = np.zeros(12, dtype=np.float32)
        self.stop_state = self.MOVE
        self.stop_age = 0
        self.capture_latched = False
        self.capture_stable_steps = 0
        self.current_root_vxy = np.zeros(2, dtype=np.float32)
        self.previous_root_vxy = np.zeros(2, dtype=np.float32)
        self.last_nonzero_command = np.zeros(3, dtype=np.float32)
        self.transition_phase = np.zeros(2, dtype=np.float32)
        self.last_motion_phase = np.float32(0.0)
        self.last_gait_frequency = np.float32(0.0)
        self.context_was_zero = False
        self.last_step_index = None

    def _state(self, sample):
        if not isinstance(sample.step_index, int) or sample.step_index < 0:
            raise ValueError('step_index must be a non-negative integer')
        if (
                self.last_step_index is not None
                and sample.step_index <= self.last_step_index):
            raise ValueError('step_index must increase strictly')
        values = {
            'imu_gyro': _vector(sample.imu_gyro, 3, 'imu_gyro'),
            'imu_up_vector': _vector(
                sample.imu_up_vector, 3, 'imu_up_vector'),
            'root_linear_velocity_body': _vector(
                sample.root_linear_velocity_body, 3,
                'root_linear_velocity_body'),
            'root_angular_velocity_body': _vector(
                sample.root_angular_velocity_body, 3,
                'root_angular_velocity_body'),
            'leg_positions': _vector(
                sample.leg_positions, 12, 'leg_positions'),
            'leg_velocities': _vector(
                sample.leg_velocities, 12, 'leg_velocities'),
            'shoulder_pitch_positions': _vector(
                sample.shoulder_pitch_positions, 2,
                'shoulder_pitch_positions'),
            'foot_force_magnitudes': _vector(
                sample.foot_force_magnitudes, 2,
                'foot_force_magnitudes'),
        }
        if not math.isfinite(sample.torso_up_z):
            raise ValueError('torso_up_z must be finite')
        values['torso_up_z'] = np.float32(sample.torso_up_z)
        return values

    def _support_contacts(self, state):
        threshold = self.contract['stop_controller'][
            'contact_force_threshold']
        return state['foot_force_magnitudes'] > threshold

    def _capture_candidate(self, state):
        stop = self.contract['stop_controller']
        support = self._support_contacts(state)
        return bool(
            np.linalg.norm(state['root_linear_velocity_body'][:2])
            < stop['capture_planar_speed_threshold']
            and abs(state['root_angular_velocity_body'][2])
            < stop['capture_yaw_speed_threshold']
            and state['torso_up_z'] > stop['capture_minimum_up_z']
            and np.linalg.norm(state['root_angular_velocity_body'][:2])
            < stop['capture_angular_velocity_xy_threshold']
            and abs(state['root_linear_velocity_body'][2])
            < stop['capture_vertical_velocity_threshold']
            and bool(np.all(support)))

    def _recovery_breach(self, state):
        stop = self.contract['stop_controller']
        support = self._support_contacts(state)
        return bool(
            state['torso_up_z'] < stop['recovery_minimum_up_z']
            or np.linalg.norm(state['root_linear_velocity_body'][:2])
            > stop['recovery_planar_speed_threshold']
            or abs(state['root_angular_velocity_body'][2])
            > stop['recovery_yaw_speed_threshold']
            or np.linalg.norm(state['root_angular_velocity_body'][:2])
            > stop['recovery_angular_velocity_xy_threshold']
            or abs(state['root_linear_velocity_body'][2])
            > stop['recovery_vertical_velocity_threshold']
            or not bool(np.all(support)))

    def _training_gait_command(self, command):
        gait = self.contract['deployment_gait']
        planar_speed = float(np.linalg.norm(command[:2]))
        yaw_speed = abs(float(command[2]))
        if (
                planar_speed <= gait['stationary_threshold']
                and yaw_speed <= gait['stationary_threshold']):
            return np.float32(0.0), np.float32(0.0)
        frequency = np.clip(
            gait['frequency_bias']
            + gait['frequency_speed_gain'] * planar_speed
            + gait['frequency_yaw_gain'] * yaw_speed,
            gait['frequency_limits'][0], gait['frequency_limits'][1])
        foot_height = np.clip(
            gait['height_bias']
            + gait['height_speed_gain'] * planar_speed,
            gait['height_limits'][0], gait['height_limits'][1])
        return np.float32(frequency), np.float32(foot_height)

    def _gait_command(self, command):
        frequency, foot_height = self._training_gait_command(command)
        if frequency == 0.0:
            return frequency, foot_height
        adapter = self.contract['deployment_gait']['gazebo_adapter']
        return (
            np.float32(frequency * adapter['frequency_scale']),
            np.float32(max(
                foot_height, adapter['minimum_foot_height'])),
        )

    def _gait_phase(self, step_index, gait_frequency):
        phase_steps = max(step_index - 1, 0)
        return np.float32(
            2.0 * math.pi * self.contract['control']['control_period']
            * gait_frequency * phase_steps)

    def _refresh_context(
            self, state, command, step_index, gait_frequency):
        zero = bool(np.all(command == 0.0))
        moving = not zero
        candidate = self._capture_candidate(state)
        if zero and candidate and not self.capture_latched:
            self.capture_stable_steps += 1
        if moving or not candidate:
            self.capture_stable_steps = 0
        if zero and self.capture_latched and self._recovery_breach(state):
            self.capture_latched = False
            self.capture_stable_steps = 0

        previously_latched = self.capture_latched
        if moving:
            self.capture_latched = False
        dwell = self.contract['stop_controller']['capture_dwell_steps']
        if zero and candidate and self.capture_stable_steps >= dwell:
            self.capture_latched = True

        self.previous_root_vxy[:] = self.current_root_vxy
        if self.last_step_index is None:
            self.previous_root_vxy[:] = state[
                'root_linear_velocity_body'][:2]
        self.current_root_vxy[:] = state['root_linear_velocity_body'][:2]

        phase = self._gait_phase(step_index, gait_frequency)
        if moving:
            self.last_nonzero_command[:] = command
            self.last_motion_phase = phase
            self.last_gait_frequency = gait_frequency
            self.transition_phase[:] = (math.sin(phase), math.cos(phase))
        entering_zero = zero and not self.context_was_zero
        if entering_zero:
            transition_angle = self.last_motion_phase + np.float32(
                2.0 * math.pi
                * self.contract['control']['control_period']
                * self.last_gait_frequency)
            self.transition_phase[:] = (
                math.sin(transition_angle), math.cos(transition_angle))
        self.context_was_zero = zero

        if self.capture_latched and not previously_latched:
            self.capture_targets[:] = state['leg_positions']
            self.capture_shoulder_targets[:] = state[
                'shoulder_pitch_positions']

        normalized_age = np.clip(
            self.stop_age
            / self.contract['stop_controller']['stop_age_scale_steps'],
            0.0, 1.0)
        capture_and_age = np.float32(0.5 + 0.5 * normalized_age)
        if not self.capture_latched:
            capture_and_age = -capture_and_age
        if moving:
            capture_and_age = np.float32(0.0)
        support = self._support_contacts(state).astype(np.float32)
        return np.concatenate((
            self.current_root_vxy,
            self.previous_root_vxy,
            self.last_nonzero_command,
            self.transition_phase,
            np.asarray([capture_and_age], dtype=np.float32),
            support,
        )).astype(np.float32)

    def _observation(
            self, state, command, stop_context, step_index,
            gait_frequency, foot_height):
        phase = self._gait_phase(step_index, gait_frequency)
        gait_phase = np.asarray([
            math.cos(phase), math.cos(phase + math.pi),
            math.sin(phase), math.sin(phase + math.pi),
        ], dtype=np.float32)
        zero = bool(np.all(command == 0.0))
        velocity_or_context = (
            stop_context if zero else state['leg_velocities'])
        return build_observation({
            'imu_gyro': state['imu_gyro'],
            'imu_up_vector': state['imu_up_vector'],
            'joint_position': (
                state['leg_positions'] - self.default_leg_positions),
            'joint_velocity': state['leg_velocities'],
            'previous_action': self.previous_action,
            'velocity_command': command,
            'joint_velocity_or_stop_context': velocity_or_context,
            'joint_position_target_error': (
                state['leg_positions'] - self.motor_targets),
            'foot_contacts': (
                state['foot_force_magnitudes'] > 0.0).astype(np.float32),
            'gait_phase': gait_phase,
            'gait_frequency': [gait_frequency],
            'gait_index': [self.contract['deployment_gait']['gait_index']],
            'foot_height': [foot_height],
        }, self.contract)

    def _moving_shoulders(self, command, step_index, gait_frequency):
        shoulder = self.contract['shoulder_pitch']
        planar_speed = float(np.linalg.norm(command[:2]))
        amplitude = shoulder['peak'] * np.clip(
            planar_speed / shoulder['full_speed'], 0.0, 1.0)
        swing = np.float32(amplitude * math.sin(
            self._gait_phase(step_index, gait_frequency)))
        targets = self.shoulder_defaults + np.asarray(
            [-swing, swing], dtype=np.float32)
        return np.clip(
            targets, self.shoulder_minimum, self.shoulder_maximum)

    def _process_action(
            self, state, command, action, step_index, gait_frequency):
        previous_motor_targets = self.motor_targets.copy()
        previous_shoulders = self.shoulder_targets.copy()
        parent_motor_targets = np.clip(
            self.motor_targets + self.action_scale * action,
            self.leg_minimum,
            self.leg_maximum,
        )
        parent_shoulders = self._moving_shoulders(
            command, step_index, gait_frequency)
        zero = bool(np.all(command == 0.0))
        moving = not zero
        entering_stop = zero and self.stop_state == self.MOVE

        if moving:
            self.stop_state = self.MOVE
            self.capture_latched = False
            self.capture_stable_steps = 0
            self.stop_targets[:] = parent_motor_targets
            self.stop_shoulder_targets[:] = parent_shoulders
            self.stop_age = 0
        if entering_stop:
            self.stop_state = self.STOP
            self.stop_targets[:] = previous_motor_targets
            self.stop_shoulder_targets[:] = previous_shoulders
            self.stop_age = 0
            self.capture_latched = False
            self.capture_stable_steps = 0

        reference = (
            self.capture_targets if zero and self.capture_latched
            else state['leg_positions'])
        desired = np.clip(
            reference + self.stop_residual_scale * np.tanh(action),
            self.leg_minimum,
            self.leg_maximum,
        )
        target_delta = np.clip(
            desired - self.stop_targets,
            -self.stop_rate_limit,
            self.stop_rate_limit,
        )
        if zero:
            self.stop_targets += target_delta
            self.stop_age += 1

        shoulder_reference = (
            self.capture_shoulder_targets
            if zero and self.capture_latched
            else self.stop_shoulder_targets)
        shoulder_delta = np.clip(
            shoulder_reference - self.stop_shoulder_targets,
            -self.stop_shoulder_rate_limit,
            self.stop_shoulder_rate_limit,
        )
        if zero:
            self.stop_shoulder_targets += shoulder_delta
        self.stop_shoulder_targets[:] = np.clip(
            self.stop_shoulder_targets,
            self.shoulder_minimum,
            self.shoulder_maximum,
        )
        self.motor_targets[:] = (
            self.stop_targets if zero else parent_motor_targets)
        self.shoulder_targets[:] = (
            self.stop_shoulder_targets if zero else parent_shoulders)

    def step(self, sample, velocity_command):
        """Compute one 50 Hz result without publishing controller commands."""
        state = self._state(sample)
        command = _vector(velocity_command, 3, 'velocity_command')
        gait_frequency, foot_height = self._gait_command(command)
        stop_context = self._refresh_context(
            state, command, sample.step_index, gait_frequency)
        observation = self._observation(
            state, command, stop_context, sample.step_index,
            gait_frequency, foot_height)
        action = np.clip(
            self.policy.infer(observation),
            -self.action_clip,
            self.action_clip,
        ).astype(np.float32)
        self._process_action(
            state, command, action, sample.step_index, gait_frequency)
        self.previous_action[:] = action
        self.last_step_index = sample.step_index
        if not np.isfinite(self.motor_targets).all():
            raise ValueError('V12 runtime produced non-finite leg targets')
        mode = 'MOVE'
        if self.stop_state == self.STOP:
            mode = 'HOLD' if self.capture_latched else 'BRAKE'
        return V12RuntimeOutput(
            mode=mode,
            observation=observation.copy(),
            policy_action=action.copy(),
            leg_position_targets=self.motor_targets.copy(),
            shoulder_pitch_targets=self.shoulder_targets.copy(),
            capture_latched=self.capture_latched,
            capture_stable_steps=self.capture_stable_steps,
            stop_age=self.stop_age,
        )
