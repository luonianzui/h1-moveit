from pathlib import Path

from h1_2_navigation.locomotion_contract import load_contract
from h1_2_navigation.v12_policy import V12Policy
from h1_2_navigation.v12_runtime import V12LocomotionRuntime
from h1_2_navigation.v12_runtime import V12SensorState
import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = PACKAGE_ROOT / 'config' / 'v12_locomotion.yaml'


class FixedPolicy:
    def __init__(self, action):
        self.action = np.asarray(action, dtype=np.float32)

    def infer(self, observation):
        assert np.asarray(observation).shape == (78,)
        return self.action.copy()


def _state(step, leg_positions=None, forces=(20.0, 20.0), up_z=1.0):
    if leg_positions is None:
        leg_positions = np.asarray([
            0.0, -0.16, 0.0, 0.36, -0.20, 0.0,
            0.0, -0.16, 0.0, 0.36, -0.20, 0.0,
        ])
    return V12SensorState(
        step_index=step,
        imu_gyro=(0.0, 0.0, 0.0),
        imu_up_vector=(0.0, 0.0, up_z),
        root_linear_velocity_body=(0.0, 0.0, 0.0),
        root_angular_velocity_body=(0.0, 0.0, 0.0),
        torso_up_z=up_z,
        leg_positions=leg_positions,
        leg_velocities=np.arange(12, dtype=np.float32) * 0.01,
        shoulder_pitch_positions=(0.4, 0.4),
        foot_force_magnitudes=forces,
    )


def _runtime(action=None):
    if action is None:
        action = np.zeros(12)
    return V12LocomotionRuntime(
        load_contract(CONTRACT_PATH), FixedPolicy(action))


def test_moving_path_integrates_clipped_policy_action():
    runtime = _runtime(np.full(12, 2.0))

    output = runtime.step(_state(0), (0.1, 0.0, 0.0))

    assert output.mode == 'MOVE'
    np.testing.assert_array_equal(output.policy_action, np.ones(12))
    np.testing.assert_allclose(
        output.leg_position_targets,
        np.clip(
            runtime.default_leg_positions + 0.3,
            runtime.leg_minimum,
            runtime.leg_maximum,
        ),
        rtol=0.0,
        atol=3.0e-8,
    )
    np.testing.assert_allclose(
        output.observation[45:57],
        np.arange(12, dtype=np.float32) * 0.01,
    )
    assert output.observation[75] == pytest.approx(
        0.785 * 0.6574621959237344)
    assert output.observation[77] == pytest.approx(0.08)


def test_speed_coupled_gait_preserves_training_formula_before_gazebo_adapter():
    runtime = _runtime()

    command = np.asarray((0.1, 0.2, -0.3), dtype=np.float32)
    frequency, foot_height = runtime._training_gait_command(command)
    moving = runtime.step(_state(1), command)
    stopped = runtime.step(_state(2), (0.0, 0.0, 0.0))

    speed = np.linalg.norm((0.1, 0.2))
    assert frequency == pytest.approx(0.75 + 0.35 * speed + 0.08 * 0.3)
    assert foot_height == pytest.approx(0.06 + 0.035 * speed)
    assert moving.observation[75] == pytest.approx(
        frequency * 0.6574621959237344)
    assert moving.observation[77] == pytest.approx(0.08)
    np.testing.assert_allclose(
        stopped.observation[71:75], (1.0, -1.0, 0.0, 0.0), atol=1.0e-7)
    assert stopped.observation[75] == 0.0
    assert stopped.observation[77] == 0.0


def test_first_stop_action_is_rate_limited_without_target_jump():
    runtime = _runtime(np.ones(12))
    runtime.step(_state(0), (0.1, 0.0, 0.0))
    before = runtime.motor_targets.copy()

    output = runtime.step(_state(1), (0.0, 0.0, 0.0))

    assert output.mode == 'BRAKE'
    np.testing.assert_array_less(
        np.abs(output.leg_position_targets - before),
        runtime.stop_rate_limit + 1.0e-7,
    )
    assert output.stop_age == 1
    assert output.observation[54] == pytest.approx(-0.5)


def test_stable_dwell_latches_capture_and_recovery_clears_it():
    runtime = _runtime()
    output = None
    for step in range(11):
        output = runtime.step(_state(step), (0.0, 0.0, 0.0))

    assert output is not None
    assert output.mode == 'HOLD'
    assert output.capture_latched is True
    assert output.capture_stable_steps == 10
    assert output.observation[54] > 0.5

    breached = _state(11, forces=(20.0, 0.0))
    output = runtime.step(breached, (0.0, 0.0, 0.0))

    assert output.mode == 'BRAKE'
    assert output.capture_latched is False
    assert output.capture_stable_steps == 0


def test_runtime_rejects_reused_sensor_sample():
    runtime = _runtime()
    runtime.step(_state(0), (0.0, 0.0, 0.0))

    with pytest.raises(ValueError, match='increase strictly'):
        runtime.step(_state(0), (0.0, 0.0, 0.0))


def test_frozen_policy_and_runtime_produce_finite_zero_motion_result():
    policy = V12Policy.load(
        PACKAGE_ROOT / 'policy' / 'v12_model_17250_policy.npz')
    runtime = V12LocomotionRuntime(load_contract(CONTRACT_PATH), policy)

    output = runtime.step(_state(0), (0.0, 0.0, 0.0))

    assert output.mode == 'BRAKE'
    assert output.observation.shape == (78,)
    assert output.policy_action.shape == (12,)
    assert np.isfinite(output.leg_position_targets).all()
