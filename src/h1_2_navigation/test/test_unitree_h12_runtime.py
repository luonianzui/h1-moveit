from pathlib import Path

import numpy as np

from h1_2_navigation.unitree_h12_contract import load_contract
from h1_2_navigation.unitree_h12_policy import UnitreeH12Policy
from h1_2_navigation.unitree_h12_runtime import projected_gravity
from h1_2_navigation.unitree_h12_runtime import UnitreeH12Runtime
from h1_2_navigation.unitree_h12_runtime import UnitreeH12SensorState


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT = PACKAGE_ROOT / 'config' / 'unitree_h12_locomotion.yaml'
POLICY = PACKAGE_ROOT / 'policy' / 'unitree_h12_motion_policy.npz'


def _runtime():
    contract = load_contract(CONTRACT)
    return contract, UnitreeH12Runtime(
        contract, UnitreeH12Policy.load(POLICY))


def _standing_state(contract):
    defaults = np.asarray(list(
        contract['training_default_positions'].values()), dtype=np.float32)
    return UnitreeH12SensorState(
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        base_angular_velocity=(0.0, 0.0, 0.0),
        leg_positions=defaults,
        leg_velocities=np.zeros(12, dtype=np.float32),
    )


def test_projected_gravity_matches_official_identity_convention():
    np.testing.assert_array_equal(
        projected_gravity((0.0, 0.0, 0.0, 1.0)),
        np.asarray([0.0, 0.0, -1.0], dtype=np.float32))


def test_runtime_builds_official_observation_and_direct_targets():
    contract, runtime = _runtime()
    output = runtime.step(
        _standing_state(contract), (0.5, 0.0, 0.0))
    expected_phase = 0.02 / 0.8

    np.testing.assert_array_equal(output.observation[:3], np.zeros(3))
    np.testing.assert_array_equal(
        output.observation[3:6], [0.0, 0.0, -1.0])
    np.testing.assert_array_equal(
        output.observation[6:9], [1.0, 0.0, 0.0])
    np.testing.assert_array_equal(output.observation[9:45], np.zeros(36))
    np.testing.assert_allclose(
        output.observation[45:47],
        [
            np.sin(2.0 * np.pi * expected_phase),
            np.cos(2.0 * np.pi * expected_phase),
        ],
        rtol=0.0, atol=1.0e-7)
    defaults = np.asarray(list(
        contract['training_default_positions'].values()), dtype=np.float32)
    limits = contract['joint_position_limits']
    minimum = np.asarray([values[0] for values in limits.values()])
    maximum = np.asarray([values[1] for values in limits.values()])
    expected_targets = np.clip(
        defaults + 0.25 * output.policy_action, minimum, maximum)
    np.testing.assert_allclose(
        output.leg_position_targets, expected_targets,
        rtol=0.0, atol=1.0e-7)


def test_runtime_feeds_previous_action_and_reset_restores_first_step():
    contract, runtime = _runtime()
    state = _standing_state(contract)
    first = runtime.step(state, (0.5, 0.0, 0.0))
    second = runtime.step(state, (0.5, 0.0, 0.0))

    np.testing.assert_array_equal(
        second.observation[33:45], first.policy_action)
    runtime.reset()
    repeated = runtime.step(state, (0.5, 0.0, 0.0))
    np.testing.assert_array_equal(repeated.observation, first.observation)
    np.testing.assert_array_equal(repeated.policy_action, first.policy_action)


def test_zero_command_preserves_recurrent_state_and_gait_phase():
    contract, runtime = _runtime()
    state = _standing_state(contract)
    moving = runtime.step(state, (0.05, 0.0, 0.0))
    stopped = runtime.step(state, (0.0, 0.0, 0.0))

    np.testing.assert_array_equal(stopped.observation[6:9], np.zeros(3))
    np.testing.assert_array_equal(
        stopped.observation[33:45], moving.policy_action)
    expected_phase = 2.0 * 0.02 / 0.8
    np.testing.assert_allclose(
        stopped.observation[45:47],
        [
            np.sin(2.0 * np.pi * expected_phase),
            np.cos(2.0 * np.pi * expected_phase),
        ],
        rtol=0.0, atol=1.0e-7)
