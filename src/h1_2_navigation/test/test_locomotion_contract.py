from pathlib import Path

from h1_2_navigation.locomotion_contract import build_observation
from h1_2_navigation.locomotion_contract import EXPECTED_GATES
from h1_2_navigation.locomotion_contract import EXPECTED_LEG_JOINTS
from h1_2_navigation.locomotion_contract import load_contract
from h1_2_navigation.locomotion_contract import verify_file_identity
from h1_2_navigation.v12_asset_audit import audit_assets
from h1_2_navigation.velocity_gate import VelocityGate
from h1_2_navigation.velocity_gate import VelocityLimits
import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = PACKAGE_ROOT / 'config' / 'v12_locomotion.yaml'


def _contract():
    return load_contract(CONTRACT_PATH)


def _passing_gates():
    return {name: True for name in EXPECTED_GATES}


def test_v12_contract_is_disabled_and_has_frozen_dimensions():
    contract = _contract()

    assert contract['deployment']['enabled'] is False
    assert contract['deployment']['movement_authorized'] is False
    assert contract['policy']['observation_dimension'] == 78
    assert contract['policy']['action_dimension'] == 12
    assert tuple(contract['leg_joints']) == EXPECTED_LEG_JOINTS
    assert contract['gazebo_runtime']['controller_mode'] == 'effort_pid'
    assert contract['gazebo_runtime']['left_load_topic'].endswith('/wrench')
    assert contract['gazebo_runtime']['right_load_topic'].endswith('/wrench')
    assert contract['gazebo_runtime']['load_contact_threshold'] == 5.0
    assert contract['gazebo_runtime']['armature_scale'] == 4.0
    assert contract['gazebo_runtime']['effort_stiffness_scale'] == 1.0
    assert contract['gazebo_runtime']['effort_damping_scale'] == 0.4
    assert contract['gazebo_runtime']['captured_effort_hold'] is True
    assert contract['gazebo_runtime']['captured_hold_return_rate'] == 0.0
    assert contract['gazebo_runtime']['captured_hold_angular_speed'] == 0.10
    assert contract['gazebo_runtime']['captured_hold_vertical_speed'] == 0.02
    assert contract['gazebo_runtime']['state_timeout'] == 0.25
    assert contract['gazebo_runtime']['base_frame'] == 'base_footprint'
    assert contract['gazebo_runtime']['body_frame'] == 'pelvis'
    assert contract['gazebo_runtime']['position_target_rate_limit'] == 0.02
    assert contract['gazebo_runtime']['position_stop_target_rate_limit'] == 0.001
    assert contract['gazebo_runtime']['effort_target_rate_limit'] == 0.001
    assert contract['gazebo_runtime']['effort_settle_steps'] == 100
    assert contract['gazebo_runtime']['position_proportional_gain'] == 1.0
    assert contract['gazebo_runtime']['deterministic_stop_adapter'] is False
    assert contract['gazebo_runtime']['stop_capture_dwell_steps'] == 10
    assert contract['gazebo_runtime']['motion_burst_steps'] == 15
    assert contract['gazebo_runtime']['motion_burst_planar_speed'] == 0.04
    assert contract['gazebo_runtime']['burst_restart_dwell_steps'] == 50
    assert contract['gazebo_runtime']['burst_restart_planar_speed'] == 0.02
    assert contract['gazebo_runtime']['burst_restart_minimum_up_z'] == 0.995
    assert contract['deployment_gait'] == {
        'stationary_threshold': 1.0e-6,
        'frequency_bias': 0.75,
        'frequency_speed_gain': 0.35,
        'frequency_yaw_gain': 0.08,
        'frequency_limits': [0.75, 1.25],
        'height_bias': 0.06,
        'height_speed_gain': 0.035,
        'height_limits': [0.06, 0.11],
        'gait_index': 0.0,
        'gazebo_adapter': {
            'frequency_scale': 0.6574621959237344,
            'minimum_foot_height': 0.08,
        },
    }


def test_v12_checkpoint_identity_matches_contract():
    contract = _contract()

    actual = verify_file_identity(contract['policy']['checkpoint'])

    assert actual == {
        'size': 3799642,
        'sha256': (
            '92ae9d9e945ed72e5aca91370acf94673d57181113ca89ab4f1c6639e0e87405'),
    }


def test_observation_builder_preserves_all_term_slices():
    contract = _contract()
    components = {}
    expected = np.empty(78, dtype=np.float32)
    for index, term in enumerate(contract['observation_layout'], start=1):
        size = term['end'] - term['start']
        values = np.full(size, index, dtype=np.float32)
        components[term['name']] = values
        expected[term['start']:term['end']] = values

    observation = build_observation(components, contract)

    np.testing.assert_array_equal(observation, expected)


def test_observation_builder_rejects_missing_or_nonfinite_terms():
    contract = _contract()
    components = {
        term['name']: np.zeros(term['end'] - term['start'])
        for term in contract['observation_layout']
    }
    components.pop('foot_contacts')
    with pytest.raises(ValueError, match='missing observation component'):
        build_observation(components, contract)

    components['foot_contacts'] = np.asarray([0.0, np.nan])
    with pytest.raises(ValueError, match='non-finite'):
        build_observation(components, contract)


def test_velocity_gate_defaults_to_exact_zero():
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()
    gates['movement_authorized'] = False

    output = gate.apply((0.1, -0.1, 0.2), 1.0, gates)

    np.testing.assert_array_equal(output, np.zeros(3))


@pytest.mark.parametrize('missing_gate', EXPECTED_GATES)
def test_velocity_gate_requires_every_gate(missing_gate):
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()
    gates[missing_gate] = False

    output = gate.apply((0.1, 0.1, 0.1), 1.0, gates)

    np.testing.assert_array_equal(output, np.zeros(3))


def test_velocity_gate_limits_acceleration_and_speed():
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()

    first = gate.apply((1.0, -1.0, 1.0), 1.0, gates)
    second = gate.apply((1.0, -1.0, 1.0), 1.02, gates)

    np.testing.assert_allclose(first, (0.003, -0.003, 0.004))
    np.testing.assert_allclose(second, (0.006, -0.006, 0.008))
    for step in range(2, 200):
        output = gate.apply(
            (1.0, -1.0, 1.0), 1.0 + 0.02 * step, gates)
    np.testing.assert_allclose(output, (0.15, -0.10, 0.05))


def test_velocity_gate_watchdog_does_not_consume_ramp_interval():
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()

    first = gate.apply((0.1, 0.0, 0.0), 1.0, gates)
    watched = gate.watchdog(1.01)
    second = gate.apply((0.1, 0.0, 0.0), 1.02, gates)

    np.testing.assert_allclose(first, (0.003, 0.0, 0.0))
    np.testing.assert_allclose(watched, first)
    np.testing.assert_allclose(second, (0.006, 0.0, 0.0))


def test_velocity_gate_emits_exact_zero_for_deadband_and_timeout():
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()
    gate.apply((0.1, 0.0, 0.0), 1.0, gates)

    stopped = gate.apply((0.005, 0.0, 0.0), 1.02, gates)
    stale = gate.watchdog(1.33)

    np.testing.assert_array_equal(stopped, np.zeros(3))
    np.testing.assert_array_equal(stale, np.zeros(3))


def test_velocity_gate_rejects_invalid_commands_and_time():
    contract = _contract()
    gate = VelocityGate(VelocityLimits.from_contract(contract))
    gates = _passing_gates()

    with pytest.raises(ValueError, match='three finite'):
        gate.apply((0.0, np.nan, 0.0), 1.0, gates)
    gate.apply((0.1, 0.0, 0.0), 1.0, gates)
    with pytest.raises(ValueError, match='monotonic'):
        gate.apply((0.1, 0.0, 0.0), 0.9, gates)


def test_asset_audit_confirms_matches_and_blocks_unready_transfer():
    report = audit_assets(_contract())

    assert report['baseline_valid'] is True
    assert report['deployment_ready'] is False
    assert report['snapshot_claims'] == {
        'task_source_sha256_match': True,
        'robot_asset_sha256_match': True,
    }
    assert report['matched_joint_count'] == 12
    assert report['velocity_limit_match_count'] == 0
    assert report['matched_inertial_count'] == 27
    assert report['common_inertial_count'] == 28
    assert report['inertial_mismatches'] == ['right_shoulder_pitch_link']
    assert report['total_mass'] == {
        'isaac_source': 66.984,
        'gazebo_source': 66.984,
    }
    assert report['collision_counts'] == {
        'isaac_source': 2,
        'gazebo_source': 22,
    }
    assert report['blockers'] == [
        'joint_velocity_limits',
        'inertials',
        'collision_geometry',
        'default_leg_pose',
        'free_root',
        'leg_controller',
        'imu_sensor',
        'foot_contact_sensors',
    ]
    assert report['gates']['policy_export'] is True
