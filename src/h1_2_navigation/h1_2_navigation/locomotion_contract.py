"""Validation and assembly helpers for the frozen V12 locomotion contract."""

import hashlib
import math
from pathlib import Path

import numpy as np
import yaml


EXPECTED_LEG_JOINTS = (
    'left_hip_yaw_joint',
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_yaw_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
)
EXPECTED_GATES = (
    'enabled',
    'movement_authorized',
    'policy_ready',
    'state_estimate_ready',
    'contact_sensors_ready',
    'leg_controller_ready',
)
EXPECTED_CONTROLLED_JOINTS = EXPECTED_LEG_JOINTS + (
    'torso_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
)


def load_contract(path):
    """Load and validate a V12 locomotion YAML contract."""
    with Path(path).open(encoding='utf-8') as stream:
        contract = yaml.safe_load(stream)
    validate_contract(contract)
    return contract


def validate_contract(contract):
    """Reject contract drift that could change V12 policy semantics."""
    if contract.get('schema_version') != 1:
        raise ValueError('unsupported locomotion contract schema')
    deployment = contract['deployment']
    if deployment['enabled'] is not False:
        raise ValueError('locomotion must default to disabled')
    if deployment['movement_authorized'] is not False:
        raise ValueError('movement authorization must default to false')
    if tuple(deployment['required_gates']) != EXPECTED_GATES:
        raise ValueError('required locomotion gates changed')

    policy = contract['policy']
    if policy['observation_dimension'] != 78:
        raise ValueError('V12 observation dimension must be 78')
    if policy['action_dimension'] != len(EXPECTED_LEG_JOINTS):
        raise ValueError('V12 action dimension must be 12')
    if tuple(contract['leg_joints']) != EXPECTED_LEG_JOINTS:
        raise ValueError('V12 leg joint order changed')

    cursor = 0
    names = set()
    for term in contract['observation_layout']:
        name = term['name']
        if name in names:
            raise ValueError(f'duplicate observation term {name!r}')
        if term['start'] != cursor or term['end'] <= term['start']:
            raise ValueError(f'non-contiguous observation term {name!r}')
        names.add(name)
        cursor = term['end']
    if cursor != policy['observation_dimension']:
        raise ValueError('observation layout does not total 78 values')

    control = contract['control']
    if not math.isclose(control['control_period'], 0.02):
        raise ValueError('V12 control period must remain 0.02 seconds')
    if not math.isclose(control['action_scale'], 0.3):
        raise ValueError('V12 action scale must remain 0.3')
    if len(control['command_deadband']) != 3:
        raise ValueError('command deadband must contain three values')
    if any(value < 0.0 for value in control['command_deadband']):
        raise ValueError('command deadband must be non-negative')
    onset = control['motion_onset_threshold']
    if len(onset) != 3 or any(
            not math.isfinite(value) or value <= 0.0 for value in onset):
        raise ValueError('motion onset threshold must contain three positives')
    if any(
            threshold <= deadband
            for threshold, deadband in zip(
                onset, control['command_deadband'])):
        raise ValueError('motion onset threshold must exceed command deadband')
    if control['command_timeout'] <= 0.0:
        raise ValueError('command timeout must be positive')

    for axis in ('vx', 'vy', 'yaw_rate'):
        training = control['training_limits'][axis]
        safety = control['safety_limits'][axis]
        if len(training) != 2 or len(safety) != 2:
            raise ValueError(f'{axis} limits must contain two values')
        if training[0] >= training[1] or safety[0] >= safety[1]:
            raise ValueError(f'{axis} limits are not ordered')
        if safety[0] < training[0] or safety[1] > training[1]:
            raise ValueError(f'{axis} safety limits exceed training limits')
    for name in ('linear_acceleration', 'angular_acceleration'):
        if control['safety_limits'][name] <= 0.0:
            raise ValueError(f'{name} must be positive')

    defaults = contract['training_default_positions']
    if tuple(defaults) != EXPECTED_LEG_JOINTS:
        raise ValueError('training default position order changed')
    position_limits = contract['joint_position_limits']
    if tuple(position_limits) != EXPECTED_LEG_JOINTS:
        raise ValueError('joint position limit order changed')
    if any(
            len(position_limits[name]) != 2
            or position_limits[name][0] >= position_limits[name][1]
            for name in EXPECTED_LEG_JOINTS):
        raise ValueError('invalid joint position limits')

    gait = contract['deployment_gait']
    expected_gait = {
        'stationary_threshold': 1.0e-6,
        'frequency_bias': 0.75,
        'frequency_speed_gain': 0.35,
        'frequency_yaw_gain': 0.08,
        'height_bias': 0.06,
        'height_speed_gain': 0.035,
    }
    for name, expected in expected_gait.items():
        if not math.isclose(gait[name], expected):
            raise ValueError(f'V12 speed-coupled gait {name} changed')
    for name, expected in (
            ('frequency_limits', (0.75, 1.25)),
            ('height_limits', (0.06, 0.11))):
        values = gait[name]
        if len(values) != 2 or any(
                not math.isclose(value, reference)
                for value, reference in zip(values, expected)):
            raise ValueError(f'V12 speed-coupled gait {name} changed')
    adapter = gait['gazebo_adapter']
    if not math.isclose(adapter['frequency_scale'], 0.6574621959237344):
        raise ValueError('Gazebo gait frequency scale changed')
    if not math.isclose(adapter['minimum_foot_height'], 0.08):
        raise ValueError('Gazebo minimum foot height changed')
    if gait['gait_index'] != 0.0:
        raise ValueError('V12 gait index must remain zero')

    shoulder = contract['shoulder_pitch']
    if shoulder['joints'] != [
            'left_shoulder_pitch_joint', 'right_shoulder_pitch_joint']:
        raise ValueError('shoulder pitch joint order changed')
    if len(shoulder['default_positions']) != 2:
        raise ValueError('two shoulder default positions are required')
    if len(shoulder['position_limits']) != 2 or any(
            len(values) != 2 or values[0] >= values[1]
            for values in shoulder['position_limits']):
        raise ValueError('invalid shoulder position limits')
    if shoulder['peak'] < 0.0 or shoulder['full_speed'] <= 0.0:
        raise ValueError('invalid shoulder motion parameters')

    actuator = contract['training_actuator']
    if tuple(actuator['controlled_joints']) != EXPECTED_CONTROLLED_JOINTS:
        raise ValueError('V12 controlled joint order changed')
    actuator_size = len(EXPECTED_CONTROLLED_JOINTS)
    for name in ('default_positions', 'stiffness', 'damping', 'effort_limits'):
        values = actuator[name]
        if len(values) != actuator_size or any(
                not math.isfinite(value) for value in values):
            raise ValueError(f'invalid training actuator {name}')
    if any(value <= 0.0 for value in actuator['stiffness']):
        raise ValueError('training actuator stiffness must be positive')
    if any(value <= 0.0 for value in actuator['damping']):
        raise ValueError('training actuator damping must be positive')
    if any(value <= 0.0 for value in actuator['effort_limits']):
        raise ValueError('training actuator effort limits must be positive')
    if actuator['joint_velocity_limit'] <= 0.0:
        raise ValueError('training actuator velocity limit must be positive')
    for name in ('armature', 'friction', 'dynamic_friction', 'viscous_friction'):
        if not math.isfinite(actuator[name]) or actuator[name] < 0.0:
            raise ValueError(f'invalid training actuator {name}')

    gazebo = contract['gazebo_runtime']
    if gazebo['controller_mode'] not in ('position', 'effort_pid'):
        raise ValueError(
            'Gazebo V12 controller mode must be position or effort_pid')
    if (
            gazebo['standby_controller_name']
            == gazebo['effort_controller_name']):
        raise ValueError('Gazebo standby and effort controllers must differ')
    if (
            not math.isfinite(gazebo['switch_timeout'])
            or gazebo['switch_timeout'] <= 0.0):
        raise ValueError('invalid Gazebo controller switch timeout')
    if gazebo['truth_aided_odometry'] is not True:
        raise ValueError('Gazebo runtime must label truth-aided odometry')
    for name in (
            'state_timeout', 'load_contact_threshold',
            'armature_scale', 'effort_stiffness_scale',
            'effort_damping_scale',
            'position_target_rate_limit', 'position_stop_target_rate_limit',
            'effort_target_rate_limit',
            'captured_hold_angular_speed',
            'captured_hold_vertical_speed',
            'motion_burst_planar_speed', 'burst_restart_planar_speed',
            'fall_minimum_height'):
        if not math.isfinite(gazebo[name]) or gazebo[name] <= 0.0:
            raise ValueError(f'invalid Gazebo runtime {name}')
    if (
            not math.isfinite(gazebo['captured_hold_return_rate'])
            or gazebo['captured_hold_return_rate'] < 0.0):
        raise ValueError('invalid Gazebo runtime captured_hold_return_rate')
    if gazebo['position_target_rate_limit'] > control['action_scale']:
        raise ValueError('Gazebo position target rate limit exceeds action scale')
    if gazebo['position_stop_target_rate_limit'] > (
            gazebo['position_target_rate_limit']):
        raise ValueError(
            'Gazebo position stop rate exceeds position target rate')
    if gazebo['armature_scale'] < 1.0:
        raise ValueError('Gazebo armature scale cannot reduce training inertia')
    for name in ('effort_stiffness_scale', 'effort_damping_scale'):
        if not 0.0 < gazebo[name] <= 1.0:
            raise ValueError(f'Gazebo {name} must be in (0, 1]')
    if gazebo['effort_target_rate_limit'] > control['action_scale']:
        raise ValueError('Gazebo effort target rate limit exceeds action scale')
    if gazebo['effort_settle_steps'] <= 0:
        raise ValueError('Gazebo effort settling duration must be positive')
    if not 0.0 < gazebo['position_proportional_gain'] <= 1.0:
        raise ValueError('Gazebo position proportional gain must be in (0, 1]')
    if not isinstance(gazebo['deterministic_stop_adapter'], bool):
        raise ValueError('Gazebo deterministic stop adapter flag must be bool')
    if not isinstance(gazebo['captured_effort_hold'], bool):
        raise ValueError('Gazebo captured effort hold flag must be bool')
    if gazebo['stop_capture_dwell_steps'] <= 0:
        raise ValueError('Gazebo stop capture dwell must be positive')
    if gazebo['motion_burst_steps'] <= 0:
        raise ValueError('Gazebo motion burst duration must be positive')
    if gazebo['burst_restart_dwell_steps'] <= 0:
        raise ValueError('Gazebo burst restart dwell must be positive')
    if not 0.0 < gazebo['burst_restart_minimum_up_z'] <= 1.0:
        raise ValueError('invalid Gazebo burst restart minimum up-z')
    if not 0.0 < gazebo['fall_minimum_up_z'] <= 1.0:
        raise ValueError('invalid Gazebo fall minimum up-z')

    stop = contract['stop_controller']
    for name in ('stop_residual_scale', 'stop_target_rate_limit'):
        values = stop[name]
        if len(values) != len(EXPECTED_LEG_JOINTS) or any(
                not math.isfinite(value) or value < 0.0
                for value in values):
            raise ValueError(f'invalid {name}')
    if any(value > 0.03 for value in stop['stop_target_rate_limit']):
        raise ValueError('V12 stop target rate limit exceeds 0.03 rad/step')
    for name in (
            'contact_force_threshold', 'stop_shoulder_rate_limit',
            'capture_planar_speed_threshold',
            'capture_yaw_speed_threshold',
            'capture_angular_velocity_xy_threshold',
            'capture_vertical_velocity_threshold',
            'recovery_planar_speed_threshold',
            'recovery_yaw_speed_threshold',
            'recovery_angular_velocity_xy_threshold',
            'recovery_vertical_velocity_threshold'):
        if not math.isfinite(stop[name]) or stop[name] <= 0.0:
            raise ValueError(f'invalid {name}')
    if stop['capture_dwell_steps'] <= 0 or stop['stop_age_scale_steps'] <= 0:
        raise ValueError('V12 stop step counts must be positive')
    if not -1.0 <= stop['recovery_minimum_up_z'] <= (
            stop['capture_minimum_up_z']) <= 1.0:
        raise ValueError('invalid V12 capture/recovery up-z thresholds')
    checkpoint = policy['checkpoint']
    if checkpoint['size'] <= 0 or len(checkpoint['sha256']) != 64:
        raise ValueError('invalid checkpoint identity')
    return contract


def file_identity(path):
    """Return byte size and SHA-256 for a file without modifying it."""
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return {'size': path.stat().st_size, 'sha256': digest.hexdigest()}


def verify_file_identity(specification):
    """Verify a file against the identity fields in a contract mapping."""
    actual = file_identity(specification['path'])
    if 'size' in specification and actual['size'] != specification['size']:
        raise ValueError(f"size mismatch for {specification['path']}")
    if actual['sha256'] != specification['sha256']:
        raise ValueError(f"SHA-256 mismatch for {specification['path']}")
    return actual


def build_observation(components, contract):
    """Assemble one finite 78-D observation in the frozen term order."""
    validate_contract(contract)
    observation = np.empty(
        contract['policy']['observation_dimension'], dtype=np.float32)
    for term in contract['observation_layout']:
        name = term['name']
        if name not in components:
            raise ValueError(f'missing observation component {name!r}')
        values = np.asarray(components[name], dtype=np.float32).reshape(-1)
        expected_size = term['end'] - term['start']
        if values.size != expected_size:
            raise ValueError(
                f'observation component {name!r} has size {values.size}, '
                f'expected {expected_size}')
        if not np.isfinite(values).all():
            raise ValueError(f'observation component {name!r} is non-finite')
        observation[term['start']:term['end']] = values
    return observation
