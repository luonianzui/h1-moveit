"""Validation helpers for the isolated Unitree H1_2 locomotion profile."""

import hashlib
import math
from pathlib import Path

import numpy as np
import yaml

from .locomotion_contract import EXPECTED_GATES
from .locomotion_contract import EXPECTED_LEG_JOINTS


EXPECTED_OBSERVATION_LAYOUT = (
    ('base_angular_velocity', 0, 3),
    ('projected_gravity', 3, 6),
    ('velocity_command', 6, 9),
    ('joint_position', 9, 21),
    ('joint_velocity', 21, 33),
    ('previous_action', 33, 45),
    ('gait_phase', 45, 47),
)


def file_identity(path):
    """Return byte size and SHA-256 for one file."""
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'size': path.stat().st_size, 'sha256': digest.hexdigest()}


def verify_file_identity(specification, path=None):
    """Verify an explicit path against a contract identity."""
    target = specification['path'] if path is None else path
    actual = file_identity(target)
    expected = {
        'size': int(specification['size']),
        'sha256': str(specification['sha256']),
    }
    if actual != expected:
        raise ValueError(f'file identity mismatch for {target}')
    return actual


def load_contract(path):
    """Load and validate the Unitree H1_2 deployment contract."""
    with Path(path).open(encoding='utf-8') as stream:
        contract = yaml.safe_load(stream)
    validate_contract(contract)
    return contract


def _validate_identity(specification, name):
    if int(specification['size']) <= 0:
        raise ValueError(f'invalid {name} size')
    digest = str(specification['sha256'])
    if len(digest) != 64 or any(
            character not in '0123456789abcdef' for character in digest):
        raise ValueError(f'invalid {name} SHA-256')


def validate_contract(contract):
    """Reject interface drift from the reviewed official deployment."""
    if contract.get('schema_version') != 1:
        raise ValueError('unsupported Unitree H1_2 contract schema')
    deployment = contract['deployment']
    if deployment['enabled'] is not False:
        raise ValueError('locomotion must default to disabled')
    if deployment['movement_authorized'] is not False:
        raise ValueError('movement authorization must default to false')
    if tuple(deployment['required_gates']) != EXPECTED_GATES:
        raise ValueError('required locomotion gates changed')

    policy = contract['policy']
    if policy['observation_dimension'] != 47:
        raise ValueError('Unitree H1_2 observation dimension must be 47')
    if policy['action_dimension'] != 12:
        raise ValueError('Unitree H1_2 action dimension must be 12')
    if policy['recurrent_type'] != 'lstm':
        raise ValueError('Unitree H1_2 recurrent type must be LSTM')
    if policy['recurrent_hidden_dimension'] != 64:
        raise ValueError('Unitree H1_2 recurrent hidden size must be 64')
    if not math.isclose(policy['gait_period'], 0.8):
        raise ValueError('Unitree H1_2 gait period must be 0.8 seconds')
    if not math.isclose(policy['action_clip'], 100.0):
        raise ValueError('Unitree H1_2 action clip must remain 100')
    layout = tuple(
        (term['name'], term['start'], term['end'])
        for term in contract['observation_layout'])
    if layout != EXPECTED_OBSERVATION_LAYOUT:
        raise ValueError('Unitree H1_2 observation layout changed')

    control = contract['control']
    if not math.isclose(control['control_period'], 0.02):
        raise ValueError('Unitree H1_2 control period must be 0.02 seconds')
    if not math.isclose(control['action_scale'], 0.25):
        raise ValueError('Unitree H1_2 action scale must be 0.25')
    expected_scales = {
        'angular_velocity': (0.25, 0.25, 0.25),
        'joint_position': (1.0,) * 12,
        'joint_velocity': (0.05,) * 12,
        'velocity_command': (2.0, 2.0, 0.25),
    }
    for name, expected in expected_scales.items():
        values = tuple(float(value) for value in control['scales'][name])
        if values != expected:
            raise ValueError(f'Unitree H1_2 {name} scales changed')
    if len(control['command_deadband']) != 3 or any(
            value < 0.0 for value in control['command_deadband']):
        raise ValueError('invalid Unitree H1_2 command deadband')
    onset = control['motion_onset_threshold']
    if len(onset) != 3 or any(value <= 0.0 for value in onset):
        raise ValueError('invalid Unitree H1_2 motion onset threshold')
    if any(
            threshold <= deadband
            for threshold, deadband in zip(
                onset, control['command_deadband'])):
        raise ValueError('motion onset must exceed the command deadband')
    if control['command_timeout'] <= 0.0:
        raise ValueError('command timeout must be positive')
    for axis in ('vx', 'vy', 'yaw_rate'):
        training = control['training_limits'][axis]
        safety = control['safety_limits'][axis]
        if len(training) != 2 or len(safety) != 2:
            raise ValueError(f'invalid {axis} limits')
        if training[0] >= training[1] or safety[0] >= safety[1]:
            raise ValueError(f'unordered {axis} limits')
        if safety[0] < training[0] or safety[1] > training[1]:
            raise ValueError(f'{axis} safety limits exceed training limits')
    for name in ('linear_acceleration', 'angular_acceleration'):
        if control['safety_limits'][name] <= 0.0:
            raise ValueError(f'{name} must be positive')

    if tuple(contract['leg_joints']) != EXPECTED_LEG_JOINTS:
        raise ValueError('Unitree H1_2 leg joint order changed')
    defaults = contract['training_default_positions']
    limits = contract['joint_position_limits']
    if tuple(defaults) != EXPECTED_LEG_JOINTS:
        raise ValueError('Unitree H1_2 default position order changed')
    if tuple(limits) != EXPECTED_LEG_JOINTS:
        raise ValueError('Unitree H1_2 joint limit order changed')
    if any(
            len(limits[name]) != 2 or limits[name][0] >= limits[name][1]
            for name in EXPECTED_LEG_JOINTS):
        raise ValueError('invalid Unitree H1_2 joint position limits')

    actuator = contract['training_actuator']
    if tuple(actuator['controlled_joints']) != EXPECTED_LEG_JOINTS:
        raise ValueError('Unitree H1_2 controlled joint order changed')
    count = len(EXPECTED_LEG_JOINTS)
    for name in ('default_positions', 'stiffness', 'damping', 'effort_limits'):
        values = actuator[name]
        if len(values) != count or any(
                not math.isfinite(value) for value in values):
            raise ValueError(f'invalid Unitree H1_2 actuator {name}')
    if any(value <= 0.0 for value in actuator['stiffness']):
        raise ValueError('actuator stiffness must be positive')
    if any(value <= 0.0 for value in actuator['damping']):
        raise ValueError('actuator damping must be positive')
    if any(value <= 0.0 for value in actuator['effort_limits']):
        raise ValueError('actuator effort limits must be positive')
    expected_leg_stiffness = (
        200.0, 200.0, 200.0, 300.0, 40.0, 40.0,
        200.0, 200.0, 200.0, 300.0, 40.0, 40.0,
    )
    expected_leg_damping = (
        2.5, 2.5, 2.5, 4.0, 2.0, 2.0,
        2.5, 2.5, 2.5, 4.0, 2.0, 2.0,
    )
    if tuple(actuator['stiffness'][:12]) != expected_leg_stiffness:
        raise ValueError('official Unitree H1_2 leg stiffness changed')
    if tuple(actuator['damping'][:12]) != expected_leg_damping:
        raise ValueError('official Unitree H1_2 leg damping changed')
    for name in ('armature', 'friction', 'viscous_friction'):
        if not math.isfinite(actuator[name]) or actuator[name] < 0.0:
            raise ValueError(f'invalid actuator {name}')

    runtime = contract['gazebo_runtime']
    if runtime['direct_position_targets'] is not True:
        raise ValueError('Unitree H1_2 movement targets must remain direct')
    if runtime['upper_body_mode'] != 'fixed_at_official_zero':
        raise ValueError('Unitree H1_2 upper body must match 12-DOF model')
    if runtime['foot_collision_mode'] != 'official_mesh':
        raise ValueError('Unitree H1_2 foot collision must use official mesh')
    if not math.isclose(runtime['spawn_height'], 1.03):
        raise ValueError('Unitree H1_2 spawn height must match official model')
    if not math.isclose(runtime['physics_step_size'], 0.002):
        raise ValueError('Unitree H1_2 physics step must match deployment')
    if runtime['standby_controller_name'] == runtime['effort_controller_name']:
        raise ValueError('standby and effort controllers must differ')
    for name in (
            'load_contact_threshold', 'state_timeout', 'switch_timeout',
            'fall_minimum_up_z', 'fall_minimum_height'):
        if not math.isfinite(runtime[name]) or runtime[name] <= 0.0:
            raise ValueError(f'invalid Gazebo runtime {name}')
    if runtime['effort_settle_steps'] <= 0:
        raise ValueError('effort settle steps must be positive')
    zero_command = contract['zero_command']
    required_zero_command = {
        'mode': 'recurrent_station_keeping',
        'preserve_recurrent_state': True,
        'preserve_gait_phase': True,
        'reset_recurrent_state_on_entry': True,
        'direct_position_targets': True,
    }
    if any(zero_command.get(name) != value
           for name, value in required_zero_command.items()):
        raise ValueError('zero command transition semantics changed')
    for name in (
            'position_gain', 'yaw_gain',
            'linear_velocity_gain', 'yaw_rate_gain'):
        if (not math.isfinite(zero_command[name])
                or zero_command[name] <= 0.0):
            raise ValueError(f'invalid zero command {name}')
    station_limits = zero_command['policy_command_limits']
    safety = control['safety_limits']
    safety_maximum = (
        safety['vx'][1], safety['vy'][1], safety['yaw_rate'][1])
    if (
            len(station_limits) != 3
            or any(not math.isfinite(value) or value <= 0.0
                   for value in station_limits)
            or any(value > maximum for value, maximum in zip(
            station_limits, safety_maximum))):
        raise ValueError('invalid zero command policy limits')
    motion_tracking = contract['motion_tracking']
    if (
            motion_tracking.get('mode') != 'recurrent_pose_tracking'
            or motion_tracking.get('feedforward_command') is not True):
        raise ValueError('invalid Unitree H1_2 motion tracking mode')
    for name in (
            'position_gain', 'yaw_gain',
            'maximum_position_error', 'maximum_yaw_error'):
        if (
                not math.isfinite(motion_tracking[name])
                or motion_tracking[name] <= 0.0):
            raise ValueError(f'invalid motion tracking {name}')
    tracking_limits = motion_tracking['policy_command_limits']
    if (
            len(tracking_limits) != 3
            or any(not math.isfinite(value) or value <= 0.0
                   for value in tracking_limits)
            or any(value > maximum for value, maximum in zip(
                tracking_limits, safety_maximum))):
        raise ValueError('invalid motion tracking policy limits')

    for name, specification in contract['source_assets'].items():
        if not specification.get('path'):
            raise ValueError(f'missing source asset path for {name}')
        _validate_identity(specification, name)
    _validate_identity(policy['export'], 'policy export')
    _validate_identity(policy['golden'], 'policy golden corpus')
    return contract


def verify_source_assets(contract):
    """Verify every reviewed upstream source without writing files."""
    return {
        name: verify_file_identity(specification)
        for name, specification in contract['source_assets'].items()
    }


def build_observation(components, contract):
    """Assemble one finite 47-D observation in official term order."""
    validate_contract(contract)
    observation = np.empty(47, dtype=np.float32)
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
