from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from h1_2_navigation.locomotion_contract import EXPECTED_CONTROLLED_JOINTS
from h1_2_navigation.locomotion_contract import EXPECTED_LEG_JOINTS
from h1_2_navigation.unitree_h12_contract import load_contract
from h1_2_navigation.unitree_h12_contract import verify_file_identity
from h1_2_navigation.unitree_h12_contract import verify_source_assets
from h1_2_navigation.unitree_h12_urdf_adapter import (
    adapt_unitree_h12_urdf)
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
CONTRACT_PATH = PACKAGE_ROOT / 'config' / 'unitree_h12_locomotion.yaml'
CONTROLLERS_PATH = PACKAGE_ROOT / 'config' / 'unitree_h12_controllers.yaml'
DESCRIPTION_PATH = (
    WORKSPACE_ROOT / 'src' / 'h1_2_description' / 'urdf'
    / 'h1_2_navigation.urdf.xacro')
WORLD_PATH = PACKAGE_ROOT / 'worlds' / 'unitree_h12_world.sdf'
BASE_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'unitree_h12_navigation_gazebo.launch.py')
COMBINED_LAUNCH = (
    PACKAGE_ROOT / 'launch' / 'incubator_navigation_gazebo.launch.py')
CONTROLLER = (
    PACKAGE_ROOT / 'h1_2_navigation' / 'unitree_h12_gazebo_controller.py')
AUTO_NAVIGATOR = (
    PACKAGE_ROOT / 'h1_2_navigation' / 'incubator_auto_navigator.py')
SETUP = PACKAGE_ROOT / 'setup.py'


def _adapted_description():
    process = subprocess.run(
        [
            'xacro', str(DESCRIPTION_PATH),
            f'controller_parameters_file:={CONTROLLERS_PATH}',
        ], check=True, capture_output=True, text=True)
    contract = load_contract(CONTRACT_PATH)
    return ET.fromstring(adapt_unitree_h12_urdf(
        process.stdout, contract))


def test_contract_binds_official_sources_and_export():
    contract = load_contract(CONTRACT_PATH)
    assert len(verify_source_assets(contract)) == 8
    assert verify_file_identity(
        contract['policy']['export'],
        PACKAGE_ROOT / 'policy' / 'unitree_h12_motion_policy.npz')
    assert tuple(contract['leg_joints']) == EXPECTED_LEG_JOINTS
    assert tuple(
        contract['training_actuator']['controlled_joints']
    ) == EXPECTED_LEG_JOINTS
    assert contract['gazebo_runtime']['direct_position_targets'] is True
    assert contract['gazebo_runtime']['upper_body_mode'] == (
        'fixed_at_official_zero')
    assert contract['gazebo_runtime']['foot_collision_mode'] == (
        'official_mesh')
    assert contract['gazebo_runtime']['spawn_height'] == 1.03
    assert contract['gazebo_runtime']['physics_step_size'] == 0.002
    assert contract['zero_command'] == {
        'mode': 'recurrent_station_keeping',
        'preserve_recurrent_state': True,
        'preserve_gait_phase': True,
        'reset_recurrent_state_on_entry': True,
        'direct_position_targets': True,
        'position_gain': 2.0,
        'yaw_gain': 1.0,
        'linear_velocity_gain': 0.8,
        'yaw_rate_gain': 0.5,
        'policy_command_limits': [0.10, 0.08, 0.05],
    }
    assert contract['motion_tracking'] == {
        'mode': 'recurrent_pose_tracking',
        'feedforward_command': True,
        'position_gain': 2.0,
        'yaw_gain': 1.0,
        'maximum_position_error': 0.10,
        'maximum_yaw_error': 0.20,
        'policy_command_limits': [0.10, 0.08, 0.05],
    }
    assert 'frequency_scale' not in contract['gazebo_runtime']
    assert 'motion_burst_steps' not in contract['gazebo_runtime']
    assert 'effort_target_rate_limit' not in contract['gazebo_runtime']


def test_controller_yaml_uses_official_leg_pd_and_isolated_names():
    contract = load_contract(CONTRACT_PATH)
    controllers = yaml.safe_load(CONTROLLERS_PATH.read_text())
    parameters = controllers[
        'unitree_h12_effort_pid_controller']['ros__parameters']
    assert tuple(parameters['dof_names']) == EXPECTED_LEG_JOINTS
    assert tuple(controllers[
        'unitree_h12_standby_position_controller'
    ]['ros__parameters']['joints']) == EXPECTED_LEG_JOINTS
    for index, joint in enumerate(EXPECTED_LEG_JOINTS):
        gains = parameters['gains'][joint]
        assert gains['p'] == contract['training_actuator']['stiffness'][index]
        assert gains['d'] == contract['training_actuator']['damping'][index]
    assert controllers['controller_manager']['ros__parameters'][
        'update_rate'] == 500
    assert controllers['controller_manager']['ros__parameters'][
        'enforce_command_limits'] is True


def test_v13_controller_has_continuous_direct_policy_path():
    source = CONTROLLER.read_text(encoding='utf-8')

    assert 'UnitreeH12Runtime' in source
    assert 'output.leg_position_targets' in source
    assert "'STATION_KEEPING_GAIT'" in source
    assert '_station_keeping_command(' in source
    assert '_pose_tracking_command(' in source
    assert '_advance_motion_reference(' in source
    assert 'station_body_velocity' in source
    assert 'station_yaw_rate' in source
    assert 'station_entry_recurrent_reset' in source
    assert 'self.policy.reset()' in source
    assert "self.contract['motion_tracking']" in source
    assert "self.contract['zero_command']['mode']" in source
    assert 'def _stop_step' not in source
    assert 'if exact_zero or self.braking' not in source
    assert "'BRAKE'" not in source
    assert 'motion_burst_steps' not in source
    assert 'burst_restart_dwell_steps' not in source
    assert 'capture_targets' not in source
    assert 'frequency_scale' not in source
    assert 'MultiThreadedExecutor(num_threads=2)' in source
    assert 'output.header = message.header' not in source
    assert 'odom_transform.header = output.header' not in source
    assert 'pelvis_transform.header = output.header' not in source


def test_launch_uses_only_isolated_unitree_runtime_assets():
    launch = BASE_LAUNCH.read_text(encoding='utf-8')

    for expected in (
            'unitree_h12_locomotion.yaml',
            'unitree_h12_controllers.yaml',
            'unitree_h12_motion_policy.npz',
            'unitree_h12_world.sdf',
            'unitree_h12_urdf_adapter',
            'unitree_h12_velocity_gate',
            'unitree_h12_gazebo_controller',
            'unitree_h12_standby_position_controller',
            'unitree_h12_effort_pid_controller'):
        assert expected in launch
    assert "'movement_authorized', default_value='false'" in launch
    assert "'upright_assistance', default_value='false'" in launch
    assert "'start_localization', default_value='true'" in launch
    assert "'start_navigation', default_value='true'" in launch
    assert "'-z', '1.03'" in launch
    assert 'v12_model_17250_policy.npz' not in launch
    assert "reason='Unitree H1_2 Gazebo controller exited'" in launch
    assert "'FollowPath.desired_linear_vel': 0.10" in launch
    assert "'FollowPath.min_approach_linear_velocity': 0.05" in launch
    assert "'goal_checker.xy_goal_tolerance': 0.08" in launch
    assert "'FollowPath.use_rotate_to_heading': False" in launch
    assert 'delayed_navigation_lifecycle = TimerAction(' in launch
    assert 'actions=[navigation_lifecycle]' in launch
    assert (
        '[\n                velocity_gate,\n                unitree_controller,\n'
        in launch)
    assert launch.index('delayed_navigation_lifecycle = TimerAction(') < (
        launch.index('start_runtime = RegisterEventHandler('))


def test_adapted_robot_matches_official_12_dof_structure():
    root = _adapted_description()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    leg_joints = set(EXPECTED_LEG_JOINTS)
    upper_joints = set(EXPECTED_CONTROLLED_JOINTS) - leg_joints

    assert all(joints[name].get('type') == 'revolute' for name in leg_joints)
    assert all(joints[name].get('type') == 'fixed' for name in upper_joints)
    control = root.find("ros2_control[@name='H1_2NavigationGazeboSystem']")
    assert tuple(
        joint.get('name') for joint in control.findall('joint')
    ) == EXPECTED_LEG_JOINTS
    for side in ('left', 'right'):
        link_name = f'{side}_ankle_roll_link'
        collision = root.find(
            f"link[@name='{link_name}']/collision")
        assert collision.find('geometry/box') is None
        mesh = collision.find('geometry/mesh')
        assert mesh.get('filename').endswith(f'{link_name}.STL')
        assert collision.find('origin').get('xyz') == '0 0 0'
        assert root.find(
            f"link[@name='{side}_dex1_navigation_visual_link']"
        ) is not None


def test_unitree_world_preserves_scene_at_official_physics_step():
    world = ET.parse(WORLD_PATH).getroot().find('world')
    physics = world.find('physics')

    assert float(physics.find('max_step_size').text) == 0.002
    assert float(physics.find('real_time_update_rate').text) == 250.0
    assert float(physics.find('real_time_factor').text) == 0.5
    assert [model.get('name') for model in world.findall('model')] == [
        'ground_plane', 'navigation_enclosure']
    wall_sizes = [
        collision.find('geometry/box/size').text
        for collision in world.findall(
            "model[@name='navigation_enclosure']/link/collision")
    ]
    assert wall_sizes == [
        '5.2 0.1 2.5', '5.2 0.1 2.5',
        '0.1 5.0 2.5', '0.1 5.0 2.5']


def test_incubator_entry_defaults_to_v12_and_adds_unitree_option():
    launch = COMBINED_LAUNCH.read_text(encoding='utf-8')

    assert "'locomotion_profile', default_value='v12'" in launch
    assert "choices=['v12', 'unitree_h12']" in launch
    assert 'v12_navigation_gazebo.launch.py' in launch
    assert 'unitree_h12_navigation_gazebo.launch.py' in launch
    assert '/h1_2_navigation/v12/status' in launch
    assert '/h1_2_navigation/unitree_h12/status' in launch
    assert "'models', 'incubator', 'model.sdf'" in launch
    assert "'models', 'green_cube', 'model.sdf'" in launch
    assert "'models', 'table', 'model.sdf'" in launch


def test_auto_navigator_status_topic_is_backward_compatible():
    source = AUTO_NAVIGATOR.read_text(encoding='utf-8')

    assert "'locomotion_status_topic', '/h1_2_navigation/v12/status'" in source
    assert 'String, self.locomotion_status_topic' in source
    assert 'self._on_v12, latest_qos' in source


def test_setup_installs_both_new_runtime_executables():
    source = SETUP.read_text(encoding='utf-8')

    assert 'unitree_h12_velocity_gate:main' in source
    assert 'unitree_h12_gazebo_controller:main' in source
    assert "glob('worlds/*.sdf')" in source
