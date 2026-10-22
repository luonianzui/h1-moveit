from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from h1_2_navigation.navigation_urdf_adapter import adapt_urdf
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
DESCRIPTION = WORKSPACE_ROOT / 'src' / 'h1_2_description' / 'urdf' / (
    'h1_2_navigation.urdf.xacro')
CONTRACT = PACKAGE_ROOT / 'config' / 'v12_locomotion.yaml'
CONTROLLERS = PACKAGE_ROOT / 'config' / 'v12_controllers.yaml'
WORLD = WORKSPACE_ROOT / 'src' / 'h1_2_description' / 'worlds' / 'h1_world.sdf'
LAUNCH = PACKAGE_ROOT / 'launch' / 'v12_navigation_gazebo.launch.py'


def _expanded_description():
    result = subprocess.run(
        ['xacro', str(DESCRIPTION)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _adapted_description():
    contract = yaml.safe_load(CONTRACT.read_text())
    expanded = ET.tostring(_expanded_description(), encoding='unicode')
    return ET.fromstring(adapt_urdf(expanded, contract))


def test_navigation_description_is_free_root_and_feet_only():
    root = _expanded_description()
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    assert 'world' not in links
    assert 'world_joint' not in joints
    body_collisions = {
        name: len(link.findall('collision'))
        for name, link in links.items()
        if link.findall('collision')
    }
    assert body_collisions == {
        'left_ankle_roll_link': 1,
        'right_ankle_roll_link': 1,
        'livox_mid360_link': 1,
    }


def test_navigation_visual_grippers_do_not_change_training_dynamics():
    root = _expanded_description()
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}

    for side in ('left', 'right'):
        link_name = f'{side}_dex1_navigation_visual_link'
        joint_name = f'{side}_dex1_navigation_visual_joint'
        visual_link = links[link_name]
        visual_joint = joints[joint_name]

        assert len(visual_link.findall('visual')) == 8
        assert visual_link.find('collision') is None
        assert visual_link.find('inertial') is None
        assert visual_joint.get('type') == 'fixed'
        assert visual_joint.find('parent').get('link') == (
            f'{side}_wrist_yaw_link')
        assert visual_joint.find('child').get('link') == link_name

    control = root.find("ros2_control[@name='H1_2NavigationGazeboSystem']")
    controlled_names = {joint.get('name') for joint in control.findall('joint')}
    assert not any('dex1' in name for name in controlled_names)


def test_navigation_description_keeps_training_joint_initialization():
    root = _expanded_description()
    contract = yaml.safe_load(CONTRACT.read_text())
    control = root.find("ros2_control[@name='H1_2NavigationGazeboSystem']")
    assert control is not None
    interfaces = {joint.get('name'): joint for joint in control.findall('joint')}
    assert len(interfaces) == 27
    for name, expected in contract['training_default_positions'].items():
        state = interfaces[name].find("state_interface[@name='position']")
        initial = state.find("param[@name='initial_value']")
        assert float(initial.text) == expected
        command = interfaces[name].find("command_interface[@name='effort']")
        assert command is not None


def test_effort_pid_controller_matches_ros2_control_and_training():
    root = _expanded_description()
    control = root.find("ros2_control[@name='H1_2NavigationGazeboSystem']")
    interface_names = [joint.get('name') for joint in control.findall('joint')]
    controller = yaml.safe_load(CONTROLLERS.read_text())
    parameters = controller['v12_effort_pid_controller']['ros__parameters']
    configured_names = parameters['dof_names']
    assert configured_names == interface_names
    assert controller['v12_standby_position_controller'][
        'ros__parameters']['joints'] == interface_names
    assert controller['controller_manager']['ros__parameters'][
        'update_rate'] == 250
    assert controller['controller_manager']['ros__parameters'][
        'enforce_command_limits'] is True
    contract = yaml.safe_load(CONTRACT.read_text())
    stiffness_scale = contract['gazebo_runtime']['effort_stiffness_scale']
    damping_scale = contract['gazebo_runtime']['effort_damping_scale']
    gazebo_plugin = root.find(
        "gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    assert gazebo_plugin is not None
    assert float(gazebo_plugin.find('position_proportional_gain').text) == (
        contract['gazebo_runtime']['position_proportional_gain'])
    for index, name in enumerate(configured_names):
        gain = parameters['gains'][name]
        assert gain['p'] == pytest.approx(
            contract['training_actuator']['stiffness'][index]
            * stiffness_scale)
        assert gain['d'] == pytest.approx(
            contract['training_actuator']['damping'][index] * damping_scale)
        limit = contract['training_actuator']['effort_limits'][index]
        assert gain['u_clamp_min'] == -limit
        assert gain['u_clamp_max'] == limit


def test_navigation_adapter_applies_training_joint_dynamics():
    contract = yaml.safe_load(CONTRACT.read_text())
    root = _adapted_description()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    armature = (
        contract['training_actuator']['armature']
        * contract['gazebo_runtime']['armature_scale'])
    for joint_name in contract['training_actuator']['controlled_joints']:
        joint = joints[joint_name]
        dynamics = joint.find('dynamics')
        assert float(dynamics.get('damping')) == (
            contract['training_actuator']['viscous_friction'])
        assert float(dynamics.get('friction')) == (
            contract['training_actuator']['friction'])
        axis = [
            float(value) for value in joint.find('axis').get('xyz').split()
        ]
        inertia = links[joint.find('child').get('link')].find(
            './inertial/inertia')
        diagonal = [
            float(inertia.get('ixx')), float(inertia.get('iyy')),
            float(inertia.get('izz')),
        ]
        assert sum(value * component * component
                   for value, component in zip(diagonal, axis)) >= armature


def test_navigation_sensors_and_odometry_are_declared():
    root = _expanded_description()
    xml = ET.tostring(root, encoding='unicode')
    assert 'v12_imu' in xml
    assert 'left_foot_load' in xml
    assert 'right_foot_load' in xml
    assert 'left_foot_contact' in xml
    assert 'right_foot_contact' in xml
    assert '/h1_2_navigation/left_foot/contacts' in xml
    assert '<odom_publish_frequency>250</odom_publish_frequency>' in xml
    assert '/h1_2_navigation/right_foot/contacts' in xml
    assert 'gz-sim-odometry-publisher-system' in xml
    assert '/h1_2_navigation/ground_truth/odom' in xml


def test_v12_controller_drops_stale_sensor_backlog_and_holds_last_target():
    controller = (
        PACKAGE_ROOT / 'h1_2_navigation' / 'v12_gazebo_controller.py'
    ).read_text(encoding='utf-8')

    assert controller.count('qos_profile_sensor_data)') == 5
    assert 'abs(now - self.received[name]) <= timeout' in controller
    assert 'else self.last_commanded_targets' in controller
    assert 'self._publish_hold_targets(self.default_positions)' not in controller
    assert 'MultiThreadedExecutor(num_threads=2)' in controller
    assert "'GAZEBO_BRAKE'" in controller
    assert "'GAZEBO_HOLD'" in controller
    assert 'self.gazebo_stop_targets, self.default_positions, rates' in controller
    assert 'self._publish_hold_targets(self.gazebo_stop_targets)' in controller
    assert "rates, self.runtime_config['effort_target_rate_limit']" in controller
    assert "rates, self.runtime_config['position_stop_target_rate_limit']" in controller
    assert 'self.motion_arming.reaches_onset(self.command)' in controller
    assert 'self.position_hold_targets = self.positions.copy()' in controller
    assert "'ARMING_POSITION_HOLD'" in controller
    assert "'POSITION_HOLD_READY'" in controller
    assert "self.runtime_config['motion_burst_steps']" in controller
    assert "if output.mode == 'MOVE'" in controller
    assert 'output.mode == self.runtime.MOVE' not in controller
    assert "'motion_burst_planar_speed'" in controller
    assert "self.runtime_config['burst_restart_dwell_steps']" in controller
    assert 'requested_command=requested_command.tolist()' in controller
    assert "if 'odom' in self.received:" in controller
    assert 'payload.update(self._root_diagnostics())' in controller
    assert "'CAPTURED_HOLD'" in controller
    assert 'self.captured_effort_hold_targets[:] = targets' in controller
    assert "self.runtime_config['captured_hold_return_rate']" in controller
    assert 'canonical_target_ready' not in controller
    assert 'self.post_motion_position_hold_active' not in controller
    assert "'CAPTURED_TO_POSITION_HOLD'" not in controller
    assert controller.index('self._position_mode_ready()') < controller.index(
        'and not self._effort_mode_ready())')
    assert "'effort_target_rate_limit'" in controller
    assert "if output.mode == 'MOVE'" in controller
    assert "'stop_target_rate_limit'" in controller
    assert "'EFFORT_PID_SETTLING'" in controller
    assert controller.count(
        'callback_group=self.control_callback_group') == 2


def test_dynamic_contact_sensors_have_required_system_order():
    world = WORLD.read_text(encoding='utf-8')

    assert world.index('gz-sim-user-commands-system') < world.index(
        'gz-sim-contact-system')
    assert 'gz-sim-apply-link-wrench-system' in world


def test_navigation_world_matches_training_physics_period():
    world = ET.parse(WORLD).getroot()
    physics = world.find('./world/physics')

    assert float(physics.find('max_step_size').text) == 0.004
    assert float(physics.find('real_time_update_rate').text) == 125.0
    assert float(physics.find('real_time_factor').text) == 0.5


def test_navigation_launch_selects_physics_engine_explicitly():
    launch = LAUNCH.read_text(encoding='utf-8')

    assert "LaunchConfiguration('physics_engine')" in launch
    assert "' --physics-engine ', physics_engine" in launch
    assert "default_value='gz-physics-dartsim-plugin'" in launch


def test_upright_assistance_is_explicit_and_fail_closed():
    launch = LAUNCH.read_text(encoding='utf-8')
    stabilizer = (
        PACKAGE_ROOT / 'h1_2_navigation' / 'gazebo_upright_stabilizer.py'
    ).read_text(encoding='utf-8')
    assistance = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'gazebo_upright_assistance.yaml').read_text(
            encoding='utf-8'))['h1_2_gazebo_upright_stabilizer'][
                'ros__parameters']

    assert "LaunchConfiguration('upright_assistance')" in launch
    assert "'upright_assistance', default_value='false'" in launch
    assert "'movement_authorized': ParameterValue(" in launch
    assert 'ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench' in launch
    assert "reason='Nav2 controller server exited'" in launch
    assert assistance['planar_damping'] == 300.0
    assert assistance['captured_planar_damping'] == 900.0
    assert assistance['v12_status_topic'] == '/h1_2_navigation/v12/status'
    assert "if state == 'CAPTURED_HOLD':" in stabilizer
    assert "'MOVE'," in stabilizer
    assert 'self.captured_hold_damping_active = False' in stabilizer
    assert 'self.captured_config' in stabilizer
