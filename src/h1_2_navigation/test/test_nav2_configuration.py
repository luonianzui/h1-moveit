from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LOCALIZER = (
    PACKAGE_ROOT / 'h1_2_navigation' / 'mid360_square_localizer.py')


def _parameters(file_name, node_name):
    with (PACKAGE_ROOT / 'config' / file_name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)[node_name]['ros__parameters']


def _nested_parameters(file_name, namespace, node_name):
    with (PACKAGE_ROOT / 'config' / file_name).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)[namespace][node_name]['ros__parameters']


def test_amcl_owns_map_to_odom_for_fixed_localization():
    parameters = _parameters('nav2_amcl.yaml', 'amcl')

    assert parameters['global_frame_id'] == 'map'
    assert parameters['odom_frame_id'] == 'odom'
    assert parameters['base_frame_id'] == 'base_footprint'
    assert parameters['map_topic'] == '/map'
    assert parameters['scan_topic'] == '/scan'
    assert parameters['tf_broadcast'] is True
    assert parameters['first_map_only'] is True
    assert parameters['transform_tolerance'] == 1.0


def test_amcl_starts_with_known_square_room_pose():
    parameters = _parameters('nav2_amcl.yaml', 'amcl')

    assert parameters['set_initial_pose'] is True
    assert parameters['always_reset_initial_pose'] is True
    assert parameters['initial_pose'] == {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'yaw': 0.0,
    }


def test_lifecycle_manager_only_controls_amcl():
    parameters = _parameters(
        'nav2_amcl.yaml', 'lifecycle_manager_localization')

    assert parameters['autostart'] is True
    assert parameters['node_names'] == ['amcl']


def test_default_scan_contract_remains_backward_compatible():
    parameters = _parameters(
        'mid360_localization.yaml', 'h1_2_mid360_square_localizer')

    assert parameters['base_frame'] == 'pelvis'
    assert parameters['scan_frame'] == 'pelvis'
    assert parameters['minimum_height'] == -0.35
    assert parameters['maximum_height'] == 1.25
    assert parameters['scan_minimum_range'] == 0.10
    assert parameters['minimum_range'] == 0.80
    assert parameters['localization_candidate_band'] == 0.50
    assert parameters['static_obstacle_enabled'] is False
    assert parameters['transform_timeout'] == 1.0
    assert parameters['publish_map_to_odom'] is False
    assert parameters['odom_frame'] == 'odom'
    assert parameters['map_to_odom_publish_rate'] == 50.0
    assert parameters['map_to_odom_transform_tolerance'] == 2.0


def test_scan_publication_uses_the_near_obstacle_range_gate():
    source = LOCALIZER.read_text(encoding='utf-8')
    publish_scan = source[source.index('    def _publish_scan('):]

    assert 'self.scan_minimum_range,' in publish_scan
    assert 'message.range_min = self.scan_minimum_range' in publish_scan


def test_planner_uses_navfn_without_unknown_space():
    parameters = _parameters('nav2_planner.yaml', 'planner_server')

    assert parameters['planner_plugins'] == ['GridBased']
    assert parameters['GridBased'] == {
        'plugin': 'nav2_navfn_planner::NavfnPlanner',
        'tolerance': 0.10,
        'use_astar': True,
        'allow_unknown': False,
    }


def test_global_costmap_uses_localization_frames_and_scan():
    parameters = _nested_parameters(
        'nav2_planner.yaml', 'global_costmap', 'global_costmap')

    assert parameters['global_frame'] == 'map'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert parameters['track_unknown_space'] is False
    assert parameters['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]
    assert parameters['static_layer']['map_subscribe_transient_local'] is True
    assert parameters['obstacle_layer']['observation_sources'] == 'scan'
    assert parameters['obstacle_layer']['scan']['topic'] == '/scan'
    assert parameters['obstacle_layer']['scan']['obstacle_min_range'] == 0.10
    assert parameters['obstacle_layer']['scan']['raytrace_min_range'] == 0.10
    assert parameters['inflation_layer']['inflation_radius'] == 0.50


def test_navigation_lifecycle_controls_complete_motion_chain():
    parameters = _parameters(
        'nav2_control.yaml', 'lifecycle_manager_navigation')

    assert parameters['autostart'] is True
    assert parameters['node_names'] == [
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]


def test_controller_is_low_speed_regulated_pure_pursuit():
    parameters = _parameters('nav2_control.yaml', 'controller_server')
    plugin = parameters['FollowPath']

    assert parameters['controller_frequency'] == 10.0
    assert parameters['controller_plugins'] == ['FollowPath']
    assert parameters['goal_checker']['xy_goal_tolerance'] == 0.04
    assert plugin['plugin'].endswith('RegulatedPurePursuitController')
    assert plugin['desired_linear_vel'] == 0.05
    assert plugin['min_approach_linear_velocity'] == 0.03
    assert plugin['rotate_to_heading_angular_vel'] == 0.05
    assert plugin['max_angular_accel'] == 0.40
    assert plugin['allow_reversing'] is False


def test_local_costmap_uses_planar_base_and_mid360_scan():
    parameters = _nested_parameters(
        'nav2_control.yaml', 'local_costmap', 'local_costmap')

    assert parameters['global_frame'] == 'odom'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert parameters['obstacle_layer']['scan']['topic'] == '/scan'
    assert parameters['obstacle_layer']['scan']['data_type'] == 'LaserScan'
    assert parameters['obstacle_layer']['scan']['obstacle_min_range'] == 0.10
    assert parameters['obstacle_layer']['scan']['raytrace_min_range'] == 0.10


def test_bt_navigator_is_bound_to_planar_navigation_frames():
    parameters = _parameters('nav2_control.yaml', 'bt_navigator')

    assert parameters['global_frame'] == 'map'
    assert parameters['robot_base_frame'] == 'base_footprint'
    assert parameters['odom_topic'] == '/odom'
    assert parameters['navigators'] == ['navigate_to_pose']
