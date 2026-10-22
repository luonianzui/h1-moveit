import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    movement_authorized = LaunchConfiguration('movement_authorized')
    auto_navigate = LaunchConfiguration('auto_navigate')
    upright_assistance = LaunchConfiguration('upright_assistance')
    locomotion_profile = LaunchConfiguration('locomotion_profile')
    headless = LaunchConfiguration('headless')
    startup_x = '-0.60'
    startup_y = '0.0'
    startup_yaw = '0.0'
    goal_x = '-0.45'
    goal_y = '0.0'
    goal_yaw = '0.0'
    description_share = get_package_share_directory('h1_2_description')
    navigation_share = get_package_share_directory('h1_2_navigation')
    demo_prefix = get_package_prefix('h1_2_moveit_demo')
    existing_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = os.path.join(demo_prefix, 'lib', 'h1_2_moveit_demo')
    system_plugin_path = os.pathsep.join(
        path for path in (plugin_path, existing_plugins) if path)

    v12_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_share, 'launch', 'v12_navigation_gazebo.launch.py')),
        launch_arguments={
            'headless': headless,
            'movement_authorized': movement_authorized,
            'upright_assistance': upright_assistance,
            'startup_x': startup_x,
            'startup_y': startup_y,
            'startup_yaw': startup_yaw,
            'static_obstacle_enabled': 'true',
            'static_obstacle_minimum_x': '0.20',
            'static_obstacle_maximum_x': '1.00',
            'static_obstacle_minimum_y': '-0.60',
            'static_obstacle_maximum_y': '0.60',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", locomotion_profile, "'.lower() == 'v12'",
        ])),
    )
    unitree_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_share, 'launch',
            'unitree_h12_navigation_gazebo.launch.py')),
        launch_arguments={
            'headless': headless,
            'movement_authorized': movement_authorized,
            'upright_assistance': upright_assistance,
            'startup_x': startup_x,
            'startup_y': startup_y,
            'startup_yaw': startup_yaw,
            'static_obstacle_enabled': 'true',
            'static_obstacle_minimum_x': '0.20',
            'static_obstacle_maximum_x': '1.00',
            'static_obstacle_minimum_y': '-0.60',
            'static_obstacle_maximum_y': '0.60',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", locomotion_profile, "'.lower() == 'unitree_h12'",
        ])),
    )
    table = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'h1_world',
            '-file', os.path.join(
                description_share, 'models', 'table', 'model.sdf'),
            '-name', 'table',
            '-x', '0.600', '-y', '0.0', '-z', '0.0',
        ],
    )
    cube = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'h1_world',
            '-file', os.path.join(
                description_share, 'models', 'green_cube', 'model.sdf'),
            '-name', 'green_cube',
            '-x', '0.334', '-y', '0.377', '-z', '0.830',
            '-Y', '1.5707963267948966',
        ],
    )
    incubator = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'h1_world',
            '-file', os.path.join(
                description_share, 'models', 'incubator', 'model.sdf'),
            '-name', 'incubator',
            '-x', '0.590', '-y', '0.0', '-z', '0.815',
        ],
    )
    auto_navigator_v12 = Node(
        package='h1_2_navigation',
        executable='incubator_auto_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enabled': True,
            'movement_authorized': ParameterValue(
                movement_authorized, value_type=bool),
            'startup_x': float(startup_x),
            'startup_y': float(startup_y),
            'startup_yaw': float(startup_yaw),
            'goal_x': float(goal_x),
            'goal_y': float(goal_y),
            'goal_yaw': float(goal_yaw),
            'locomotion_status_topic': '/h1_2_navigation/v12/status',
        }],
        condition=IfCondition(PythonExpression([
            "'", auto_navigate, "'.lower() == 'true' and '",
            locomotion_profile, "'.lower() == 'v12'",
        ])),
    )
    auto_navigator_unitree = Node(
        package='h1_2_navigation',
        executable='incubator_auto_navigator',
        name='h1_2_incubator_auto_navigator_unitree_h12',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'enabled': True,
            'movement_authorized': ParameterValue(
                movement_authorized, value_type=bool),
            'startup_x': float(startup_x),
            'startup_y': float(startup_y),
            'startup_yaw': float(startup_yaw),
            'goal_x': float(goal_x),
            'goal_y': float(goal_y),
            'goal_yaw': float(goal_yaw),
            'locomotion_status_topic': (
                '/h1_2_navigation/unitree_h12/status'),
        }],
        condition=IfCondition(PythonExpression([
            "'", auto_navigate, "'.lower() == 'true' and '",
            locomotion_profile, "'.lower() == 'unitree_h12'",
        ])),
    )

    def continue_or_shutdown(next_actions, label):
        def on_exit(event, context):
            del context
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(
                    reason=f'{label} failed with code {event.returncode}'))]
            return next_actions
        return on_exit

    start_cube = RegisterEventHandler(OnProcessExit(
        target_action=table,
        on_exit=continue_or_shutdown([cube], 'table spawn'),
    ))
    start_incubator = RegisterEventHandler(OnProcessExit(
        target_action=cube,
        on_exit=continue_or_shutdown([incubator], 'cube spawn'),
    ))
    start_auto_navigation = RegisterEventHandler(OnProcessExit(
        target_action=incubator,
        on_exit=continue_or_shutdown(
            [auto_navigator_v12, auto_navigator_unitree],
            'incubator spawn'),
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server without its GUI'),
        DeclareLaunchArgument(
            'locomotion_profile', default_value='v12',
            choices=['v12', 'unitree_h12'],
            description='Select the isolated locomotion implementation'),
        DeclareLaunchArgument(
            'movement_authorized', default_value='false',
            description='Allow bounded Gazebo locomotion'),
        DeclareLaunchArgument(
            'upright_assistance', default_value='false',
            description='Enable Gazebo-only upright assistance'),
        DeclareLaunchArgument(
            'auto_navigate', default_value='false',
            description='Automatically plan and execute the staging goal'),
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', system_plugin_path),
        v12_navigation,
        unitree_navigation,
        table,
        start_cube,
        start_incubator,
        start_auto_navigation,
    ])
