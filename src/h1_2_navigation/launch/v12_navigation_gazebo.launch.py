import os

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
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    physics_engine = LaunchConfiguration('physics_engine')
    movement_authorized = LaunchConfiguration('movement_authorized')
    upright_assistance = LaunchConfiguration('upright_assistance')
    start_localization = LaunchConfiguration('start_localization')
    start_navigation = LaunchConfiguration('start_navigation')
    startup_x = LaunchConfiguration('startup_x')
    startup_y = LaunchConfiguration('startup_y')
    startup_yaw = LaunchConfiguration('startup_yaw')
    static_obstacle_enabled = LaunchConfiguration(
        'static_obstacle_enabled')
    static_obstacle_minimum_x = LaunchConfiguration(
        'static_obstacle_minimum_x')
    static_obstacle_maximum_x = LaunchConfiguration(
        'static_obstacle_maximum_x')
    static_obstacle_minimum_y = LaunchConfiguration(
        'static_obstacle_minimum_y')
    static_obstacle_maximum_y = LaunchConfiguration(
        'static_obstacle_maximum_y')
    description_share = get_package_share_directory('h1_2_description')
    navigation_share = get_package_share_directory('h1_2_navigation')
    isaac_assets = '/home/liu/test/h1_2_isaaclab/assets'
    world_file = os.path.join(
        description_share, 'worlds', 'h1_world.sdf')
    robot_xacro = os.path.join(
        description_share, 'urdf', 'h1_2_navigation.urdf.xacro')
    contract_file = os.path.join(
        navigation_share, 'config', 'v12_locomotion.yaml')
    localizer_file = os.path.join(
        navigation_share, 'config', 'mid360_localization.yaml')
    amcl_file = os.path.join(
        navigation_share, 'config', 'nav2_amcl.yaml')
    planner_file = os.path.join(
        navigation_share, 'config', 'nav2_planner.yaml')
    control_file = os.path.join(
        navigation_share, 'config', 'nav2_control.yaml')
    navigation_tree = os.path.join(
        navigation_share, 'behavior_trees',
        'navigate_to_pose_low_speed.xml')
    policy_file = os.path.join(
        navigation_share, 'policy', 'v12_model_17250_policy.npz')
    upright_assistance_file = os.path.join(
        navigation_share, 'config', 'gazebo_upright_assistance.yaml')
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                '/usr/bin/python3 -m ',
                'h1_2_navigation.navigation_urdf_adapter',
                ' --source-xacro ', robot_xacro,
                ' --contract-file ', contract_file,
            ]),
            value_type=str,
        )
    }
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': [
                '-r -v 3 ',
                PythonExpression([
                    "'-s' if '", headless,
                    "'.lower() == 'true' else ''",
                ]),
                ' --physics-engine ', physics_engine,
                ' ', world_file,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'h1_2_navigation',
            '-x', startup_x,
            '-y', startup_y,
            '-z', '1.08912',
            '-Y', startup_yaw,
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/livox/mid360/points@'
            'sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/h1_2_navigation/imu@'
            'sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/h1_2_navigation/left_foot/wrench@'
            'geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/h1_2_navigation/right_foot/wrench@'
            'geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
            '/h1_2_navigation/ground_truth/odom@'
            'nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/h1_world/wrench@'
            'ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
        ],
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    standby_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'v12_standby_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    effort_pid_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'v12_effort_pid_controller',
            '--inactive',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    velocity_gate = Node(
        package='h1_2_navigation',
        executable='nav2_velocity_gate',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'contract_file': contract_file,
            'enabled': True,
            'movement_authorized': ParameterValue(
                movement_authorized, value_type=bool),
            'policy_ready': True,
            'state_estimate_ready': True,
            'contact_sensors_ready': True,
            'leg_controller_ready': True,
        }],
    )
    v12_controller = Node(
        package='h1_2_navigation',
        executable='v12_gazebo_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'contract_file': contract_file,
            'policy_file': policy_file,
            'enabled': True,
            'movement_authorized': ParameterValue(
                movement_authorized, value_type=bool),
        }],
    )
    upright_stabilizer = Node(
        package='h1_2_navigation',
        executable='gazebo_upright_stabilizer',
        output='screen',
        parameters=[upright_assistance_file, {
            'use_sim_time': use_sim_time,
            'movement_authorized': ParameterValue(
                movement_authorized, value_type=bool),
        }],
        condition=IfCondition(PythonExpression([
            "'", upright_assistance, "'.lower() == 'true' and '",
            movement_authorized, "'.lower() == 'true'",
        ])),
    )
    localizer = Node(
        package='h1_2_navigation',
        executable='mid360_square_localizer',
        output='screen',
        parameters=[localizer_file, {
            'publish_map_to_odom': True,
            'initial_x': ParameterValue(startup_x, value_type=float),
            'initial_y': ParameterValue(startup_y, value_type=float),
            'initial_yaw': ParameterValue(startup_yaw, value_type=float),
            'static_obstacle_enabled': ParameterValue(
                static_obstacle_enabled, value_type=bool),
            'static_obstacle_minimum_x': ParameterValue(
                static_obstacle_minimum_x, value_type=float),
            'static_obstacle_maximum_x': ParameterValue(
                static_obstacle_maximum_x, value_type=float),
            'static_obstacle_minimum_y': ParameterValue(
                static_obstacle_minimum_y, value_type=float),
            'static_obstacle_maximum_y': ParameterValue(
                static_obstacle_maximum_y, value_type=float),
        }],
        condition=IfCondition(start_localization),
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[amcl_file, {
            'tf_broadcast': False,
            'initial_pose.x': ParameterValue(startup_x, value_type=float),
            'initial_pose.y': ParameterValue(startup_y, value_type=float),
            'initial_pose.yaw': ParameterValue(
                startup_yaw, value_type=float),
        }],
        condition=IfCondition(start_localization),
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[amcl_file],
        condition=IfCondition(start_localization),
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[planner_file],
        condition=IfCondition(PythonExpression([
            "'", start_localization, "'.lower() == 'true' and '",
            start_navigation, "'.lower() == 'true'",
        ])),
    )
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[control_file],
        remappings=[('cmd_vel', '/cmd_vel_nav')],
        condition=IfCondition(PythonExpression([
            "'", start_localization, "'.lower() == 'true' and '",
            start_navigation, "'.lower() == 'true'",
        ])),
    )
    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[control_file],
        condition=IfCondition(PythonExpression([
            "'", start_localization, "'.lower() == 'true' and '",
            start_navigation, "'.lower() == 'true'",
        ])),
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[control_file, {
            'default_nav_to_pose_bt_xml': navigation_tree,
        }],
        condition=IfCondition(PythonExpression([
            "'", start_localization, "'.lower() == 'true' and '",
            start_navigation, "'.lower() == 'true'",
        ])),
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[control_file],
        condition=IfCondition(PythonExpression([
            "'", start_localization, "'.lower() == 'true' and '",
            start_navigation, "'.lower() == 'true'",
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

    start_joint_state = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=continue_or_shutdown(
            [joint_state_spawner], 'robot spawn'),
    ))
    start_position = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=continue_or_shutdown(
                [standby_position_spawner],
                'joint state broadcaster spawn'),
        ))
    load_effort_pid = RegisterEventHandler(OnProcessExit(
        target_action=standby_position_spawner,
        on_exit=continue_or_shutdown(
            [effort_pid_spawner], 'standby position controller spawn'),
    ))
    start_runtime = RegisterEventHandler(OnProcessExit(
        target_action=effort_pid_spawner,
        on_exit=continue_or_shutdown(
            [velocity_gate, v12_controller],
            'effort PID controller spawn'),
    ))
    stop_on_runtime_exit = RegisterEventHandler(OnProcessExit(
        target_action=v12_controller,
        on_exit=[EmitEvent(event=Shutdown(
            reason='V12 Gazebo controller exited'))],
    ))
    stop_on_nav_controller_exit = RegisterEventHandler(OnProcessExit(
        target_action=controller,
        on_exit=[EmitEvent(event=Shutdown(
            reason='Nav2 controller server exited'))],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server without its GUI'),
        DeclareLaunchArgument(
            'physics_engine', default_value='gz-physics-dartsim-plugin',
            description='Gazebo physics engine plugin'),
        DeclareLaunchArgument(
            'movement_authorized', default_value='false',
            description='Allow bounded nonzero V12 velocity commands'),
        DeclareLaunchArgument(
            'upright_assistance', default_value='false',
            description=(
                'Gazebo-only balance assistance with passive planar damping; '
                'requires movement_authorized=true')),
        DeclareLaunchArgument(
            'start_localization', default_value='true',
            description='Start Mid-360 localizer and AMCL'),
        DeclareLaunchArgument(
            'start_navigation', default_value='true',
            description=(
                'Start Nav2 planner, controller, behavior, and BT servers '
                'when localization is enabled')),
        DeclareLaunchArgument(
            'startup_x', default_value='0.0',
            description='Gazebo and localization startup X in map metres'),
        DeclareLaunchArgument(
            'startup_y', default_value='0.0',
            description='Gazebo and localization startup Y in map metres'),
        DeclareLaunchArgument(
            'startup_yaw', default_value='0.0',
            description='Gazebo and localization startup yaw in radians'),
        DeclareLaunchArgument(
            'static_obstacle_enabled', default_value='false',
            description='Add the configured workcell rectangle to /map'),
        DeclareLaunchArgument(
            'static_obstacle_minimum_x', default_value='0.20'),
        DeclareLaunchArgument(
            'static_obstacle_maximum_x', default_value='1.00'),
        DeclareLaunchArgument(
            'static_obstacle_minimum_y', default_value='-0.60'),
        DeclareLaunchArgument(
            'static_obstacle_maximum_y', default_value='0.60'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([
                os.path.dirname(isaac_assets),
                os.path.dirname(description_share),
            ]),
        ),
        robot_state_publisher,
        gz_sim,
        bridge,
        upright_stabilizer,
        spawn_robot,
        localizer,
        amcl,
        localization_lifecycle,
        planner,
        controller,
        behavior,
        bt_navigator,
        navigation_lifecycle,
        start_joint_state,
        start_position,
        load_effort_pid,
        start_runtime,
        stop_on_runtime_exit,
        stop_on_nav_controller_exit,
    ])
