import hashlib
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory


TELEMETRY_BRIDGE_CONFIG_SHA256 = (
    '615231d3fc7d810f38165cbc679e4044263d4000cace8b56cc0ed35705ec9b0d')
MID360_LOCALIZATION_CONFIG_SHA256 = (
    '4786feb493efc8a9fee306ff90d247320b42ad44823826701d247a829f7cd27f')


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_launch_file_sha256(context, *, path, expected_sha256, label):
    resolved_path = context.perform_substitution(path)
    expected = context.perform_substitution(expected_sha256)
    actual = _sha256_file(resolved_path)
    if actual != expected:
        raise RuntimeError(
            f'{label} SHA-256 mismatch: expected {expected}, got {actual}')
    return []


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    spawn_incubator = LaunchConfiguration('spawn_incubator')
    table_x = LaunchConfiguration('table_x')
    cube_x = LaunchConfiguration('cube_x')
    incubator_x = LaunchConfiguration('incubator_x')
    incubator_y = LaunchConfiguration('incubator_y')
    incubator_z = LaunchConfiguration('incubator_z')
    bridge_support_states = LaunchConfiguration('bridge_support_states')
    robot_joint_state_relay_source = LaunchConfiguration(
        'robot_joint_state_relay_source')
    robot_joint_state_relay_sha256 = LaunchConfiguration(
        'robot_joint_state_relay_sha256')
    telemetry_bridge_config = LaunchConfiguration(
        'telemetry_bridge_config')
    telemetry_bridge_config_sha256 = LaunchConfiguration(
        'telemetry_bridge_config_sha256')

    # 获取包路径
    h1_2_description_path = FindPackageShare('h1_2_description')
    h1_2_moveit_config_path = FindPackageShare('h1_2_moveit_config')
    h1_2_description_abs_path = get_package_share_directory('h1_2_description')
    h1_2_moveit_demo_prefix = get_package_prefix('h1_2_moveit_demo')
    robot_joint_state_relay_default = os.path.join(
        h1_2_moveit_demo_prefix, 'lib', 'h1_2_moveit_demo',
        'h1_robot_joint_state_relay_829.py')
    robot_joint_state_relay_default_sha256 = _sha256_file(
        robot_joint_state_relay_default)
    telemetry_bridge_config_default = os.path.join(
        get_package_share_directory('h1_2_moveit_config'), 'config',
        'candidate3_telemetry_bridge_829.yaml')
    telemetry_bridge_config_default_sha256 = _sha256_file(
        telemetry_bridge_config_default)
    if telemetry_bridge_config_default_sha256 != TELEMETRY_BRIDGE_CONFIG_SHA256:
        raise RuntimeError('installed candidate-3 telemetry bridge config changed')
    realsense_description_abs_path = get_package_share_directory('realsense2_description')
    authority_root = os.path.dirname(os.path.realpath(__file__))
    mid360_localization_config = os.path.join(
        authority_root, 'mid360_localization.yaml')
    if (_sha256_file(mid360_localization_config) !=
            MID360_LOCALIZATION_CONFIG_SHA256):
        raise RuntimeError(
            'sealed Mid360 localization config changed')

    # 设置 Gazebo 资源路径，解析 H1-2 与标准 D435 的 model:// 网格引用。
    # 路径必须指向各包 share 目录的父目录。
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.dirname(h1_2_description_abs_path),
            os.path.dirname(realsense_description_abs_path),
        ])
    )

    # The articulated incubator uses a model-local latch system from the demo
    # package. Preserve any pre-existing Gazebo system-plugin search path.
    gz_plugin_paths = [
        os.path.join(h1_2_moveit_demo_prefix, 'lib', 'h1_2_moveit_demo'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
    ]
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.pathsep.join(path for path in gz_plugin_paths if path)
    )

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([h1_2_description_path, 'urdf', 'h1_2_gazebo.urdf.xacro'])
    ])

    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Fast-831 uses its own non-overwriting, 5 ms physics authority.
    world_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 'h1_world.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )

    # 生成机器人到 Gazebo（降低高度，让脚着地）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'h1_2',
            '-z', '0.0'  # 改为0，让机器人脚着地
        ],
        output='screen'
    )

    # 生成桌子
    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([h1_2_description_path, 'models', 'table', 'model.sdf']),
            '-name', 'table',
            '-x', table_x,
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Spawn the 80 x 160 x 30 mm green block directly below the neutral left
    # hand.  Its +90 degree yaw aligns the 80 mm pinch axis with the left Dex1
    # and leaves a 75 mm end-grasp offset, so the wrist can reach the original
    # middle-rack centre.  Keep this as the persistent default for subsequent
    # Gazebo runs; the nearest incubator body point is still more than 60 mm
    # away.
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([h1_2_description_path, 'models', 'green_cube', 'model.sdf']),
            '-name', 'green_cube',
            '-x', cube_x,  # kept below the neutral left TCP on the shifted table
            '-y', '0.377',  # neutral left TCP y; positive Y is the left side
            '-z', '0.830',  # table top 0.815 + block half-height 0.015
            '-Y', '1.5707963267948966'  # yaw +90 deg for the shared pickup/rack IK branch
        ],
        output='screen'
    )

    # Optional imported laboratory incubator.  Its local bottom is Z=0 and its
    # front faces -X, so these defaults put it on the rear half of the table,
    # facing the robot without overlapping the handover cube.
    spawn_incubator_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([
                h1_2_description_path, 'models', 'incubator', 'model.sdf'
            ]),
            '-name', 'incubator',
            '-x', incubator_x,
            '-y', incubator_y,
            '-z', incubator_z,
        ],
        output='screen',
        condition=IfCondition(spawn_incubator),
    )

    # Bridge - 图像话题
    gz_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/realsense_camera/color',
            '/realsense_camera/depth'
        ],
        remappings=[
            ('/realsense_camera/color', '/realsense_camera/color/image'),
            ('/realsense_camera/depth', '/realsense_camera/depth/image')
        ],
        parameters=[{'qos': 'sensor_data'}],
        output='screen'
    )

    # Bridge - 其他话题
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/realsense_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/realsense_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/incubator/latch/state@std_msgs/msg/String[gz.msgs.StringMsg',
        ],
        remappings=[
            ('/realsense_camera/camera_info', '/realsense_camera/depth/camera_info'),
        ],
        output='screen'
    )

    # Keep the completed Mid360 square-map perception active in the sealed
    # manipulation scene. These nodes are observational only and publish no
    # trajectory, support, door, gripper, or Gazebo command.
    mid360_point_cloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/livox/mid360/points@'
            'sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )
    mid360_localizer = Node(
        package='h1_2_navigation',
        executable='mid360_square_localizer',
        output='screen',
        parameters=[mid360_localization_config, {
            'use_sim_time': use_sim_time,
            'publish_map_to_odom': False,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'static_obstacle_enabled': True,
            'static_obstacle_minimum_x': 0.20,
            'static_obstacle_maximum_x': 1.00,
            'static_obstacle_minimum_y': -0.60,
            'static_obstacle_maximum_y': 0.60,
        }],
    )

    # The mature support preloads require their Gazebo transport topics to
    # have exactly zero endpoints before the detached plugins are installed.
    # Keep the historical default, but allow an execution supervisor to defer
    # these two one-way state bridges until after both exact preloads pass.
    support_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/h1_2/right_door_support/state@'
            'std_msgs/msg/String[gz.msgs.StringMsg',
            '/h1_2/left_pose_grasp/state@'
            'std_msgs/msg/String[gz.msgs.StringMsg',
        ],
        output='screen',
        condition=IfCondition(bridge_support_states),
    )

    # The incubator joint-state system publishes gz.msgs.Model.  Bridge both
    # door and handle angles explicitly so the exact moving-handle depth mask
    # never depends on an undocumented, manually started process.
    incubator_joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/h1_world/model/incubator/joint_state@'
            'sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/h1_world/model/incubator/joint_state',
             '/incubator/joint_states'),
        ],
        output='screen',
        condition=IfCondition(spawn_incubator),
    )

    # The robot's Gazebo JointStatePublisher timestamps every sample from the
    # physics step itself.  Bridge it only to a private raw topic.  The sealed
    # relay below verifies the exact 1 ms source, fixes Gazebo entity ordering,
    # and becomes the sole public /joint_states authority; ros2_control stays
    # on its local diagnostic topic.
    verify_telemetry_bridge_config = OpaqueFunction(
        function=_verify_launch_file_sha256,
        kwargs={
            'path': telemetry_bridge_config,
            'expected_sha256': telemetry_bridge_config_sha256,
            'label': 'candidate-3 telemetry bridge config',
        },
    )
    telemetry_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='candidate3_telemetry_bridge_829',
        parameters=[{'config_file': telemetry_bridge_config}],
        output='screen',
    )
    stop_if_telemetry_bridge_exits = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=telemetry_bridge,
            on_exit=[EmitEvent(event=Shutdown(
                reason='candidate-3 telemetry bridge exited'))],
        )
    )

    robot_joint_state_relay = ExecuteProcess(
        cmd=[
            '/usr/bin/python3', robot_joint_state_relay_source,
            '--self-sha256', robot_joint_state_relay_sha256,
        ],
        output='screen',
    )
    stop_if_robot_joint_state_relay_exits = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_joint_state_relay,
            on_exit=[EmitEvent(event=Shutdown(
                reason='candidate-3 robot joint-state relay exited'))],
        )
    )

    # One-way observational bridge for the unchanged handle collision.  The
    # Contacts message retains both collision names plus depth / wrench data;
    # it does not expose any Gazebo command channel.
    incubator_handle_contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/incubator/handle/contacts@'
            'ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        output='screen',
        condition=IfCondition(spawn_incubator),
    )

    # Publish a bounded, downsampled XYZ cloud for MoveIt's OctoMap updater.
    d435_point_cloud = Node(
        package='h1_2_vision',
        executable='depth_point_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pixel_stride': 4,
            'minimum_range': 0.15,
            'maximum_range': 2.0,
            'maximum_update_rate': 5.0,
            'target_pose_topic': '/tag_pose/cube',
            'target_size_x': 0.080,
            'target_size_y': 0.160,
            'target_size_z': 0.030,
            # Perception mask only.  The independent PlanningScene guard stays
            # at 10 mm, so unknown points in the 5..10 mm shell remain visible.
            'target_filter_margin': 0.005,
            'target_pose_max_age': 20.0,
            # Exact timestamped PlanningScene meshes mask the incubator.  The
            # former solid OBB / full-sweep-cylinder source filter could erase
            # unknown obstacles far from the real surfaces and is disabled.
            'known_scene_filter_enabled': False,
            'known_scene_frame': 'pelvis',
            'known_scene_filter_margin': 0.005,
            'incubator_x': ParameterValue(incubator_x, value_type=float),
            # Track only the door's current OBB around its exact +Z hinge.
            # Missing, non-finite, or older-than-2-second feedback fails open;
            # no closed-pose or swept-volume mask is permitted.
            'dynamic_door_filter_enabled': True,
            'dynamic_door_joint_topic': '/incubator/joint_states',
            'dynamic_door_filter_margin': 0.005,
            'dynamic_door_state_max_age': 2.0,
            # Filter the detailed handle only at its measured joint angle.
            # This replaces the unsafe full 0..180 degree swept-volume mask.
            'dynamic_handle_filter_enabled': True,
            'dynamic_handle_joint_topic': '/incubator/joint_states',
            'dynamic_handle_filter_margin': 0.007,
            'dynamic_handle_tip_extension': 0.003,
            'dynamic_handle_state_max_age': 2.0,
            # Remove only the exact moving Link2_2.STL AABB from its own
            # depth return.  This was part of the verified handle workflow
            # and must survive a full-machine restart.
            'left_finger2_self_filter_enabled': True,
            # Remove both pitch and collision-less yaw wrist visual returns
            # only within a 7 mm surface band of each exact shipped STL, using
            # image-time TF.  Unlike an AABB or swept volume, concavities stay
            # observable; the band remains below the independent 10 mm gate
            # and changes no PlanningScene geometry or collision permission.
            'wrist_visual_self_filter_enabled': True,
            'wrist_visual_self_filter_margin': 0.007,
            # Apply a rounded Euclidean 5.0 mm Link2_1.STL depth
            # self-render/TF mask.
            # The 1.0 mm profile left a measured point only 1.142 mm outside
            # the exact AABB; a 10 mm OctoMap voxel then intersected the link.
            # Five millimetres exceeds the voxel half-diagonal (4.330 mm),
            # while the Euclidean test avoids the 8.66 mm corner growth of a
            # per-axis expansion.  Exact image-time TF is mandatory.  This
            # changes no PlanningScene geometry, collision permission, or the
            # 10 mm gate.
            'right_finger2_slider_self_filter_enabled': True,
            'right_finger2_slider_self_filter_margin': 0.005,
            # The stationary feet are already in physical contact with the
            # known z=-1 m ground plane.  Remove only that 4 mm-thick support
            # layer within the measured high-resolution foot footprints;
            # obstacles above it and all arm-workspace points remain.
            'support_surface_filter_enabled': True,
            'support_surface_frame': 'pelvis',
            'support_surface_ground_z': -1.000,
            'support_surface_half_thickness': 0.002,
            'support_surface_x_min': -0.095,
            'support_surface_x_max': 0.195,
            'support_surface_left_y_min': 0.104,
            'support_surface_left_y_max': 0.220,
            'support_surface_right_y_min': -0.217,
            'support_surface_right_y_max': -0.104,
        }]
    )

    # Synchronized RGB/depth AprilTag localization is perception-only.  It
    # publishes confidence-bearing object states and never creates a controller
    # publisher or commands Gazebo.
    tag_localizer = Node(
        package='h1_2_vision',
        executable='tag_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'world_frame': 'world',
        }],
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([h1_2_moveit_config_path, 'config', 'ros2_controllers.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # 加载控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_torso_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'torso_controller'],
        output='screen'
    )

    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_arm_controller'],
        output='screen'
    )

    load_right_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'right_gripper_controller'],
        output='screen'
    )

    load_left_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_gripper_controller'],
        output='screen'
    )

    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'left_arm_controller'],
        output='screen'
    )

    # 延迟启动控制器
    delay_controller_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                load_joint_state_broadcaster,
                load_torso_controller,
                load_right_arm_controller,
                load_right_gripper_controller,
                load_left_gripper_controller,
                load_left_arm_controller
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument(
            'spawn_incubator', default_value='false',
            description='Spawn the imported static laboratory incubator'),
        DeclareLaunchArgument(
            'table_x', default_value='0.600',
            description='Table centre X; 0.600 gives 100 mm extra robot clearance'),
        DeclareLaunchArgument(
            'cube_x', default_value='0.334',
            description='Initial cube X; retained below the neutral left hand'),
        DeclareLaunchArgument('incubator_x', default_value='0.590'),
        DeclareLaunchArgument('incubator_y', default_value='0.00'),
        DeclareLaunchArgument('incubator_z', default_value='0.815'),
        DeclareLaunchArgument(
            'bridge_support_states', default_value='false',
            description=(
                'Bridge support states immediately only for diagnostics; the '
                'execution supervisor keeps the topics empty until both exact '
                'detached support plugins have been preloaded')),
        DeclareLaunchArgument(
            'robot_joint_state_relay_source',
            default_value=robot_joint_state_relay_default,
            description='SHA-bound robot telemetry relay source'),
        DeclareLaunchArgument(
            'robot_joint_state_relay_sha256',
            default_value=robot_joint_state_relay_default_sha256,
            description='Expected SHA-256 of the robot telemetry relay'),
        DeclareLaunchArgument(
            'telemetry_bridge_config',
            default_value=telemetry_bridge_config_default,
            description='SHA-bound high-capacity native telemetry bridge'),
        DeclareLaunchArgument(
            'telemetry_bridge_config_sha256',
            default_value=telemetry_bridge_config_default_sha256,
            description='Expected SHA-256 of the telemetry bridge config'),
        gz_model_path,
        gz_plugin_path,
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        spawn_table,
        spawn_cube,
        spawn_incubator_model,
        gz_ros_image_bridge,
        gz_ros_bridge,
        mid360_point_cloud_bridge,
        mid360_localizer,
        support_state_bridge,
        incubator_joint_state_bridge,
        verify_telemetry_bridge_config,
        telemetry_bridge,
        stop_if_telemetry_bridge_exits,
        robot_joint_state_relay,
        stop_if_robot_joint_state_relay_exits,
        incubator_handle_contact_bridge,
        d435_point_cloud,
        tag_localizer,
        delay_controller_after_spawn
    ])
