import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 获取包路径
    h1_2_description_path = get_package_share_directory('h1_2_description')
    h1_2_moveit_config_path = FindPackageShare('h1_2_moveit_config')

    # 设置 Gazebo 资源路径
    # model://h1_2_description/... 要求资源路径的父目录下存在 h1_2_description 文件夹，
    # 因此这里要用 share 目录本身（h1_2_description 的父目录），而非 h1_2_description 自己。
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(h1_2_description_path)
    )

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([h1_2_moveit_config_path, 'config', 'h1_2.urdf.xacro']),
        ' initial_positions_file:=',
        PathJoinSubstitution([h1_2_moveit_config_path, 'config', 'initial_positions.yaml'])
    ])

    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo 启动
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # 生成机器人到 Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'h1_2',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        gz_model_path,
        robot_state_publisher_node,
        gz_sim,
        spawn_robot
    ])
