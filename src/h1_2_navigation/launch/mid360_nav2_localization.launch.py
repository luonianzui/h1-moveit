import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('h1_2_navigation')
    default_localizer_parameters = os.path.join(
        package_share, 'config', 'mid360_localization.yaml')
    default_nav2_parameters = os.path.join(
        package_share, 'config', 'nav2_amcl.yaml')
    localizer_parameters = LaunchConfiguration('localizer_parameters_file')
    nav2_parameters = LaunchConfiguration('nav2_parameters_file')
    publish_fixed_odom_tf = LaunchConfiguration('publish_fixed_odom_tf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'localizer_parameters_file',
            default_value=default_localizer_parameters,
            description='Mid-360 projection and square-map parameters',
        ),
        DeclareLaunchArgument(
            'nav2_parameters_file',
            default_value=default_nav2_parameters,
            description='Nav2 AMCL and lifecycle parameters',
        ),
        DeclareLaunchArgument(
            'publish_fixed_odom_tf',
            default_value='true',
            description=(
                'Publish identity odom to base_footprint for the fixed '
                'localization test; disable when RL odometry is available'),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fixed_odom_to_base_footprint',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'pelvis',
            ],
            condition=IfCondition(publish_fixed_odom_tf),
        ),
        Node(
            package='h1_2_navigation',
            executable='mid360_square_localizer',
            name='h1_2_mid360_square_localizer',
            output='screen',
            parameters=[
                localizer_parameters,
                {'scan_frame': 'pelvis'},
            ],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_parameters],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[nav2_parameters],
        ),
    ])
