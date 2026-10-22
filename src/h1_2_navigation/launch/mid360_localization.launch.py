import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('h1_2_navigation')
    default_parameters = os.path.join(
        package_share, 'config', 'mid360_localization.yaml')
    parameters_file = LaunchConfiguration('parameters_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters_file',
            default_value=default_parameters,
            description='Mid-360 square localization parameters',
        ),
        Node(
            package='h1_2_navigation',
            executable='mid360_square_localizer',
            name='h1_2_mid360_square_localizer',
            output='screen',
            parameters=[parameters_file],
        ),
    ])
