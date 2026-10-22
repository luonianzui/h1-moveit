import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    完整的抓取测试启动文件：Gazebo + MoveIt move_group
    """

    h1_2_moveit_config_path = get_package_share_directory('h1_2_moveit_config')

    # 1. 启动 Gazebo（包含机器人、桌子、方块）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            h1_2_moveit_config_path,
            '/launch/gazebo.launch.py'
        ])
    )

    # 2. MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=False
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 3. MoveIt move_group 节点（延迟 10 秒启动，等待 Gazebo 和控制器）
    move_group_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True}
                ],
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        move_group_node
    ])
