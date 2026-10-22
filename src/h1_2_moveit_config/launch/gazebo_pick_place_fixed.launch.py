from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    完整的抓取演示：Gazebo + MoveIt + C++ demo
    禁用初始状态边界检查
    """
    config_share = get_package_share_directory("h1_2_moveit_config")

    # 1. 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_share, "launch", "gazebo.launch.py")
        )
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

    # 3. MoveIt move_group（延迟 10 秒，禁用初始状态边界检查）
    move_group_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {
                        "use_sim_time": True,
                        # 禁用初始状态边界检查
                        "start_state_max_bounds_error": 100.0,  # 默认 0.1，增大到 100
                    }
                ],
            )
        ]
    )

    # 4. C++ 抓取演示（延迟 18 秒）
    pick_place_demo = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="h1_2_moveit_demo",
                executable="right_arm_pick_place_demo",
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
        move_group_node,
        pick_place_demo
    ])
