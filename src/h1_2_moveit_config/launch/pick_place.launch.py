from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    config_share = get_package_share_directory("h1_2_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_share, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "run_demo": "false",
            "use_rviz": use_rviz,
        }.items(),
    )

    pick_place_demo = Node(
        package="h1_2_moveit_demo",
        executable="right_arm_pick_place_demo",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz for the pick-and-place visualization.",
            ),
            backend,
            TimerAction(period=8.0, actions=[pick_place_demo]),
        ]
    )
