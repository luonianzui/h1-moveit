import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    rviz_config = os.path.join(
        get_package_share_directory("h1_2_moveit_config"),
        "config",
        "moveit.rviz",
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--ogre-log", "-d", rviz_config],
                additional_env={
                    "QT_QPA_PLATFORM": "xcb",
                    "QT_X11_NO_MITSHM": "1",
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                    "MESA_GL_VERSION_OVERRIDE": "3.3COMPAT",
                },
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            )
        ]
    )
