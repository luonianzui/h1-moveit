from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """只启动 pick_place demo 节点，配合已运行的 Gazebo + move_group"""

    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    pick_place_demo = Node(
        package="h1_2_moveit_demo",
        executable="right_arm_pick_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([pick_place_demo])
