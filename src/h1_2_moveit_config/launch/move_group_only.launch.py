import os
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    启动 MoveIt move_group，配合已运行的 Gazebo
    """

    dynamic_scene_freeze_once = LaunchConfiguration(
        'dynamic_scene_freeze_once')
    require_post_navigation_hold = LaunchConfiguration(
        'require_post_navigation_hold')
    post_navigation_goal_x = LaunchConfiguration('post_navigation_goal_x')
    post_navigation_goal_y = LaunchConfiguration('post_navigation_goal_y')
    post_navigation_goal_yaw = LaunchConfiguration(
        'post_navigation_goal_yaw')
    table_x = LaunchConfiguration('table_x')

    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True
        )
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveIt move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "octomap_frame": "pelvis",
                # A 5 mm voxel prevents the 10 mm occupancy cube itself from
                # re-entering the sub-millimetre residual shell left after
                # the calibrated 6 mm robot self-mask.
                "octomap_resolution": 0.005,
            }
        ],
    )

    environment_scene_node = Node(
        package="h1_2_vision",
        executable="environment_scene",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "planning_frame": "pelvis",
                "table_x": ParameterValue(table_x, value_type=float),
                "table_y": 0.0,
                "table_center_z": -0.2,
                "safety_margin": 0.01,
            }
        ],
    )

    dynamic_scene_node = Node(
        package="h1_2_vision",
        executable="dynamic_scene",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "planning_frame": "pelvis",
                "require_pointcloud": True,
                "require_latch": True,
                "freeze_once": ParameterValue(
                    dynamic_scene_freeze_once, value_type=bool),
                "require_post_navigation_hold": ParameterValue(
                    require_post_navigation_hold, value_type=bool),
                "post_navigation_goal_x": ParameterValue(
                    post_navigation_goal_x, value_type=float),
                "post_navigation_goal_y": ParameterValue(
                    post_navigation_goal_y, value_type=float),
                "post_navigation_goal_yaw": ParameterValue(
                    post_navigation_goal_yaw, value_type=float),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'dynamic_scene_freeze_once', default_value='false',
            description='Freeze and publish only the first complete epoch'),
        DeclareLaunchArgument(
            'require_post_navigation_hold', default_value='false',
            description='Require a fresh 30 s Nav2 stop-and-hold certificate'),
        DeclareLaunchArgument(
            'post_navigation_goal_x', default_value='0.0',
            description='Authorized final manipulation station world X'),
        DeclareLaunchArgument(
            'post_navigation_goal_y', default_value='0.0',
            description='Authorized final manipulation station world Y'),
        DeclareLaunchArgument(
            'post_navigation_goal_yaw', default_value='0.0',
            description='Authorized final manipulation station world yaw'),
        DeclareLaunchArgument(
            'table_x', default_value='0.600',
            description='Table centre X matching the shifted Gazebo workcell'),
        move_group_node,
        environment_scene_node,
        dynamic_scene_node,
    ])
