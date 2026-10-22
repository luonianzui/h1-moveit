import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    run_demo = LaunchConfiguration("run_demo")
    use_rviz = LaunchConfiguration("use_rviz")

    moveit_config = (
        MoveItConfigsBuilder("h1_2", package_name="h1_2_moveit_config")
        .robot_description(file_path="config/h1_2.urdf.xacro")
        .robot_description_semantic(file_path="config/h1_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    fixed_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world", "--child-frame-id", "pelvis",
        ],
    )

    controllers_file = os.path.join(
        get_package_share_directory("h1_2_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
            output="screen",
        )
        for controller in (
            "joint_state_broadcaster",
            "left_arm_controller",
            "right_arm_controller",
            "left_gripper_controller",
            "right_gripper_controller",
        )
    ]

    rviz_config = os.path.join(
        get_package_share_directory("h1_2_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
            "QT_X11_NO_MITSHM": "1",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "MESA_GL_VERSION_OVERRIDE": "3.3COMPAT",
        },
        output="screen",
        condition=IfCondition(use_rviz),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    dual_arm_demo = Node(
        package="h1_2_moveit_demo",
        executable="dual_arm_plan_demo",
        output="screen",
        condition=IfCondition(run_demo),
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_demo",
                default_value="true",
                description="Automatically plan and execute the dual-arm demonstration.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz for visualization.",
            ),
            fixed_tf,
            robot_state_publisher,
            move_group,
            ros2_control,
            *controller_spawners,
            rviz,
            TimerAction(period=8.0, actions=[dual_arm_demo]),
        ]
    )
