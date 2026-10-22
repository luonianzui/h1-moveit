import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _stop_on_spawn_failure(event, context):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='Centrifuge spawn failed'))]
    return []


def _scene_actions(context):
    scene = LaunchConfiguration('scene').perform(context)
    if scene not in ('centrifuge', 'incubator'):
        raise ValueError('scene must be centrifuge or incubator')
    with_48box = LaunchConfiguration('with_48box').perform(context)
    if with_48box not in ('true', 'false') or (with_48box == 'true' and scene != 'centrifuge'):
        raise ValueError('with_48box requires centrifuge and true/false')
    demo = LaunchConfiguration('allow_open_lid_demo').perform(context)
    if demo not in ('true', 'false') or (demo == 'true' and scene != 'centrifuge'):
        raise ValueError('allow_open_lid_demo requires centrifuge and true/false')
    robot_press = LaunchConfiguration('enable_robot_press').perform(context)
    if robot_press not in ('true', 'false') or (
            robot_press == 'true' and (scene != 'centrifuge' or demo != 'false')):
        raise ValueError('Robot pressing requires centrifuge with the closed-lid interlock')
    if scene != 'centrifuge' or with_48box != 'true' or robot_press != 'true' or demo != 'false':
        raise ValueError('Full-cycle experiment requires contact buttons and the closed-lid interlock')
    description = get_package_share_directory('h1_2_description')
    config = get_package_share_directory('h1_2_moveit_config')
    navigation = get_package_share_directory('h1_2_navigation')
    if scene == 'centrifuge':
        prefix = get_package_prefix('h1_2_moveit_config')
        for executable in ('centrifuge_lid.py', 'centrifuge_button.py'):
            path = os.path.join(prefix, 'lib', 'h1_2_moveit_config', executable)
            if not os.access(path, os.X_OK):
                raise RuntimeError('Missing executable before scene startup: ' + path)
        if robot_press == 'true' and not os.access(os.path.join(
                prefix, 'lib', 'h1_2_moveit_config', 'centrifuge_contact_button.py'), os.X_OK):
            raise RuntimeError('Missing physical-contact button executable')
        if not os.path.isfile(os.path.join(prefix, 'lib', 'libCentrifugeButton.so')):
            raise RuntimeError('Missing centrifuge GUI plugin before scene startup')
        if not os.path.isfile(os.path.join(prefix, 'lib', 'libCentrifugeInterlock.so')):
            raise RuntimeError('Missing centrifuge interlock before scene startup')
    box_enabled = 'true' if scene == 'incubator' else 'false'
    actions = [
        SetEnvironmentVariable(
            'GZ_GUI_PLUGIN_PATH', os.pathsep.join(filter(None, [
                os.path.join(get_package_prefix('h1_2_moveit_config'), 'lib'),
                os.environ.get('GZ_GUI_PLUGIN_PATH', ''),
            ]))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/gazebo_dual.launch.py'),
            launch_arguments={
                'spawn_incubator': box_enabled,
                'spawn_cube': box_enabled,
                'enable_task_perception': box_enabled,
                'bridge_support_states': 'false',
                'table_x': '0.600',
                'world_file': '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/lab_fast_world.sdf',
                'gui_config': '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/centrifuge_run_gui.config' if scene == 'centrifuge' else '',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['/usr/bin/python3', '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/localizer_runtime.py',
                 '--ros-args', '--params-file',
                 os.path.join(navigation, 'config', 'mid360_localization.yaml'),
                 '-p', 'use_sim_time:=true', '-p', 'static_obstacle_enabled:=true',
                 '-p', 'localization_rate:=5.0'],
            output='screen',
        ),
    ]
    if scene == 'centrifuge':
        if robot_press == 'true':
            actions.extend([
                Node(package='ros_gz_bridge', executable='parameter_bridge',
                     name='centrifuge_contact_bridge',
                     arguments=[
                         '/centrifuge/lid_button/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                         '/centrifuge/rotor_button/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                     ], output='screen'),
                Node(package='h1_2_moveit_config', executable='centrifuge_contact_button.py',
                     parameters=[{'use_sim_time': True}], output='screen'),
            ])
        model_path = '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/centrifuge_brake.sdf'
        model_arguments = ['-file', model_path]
        if demo == 'true':
            model = ET.parse(model_path).getroot()
            plugin = model.find("./model/plugin[@name='CentrifugeInterlock']")
            if plugin is None:
                raise RuntimeError('Missing interlock for simulation-only open-lid demo')
            ET.SubElement(plugin, 'allow_open_lid_demo').text = 'true'
            model_arguments = ['-string', ET.tostring(model, encoding='unicode')]
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=model_arguments + [
                '-name', 'centrifuge',
                '-allow_renaming', 'false',
                '-x', '0.520', '-y', '0.0', '-z', '0.815',
            ],
            output='screen',
        )
        actions.extend([
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='centrifuge_lid_bridge',
                arguments=[
                    '/centrifuge/lid/target_angle@std_msgs/msg/Float64]gz.msgs.Double',
                    '/world/h1_world/model/centrifuge/joint_state@'
                    'sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/centrifuge/button/action@std_msgs/msg/String[gz.msgs.StringMsg',
                    '/centrifuge/rotor/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
                    '/centrifuge/machine/heartbeat@std_msgs/msg/Bool]gz.msgs.Boolean',
                    '/centrifuge/machine/status@std_msgs/msg/String[gz.msgs.StringMsg',
                ],
                remappings=[
                    ('/world/h1_world/model/centrifuge/joint_state',
                     '/centrifuge/joint_states'),
                ],
                output='screen',
            ),
            Node(
                package='h1_2_moveit_config',
                executable='centrifuge_button.py',
                name='centrifuge_button_controller',
                output='screen',
            ),
            RegisterEventHandler(OnProcessExit(
                target_action=spawn, on_exit=_stop_on_spawn_failure)),
            spawn,
        ])
        if with_48box == 'true':
            payload_path = '/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347/inputs/48box_friction.sdf'
            if not os.path.isfile(payload_path):
                raise RuntimeError('Missing 48box model before scene startup')
            for side, ordinate in (('left', '0.38'), ('right', '-0.38')):
                payload_model = ET.parse(payload_path).getroot()
                payload_topic = '/lab_48box/' + side + '/contacts'
                payload_model.find('./model/link/sensor/contact/topic').text = payload_topic
                payload_spawn = Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-string', ET.tostring(payload_model, encoding='unicode'), '-name', 'box48_' + side,
                               '-allow_renaming', 'false', '-x', '0.44', '-y', ordinate, '-z', '0.84575'],
                    output='screen',
                )
                actions.extend([
                    Node(package='ros_gz_bridge', executable='parameter_bridge',
                         name='lab_48box_' + side + '_contact_bridge',
                         arguments=[payload_topic + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
                         output='screen'),
                    RegisterEventHandler(OnProcessExit(
                        target_action=payload_spawn, on_exit=_stop_on_spawn_failure)),
                    payload_spawn,
                ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scene', default_value='centrifuge',
            choices=['centrifuge', 'incubator'],
            description='Laboratory scene; no robot task or walking commands'),
        DeclareLaunchArgument(
            'allow_open_lid_demo', default_value='false', choices=['true', 'false'],
            description='Gazebo-only 30 rpm visual demonstration with a stationary open lid'),
        DeclareLaunchArgument(
            'enable_robot_press', default_value='false', choices=['true', 'false'],
            description='Enable contact-triggered buttons, no automatic robot commands'),
        DeclareLaunchArgument(
            'with_48box', default_value='false', choices=['true', 'false'],
            description='Spawn two user-approved 85-percent 48box payloads on the table; no automatic loading'),
        OpaqueFunction(function=_scene_actions),
    ])
