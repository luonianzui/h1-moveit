from pathlib import Path


ROOT = Path(__file__).resolve().parent
OLD_IMPORT = 'from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable'
NEW_IMPORT = OLD_IMPORT + ', ExecuteProcess'
OLD_NODE = """        Node(
            package='h1_2_navigation',
            executable='mid360_square_localizer',
            name='h1_2_mid360_square_localizer',
            output='screen',
            parameters=[
                os.path.join(navigation, 'config', 'mid360_localization.yaml'),
                {'use_sim_time': True, 'static_obstacle_enabled': True},
            ],
        ),"""
NEW_NODE = f"""        ExecuteProcess(
            cmd=['/usr/bin/python3', '{ROOT}/inputs/localizer_runtime.py',
                 '--ros-args', '--params-file',
                 os.path.join(navigation, 'config', 'mid360_localization.yaml'),
                 '-p', 'use_sim_time:=true', '-p', 'static_obstacle_enabled:=true',
                 '-p', 'localization_rate:=5.0'],
            output='screen',
        ),"""


def restore_localizer_launch(source):
    if source.count(NEW_NODE) != 1 or source.count(NEW_IMPORT) != 1:
        raise ValueError('Unexpected localization launch delta')
    return source.replace(NEW_NODE, OLD_NODE, 1).replace(NEW_IMPORT, OLD_IMPORT, 1)
