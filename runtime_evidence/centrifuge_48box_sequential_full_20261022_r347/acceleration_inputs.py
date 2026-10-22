import xml.etree.ElementTree as ET

import json
from pathlib import Path
from standing_soles import make_standing_soles

ORIGINAL_GROUP = "on_exit=[\n                load_joint_state_broadcaster,\n                load_torso_controller,\n                load_right_arm_controller,\n                load_right_gripper_controller,\n                load_left_gripper_controller,\n                load_left_arm_controller\n            ]"
GROUPED_SPAWNER = "on_exit=[Node(package='controller_manager', executable='spawner',\n                          arguments=['joint_state_broadcaster', 'torso_controller',\n                                     'left_arm_controller', 'right_arm_controller',\n                                     'left_gripper_controller', 'right_gripper_controller',\n                                     '--controller-manager-timeout', '60', '--service-call-timeout', '60',\n                                     '--switch-timeout', '60', '--activate-as-group'], output='screen')]"


def canonical(element):
    return element.tag, element.attrib, (element.text or '').strip(), tuple(canonical(child) for child in element)


def verify_acceleration_input(expected, actual, filename, description):
    if filename == 'robot_dual.urdf':
        positions = json.loads((Path(__file__).resolve().parent / 'inputs/plan.json').read_text())['initial_positions']
        candidate, records = make_standing_soles(expected.decode(), description, positions)
        if canonical(ET.fromstring(actual)) != canonical(ET.fromstring(candidate)):
            raise ValueError('Unreviewed geometry outside two ground-trimmed stationary foot colliders')
        return dict(scope='fixed_pelvis_original_stance_ground_trimmed_OBB', records=records,
                    manipulation_geometry_inertia_friction_and_original_task_targets_unchanged=True)
    if filename == 'gazebo_dual.launch.py':
        source = expected.decode()
        restored = actual.decode().replace("PythonLaunchDescriptionSource(str(Path(__file__).with_name('gz_solver.launch.py')))", "PythonLaunchDescriptionSource(os.path.join(\n            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))")
        if source.count(ORIGINAL_GROUP) != 1 or restored != source.replace(ORIGINAL_GROUP, GROUPED_SPAWNER):
            raise ValueError('Only grouped activation of the original six controllers allowed')
        return dict(scope='same_six_controllers_group_activation_and_owned_gazebo_exact_order_dart_cache')
    raise ValueError('Unexpected acceleration input')
