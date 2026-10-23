import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
ORIGINAL = ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf'
NAMES = [side + f'_dex1_finger_{number}_joint' for side in ('left', 'right') for number in (1, 2)]


def create(path, data):
    with path.open('xb') as stream:
        stream.write(data)


def normalized(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [normalized(child) for child in element])


original = ET.parse(ORIGINAL).getroot()
robot = copy.deepcopy(original)
for name in NAMES:
    joint = robot.find(f"./ros2_control/joint[@name='{name}']")
    interface = joint.find('command_interface')
    assert interface.attrib == {'name': 'position'} and len(interface) == 0
    interface.set('name', 'effort')
    ET.SubElement(interface, 'param', name='min').text = '-18'
    ET.SubElement(interface, 'param', name='max').text = '18'
plugin = robot.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
plugin.find('parameters').text = str(EVIDENCE / 'inputs/controllers.yaml')
assert plugin.find('position_proportional_gain') is None
ET.SubElement(plugin, 'position_proportional_gain').text = '0.2'
FRICTION_LINKS = [side + f'_dex1_finger_{number}_' + suffix
                  for side in ('left', 'right') for number in (1, 2) for suffix in ('link', 'pad_link')]
for name in FRICTION_LINKS:
    assert robot.find(f"./link[@name='{name}']/collision") is not None
    assert robot.find(f"./gazebo[@reference='{name}']") is None
    contact = ET.SubElement(robot, 'gazebo', reference=name)
    ET.SubElement(contact, 'mu1').text = '1.2'
    ET.SubElement(contact, 'mu2').text = '1.2'
restored = copy.deepcopy(robot)
restored_plugin = restored.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
restored_plugin.remove(restored_plugin.find('position_proportional_gain'))
for name in FRICTION_LINKS:
    restored.remove(restored.find(f"./gazebo[@reference='{name}']"))
for name in NAMES:
    interface = restored.find(f"./ros2_control/joint[@name='{name}']/command_interface")
    interface.clear()
    interface.set('name', 'position')
restored.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters").text = original.find(
    "./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters").text
assert normalized(restored) == normalized(original)
create(EVIDENCE / 'inputs/robot_effort.urdf', ET.tostring(robot, encoding='utf-8', xml_declaration=True))
configuration = yaml.safe_load((ROOT / 'src/h1_2_moveit_config/config/ros2_controllers.yaml').read_text())
original_configuration = copy.deepcopy(configuration)
for side in ('left', 'right'):
    parameters = configuration[side + '_gripper_controller']['ros__parameters']
    parameters['command_interfaces'] = ['effort']
    parameters['gains'] = {name: dict(p=500.0, i=1000.0, d=1.5, i_clamp_min=-1.0,
                                     i_clamp_max=1.0, u_clamp_min=-2.0, u_clamp_max=2.0,
                                     ff_velocity_scale=0.0) for name in parameters['joints']}
for name, value in configuration.items():
    if name not in ('left_gripper_controller', 'right_gripper_controller'):
        assert value == original_configuration[name]
configuration['controller_manager']['ros__parameters']['leg_posture_controller'] = dict(type='joint_trajectory_controller/JointTrajectoryController')
configuration['leg_posture_controller'] = dict(ros__parameters=dict(
    joints=[side + '_' + suffix + '_joint' for side in ('left', 'right')
            for suffix in ('hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll')],
    command_interfaces=['position'], state_interfaces=['position', 'velocity'],
    interpolation_method='splines', state_publish_rate=50.0))
create(EVIDENCE / 'inputs/controllers.yaml', yaml.safe_dump(configuration, sort_keys=False).encode())
gazebo = (ROOT / 'src/h1_2_moveit_config/launch/gazebo.launch.py').read_text()
begin = gazebo.index('    robot_description_content = Command([')
end = gazebo.index('\n\n    robot_description =', begin)
gazebo = 'from pathlib import Path\n' + gazebo[:begin] + (
    "    robot_description_content = Path(__file__).with_name('robot_effort.urdf').read_text()") + gazebo[end:]
relay_begin = gazebo.index('    robot_joint_state_relay_default = os.path.join(')
relay_end = gazebo.index('\n    robot_joint_state_relay_default_sha256 =', relay_begin)
gazebo = gazebo[:relay_begin] + '    robot_joint_state_relay_default = ' + repr(str(EVIDENCE / 'inputs/robot_joint_state_relay_05ms.py')) + gazebo[relay_end:]
assert (EVIDENCE / 'inputs/robot_joint_state_relay_05ms.py').is_file()
compile(gazebo, 'gazebo_effort.launch.py', 'exec')
create(EVIDENCE / 'inputs/gazebo_effort.launch.py', gazebo.encode())
payload = ET.parse(ROOT / 'src/h1_2_description/models/lab_48box/model.sdf').getroot()
original_payload = copy.deepcopy(payload)
for parameter in ('mu', 'mu2'):
    node = payload.find('./model/link/collision/surface/friction/ode/' + parameter)
    assert node.text == '0.6'
    node.text = '1.2'
restored_payload = copy.deepcopy(payload)
for parameter in ('mu', 'mu2'):
    restored_payload.find('./model/link/collision/surface/friction/ode/' + parameter).text = '0.6'
assert normalized(restored_payload) == normalized(original_payload)
create(EVIDENCE / 'inputs/48box_friction.sdf', ET.tostring(payload, encoding='utf-8', xml_declaration=True))
lab = (ROOT / 'src/h1_2_moveit_config/launch/lab_scene.launch.py').read_text()
gui_source = ROOT / 'src/h1_2_moveit_config/config/centrifuge_gui.config'
gui = ET.fromstring('<config>' + gui_source.read_text().split('?>', 1)[1] + '</config>')
original_gui = copy.deepcopy(gui)
controls = [plugin for plugin in gui.findall('plugin') if plugin.get('filename') == 'WorldControl']
assert len(controls) == 1
control_index = list(gui).index(controls[0])
gui.remove(controls[0])
restored_gui = copy.deepcopy(gui)
restored_gui.insert(control_index, copy.deepcopy(controls[0]))
assert normalized(restored_gui) == normalized(original_gui)
gui_path = EVIDENCE / 'inputs/centrifuge_run_gui.config'
gui_content = '<?xml version="1.0"?>\n' + '\n'.join(ET.tostring(child, encoding='unicode') for child in gui)
create(gui_path, gui_content.encode())
old_gui = "os.path.join(\n                    config, 'config', 'centrifuge_gui.config')"
assert lab.count(old_gui) == 1
lab = lab.replace(old_gui, repr(str(gui_path)))
old_payload = "os.path.join(description, 'models', 'lab_48box', 'model.sdf')"
assert lab.count(old_payload) == 1
lab = lab.replace(old_payload, repr(str(EVIDENCE / 'inputs/48box_friction.sdf')))
machine = ET.parse(ROOT / 'src/h1_2_description/models/centrifuge/model.sdf').getroot()
original_machine = copy.deepcopy(machine)
motor = machine.find("./model/plugin[@name='CentrifugeInterlock']")
library_path = EVIDENCE / 'inputs/libCentrifugeInterlockBrakeR189.so'
assert library_path.is_file()
motor.set('name', 'CentrifugeInterlockBrakeR189')
motor.set('filename', str(library_path))
ET.SubElement(motor, 'stopped_velocity_gain').text = '80'
ET.SubElement(motor, 'stopped_position_gain').text = '8000'
restored_machine = copy.deepcopy(machine)
restored_motor = restored_machine.find("./model/plugin[@name='CentrifugeInterlockBrakeR189']")
restored_motor.remove(restored_motor.find('stopped_velocity_gain'))
restored_motor.remove(restored_motor.find('stopped_position_gain'))
restored_motor.set('name', 'CentrifugeInterlock')
restored_motor.set('filename', 'CentrifugeInterlock')
assert normalized(restored_machine) == normalized(original_machine)
create(EVIDENCE / 'inputs/centrifuge_brake.sdf', ET.tostring(machine, encoding='utf-8', xml_declaration=True))
old_machine = "os.path.join(description, 'models', 'centrifuge', 'model.sdf')"
assert lab.count(old_machine) == 1
lab = lab.replace(old_machine, repr(str(EVIDENCE / 'inputs/centrifuge_brake.sdf')))
old = "os.path.join(config, 'launch', 'gazebo.launch.py')"
assert lab.count(old) == 1
lab = lab.replace(old, repr(str(EVIDENCE / 'inputs/gazebo_effort.launch.py')))
world = ET.parse(ROOT / 'src/h1_2_description/worlds/lab_scene_world.sdf').getroot()
original_world = copy.deepcopy(world)
physics = world.find('./world/physics')
assert physics.find('max_step_size').text == '0.001'
assert physics.find('real_time_update_rate').text == '500'
assert physics.find('real_time_factor').text == '0.5'
physics.find('max_step_size').text = '0.0005'
physics.find('real_time_update_rate').text = '2000'
physics.find('real_time_factor').text = '1.0'
restored_world = copy.deepcopy(world)
restored_world.find('./world/physics/max_step_size').text = '0.001'
restored_world.find('./world/physics/real_time_update_rate').text = '500'
restored_world.find('./world/physics/real_time_factor').text = '0.5'
assert normalized(restored_world) == normalized(original_world)
create(EVIDENCE / 'inputs/lab_fast_world.sdf', ET.tostring(world, encoding='utf-8', xml_declaration=True))
old_world = "os.path.join(\n                    description, 'worlds', 'lab_scene_world.sdf')"
assert lab.count(old_world) == 1
lab = lab.replace(old_world, repr(str(EVIDENCE / 'inputs/lab_fast_world.sdf')))
lab = lab.replace("    description = get_package_share_directory('h1_2_description')",
                  "    if scene != 'centrifuge' or with_48box != 'true' or robot_press != 'true' or demo != 'false':\n"
                  "        raise ValueError('Full-cycle experiment requires contact buttons and the closed-lid interlock')\n"
                  "    description = get_package_share_directory('h1_2_description')")
old_guard = "    if with_48box == 'true' and robot_press == 'true':\n        raise ValueError('Table payloads require a new collision-audited robot plan; do not reuse r119 pressing')\n"
assert lab.count(old_guard) == 1
lab = lab.replace(old_guard, "")
compile(lab, 'lab_effort.launch.py', 'exec')
create(EVIDENCE / 'inputs/lab_effort.launch.py', lab.encode())
proof = dict(status='PASS', geometry_inertias_joints_unchanged=True,
             friction_links=FRICTION_LINKS, friction_coefficient=1.2,
             position_proportional_gain=0.2, previous_default_gain=0.1,
             payload_friction_before=0.6, payload_contact_changes_include_table=True,
             only_four_gripper_command_interfaces_changed=NAMES, limits_N=[-18, 18],
             original_urdf_sha256=hashlib.sha256(ORIGINAL.read_bytes()).hexdigest(),
             original_controller_sha256=hashlib.sha256((ROOT / 'src/h1_2_moveit_config/config/ros2_controllers.yaml').read_bytes()).hexdigest(),
             other_controllers_unchanged=True, experimental_only=True,
             world_only_changes=dict(max_step_size=[.001, .0005], real_time_update_rate=[500, 2000], real_time_factor=[.5, 1.0]),
             physics_step_s=.0005, relay_exact_source_step_ns=500000, relay_output_period_ns=10000000)
proof['motor_brake'] = dict(stopped_velocity_gain=80, stopped_position_gain=8000, torque_limit_nm=2, model_geometry_inertia_joints_unchanged=True,
                           library_sha256=hashlib.sha256(library_path.read_bytes()).hexdigest(),
                           parent_bench='centrifuge_48box_position_brake_20261010_r189')
proof['gui_only_removed_plugin'] = 'WorldControl'
proof['original_gui_sha256'] = hashlib.sha256(gui_source.read_bytes()).hexdigest()
proof['experimental_gui_sha256'] = hashlib.sha256(gui_path.read_bytes()).hexdigest()
create(EVIDENCE / 'control_fork_audit.json', json.dumps(proof, indent=2).encode())
print(json.dumps(proof, indent=2))

