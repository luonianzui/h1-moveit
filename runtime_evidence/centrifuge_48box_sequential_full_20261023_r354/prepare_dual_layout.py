import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as element_tree


EVIDENCE = Path(__file__).resolve().parent


def main():
    source = EVIDENCE / 'inputs/robot_servo.urdf'
    original = element_tree.parse(source).getroot()
    robot = copy.deepcopy(original)
    joints = [joint for joint in robot.findall('joint') if joint.find('parent').get('link') == 'world']
    assert len(joints) == 1 and joints[0].get('type') == 'fixed'
    origin = joints[0].find('origin')
    previous = origin.get('xyz')
    values = previous.split()
    assert float(values[0]) == 0.
    values[0] = '.12'
    origin.set('xyz', ' '.join(values))
    restored = copy.deepcopy(robot)
    restored.find(f"./joint[@name='{joints[0].get('name')}']/origin").set('xyz', previous)
    assert element_tree.tostring(restored) == element_tree.tostring(original)
    with (EVIDENCE / 'inputs/robot_dual.urdf').open('xb') as stream:
        stream.write(element_tree.tostring(robot, encoding='utf-8', xml_declaration=True))
    gazebo = (EVIDENCE / 'inputs/gazebo_servo.launch.py').read_text()
    assert gazebo.count("'robot_servo.urdf'") == 1
    gazebo = gazebo.replace("'robot_servo.urdf'", "'robot_dual.urdf'")
    compile(gazebo, 'gazebo_dual.launch.py', 'exec')
    with (EVIDENCE / 'inputs/gazebo_dual.launch.py').open('x') as stream:
        stream.write(gazebo)
    lab = (EVIDENCE / 'inputs/lab_servo.launch.py').read_text()
    assert lab.count("'-x', '0.600', '-y', '0.0', '-z', '0.815'") == 1
    assert lab.count('gazebo_servo.launch.py') == 1
    lab = lab.replace("'-x', '0.600', '-y', '0.0', '-z', '0.815'", "'-x', '0.520', '-y', '0.0', '-z', '0.815'")
    lab = lab.replace('gazebo_servo.launch.py', 'gazebo_dual.launch.py')
    compile(lab, 'lab_dual.launch.py', 'exec')
    with (EVIDENCE / 'inputs/lab_dual.launch.py').open('x') as stream:
        stream.write(lab)
    routes = json.loads((EVIDENCE / 'inputs/reverse_clearance_routes.json').read_text())
    selected = next(row for row in routes['records'] if row['outboard_y_m'] == .30)
    assert selected['status'] == 'COMPLETE_REVERSIBLE_CARRY_GEOMETRY_PASS_NOT_PHYSICS'
    with (EVIDENCE / 'inputs/dual_route.json').open('x') as stream:
        json.dump(selected, stream, indent=2)
    output = dict(status='PASS', initial_fixed_pelvis_x_m=.12, device_x_m=.52,
                  table_x_m=.60, table_slots_m=[[.44, .38, .84475], [.44, -.38, .84475]],
                  physics_controller_and_all_model_geometry_inertias_limits_friction_unchanged=True,
                  startup_initial_position_not_leg_motion=True, prior_identity_never_reused=True,
                  robot_dual_sha256=hashlib.sha256((EVIDENCE / 'inputs/robot_dual.urdf').read_bytes()).hexdigest(),
                  original_generated_robot_kept_for_legacy_reference_checks=True,
                  dual_route_requires_real_contact_new_grip_guards_and_actual_state_replanning=True)
    with (EVIDENCE / 'dual_layout_audit.json').open('x') as stream:
        json.dump(output, stream, indent=2)
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
