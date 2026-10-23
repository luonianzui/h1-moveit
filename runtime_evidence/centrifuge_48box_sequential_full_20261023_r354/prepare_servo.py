import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as element_tree

import yaml

from servo_model import predicted_ramp_error, verify_rate_fork


EVIDENCE = Path(__file__).resolve().parent


def main():
    original = yaml.safe_load((EVIDENCE / 'inputs/controllers.yaml').read_text())
    changed = copy.deepcopy(original)
    changed['controller_manager']['ros__parameters']['update_rate'] = 500
    verify_rate_fork(original, changed)
    with (EVIDENCE / 'inputs/controllers_servo.yaml').open('x') as stream:
        yaml.safe_dump(changed, stream, sort_keys=False)
    original_robot = element_tree.parse(EVIDENCE / 'inputs/robot_effort.urdf').getroot()
    robot = copy.deepcopy(original_robot)
    parameter = robot.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters")
    previous = parameter.text
    parameter.text = str(EVIDENCE / 'inputs/controllers_servo.yaml')
    restored = copy.deepcopy(robot)
    restored.find("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters").text = previous
    assert element_tree.tostring(restored) == element_tree.tostring(original_robot)
    with (EVIDENCE / 'inputs/robot_servo.urdf').open('xb') as stream:
        stream.write(element_tree.tostring(robot, encoding='utf-8', xml_declaration=True))
    for source, output, old, new in (
            ('gazebo_effort.launch.py', 'gazebo_servo.launch.py', "'robot_effort.urdf'", "'robot_servo.urdf'"),
            ('lab_effort.launch.py', 'lab_servo.launch.py', 'gazebo_effort.launch.py', 'gazebo_servo.launch.py')):
        content = (EVIDENCE / 'inputs' / source).read_text()
        assert old in content
        changed_source = content.replace(old, new)
        compile(changed_source, output, 'exec')
        with (EVIDENCE / 'inputs' / output).open('x') as stream:
            stream.write(changed_source)
    with (EVIDENCE / 'servo_fork_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', manager_update_rate_hz=[100, 500],
            all_other_controller_parameters_unchanged=True, geometry_limits_inertias_pose_friction_unchanged=True,
            position_proportional_gain=.2, unchanged_tracking_guard_rad=.015,
            model_predicted_ramp_error_rad=predicted_ramp_error(.8, 500., .2),
            estimate_not_fresh_physical_certificate=True, calibration_required_before_fast_loaded_motion=True,
            source='https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html',
            robot_servo_sha256=hashlib.sha256((EVIDENCE / 'inputs/robot_servo.urdf').read_bytes()).hexdigest()), stream, indent=2)


if __name__ == '__main__':
    main()
