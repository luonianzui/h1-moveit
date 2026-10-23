import json
from pathlib import Path
import re
import subprocess
import time
from dispatch_gate import fresh_gazebo_processes
from servo_model import verify_manager_rate_output
from gain_contract import verify_gain_output
from gripper_damping import verify_damping_output


EXPECTED_CONTROLLERS = ('joint_state_broadcaster', 'left_arm_controller', 'right_arm_controller',
                        'torso_controller', 'left_gripper_controller', 'right_gripper_controller')


def active_controllers(output):
    output = re.sub(r'\x1b\[[0-9;]*m', '', output)
    rows = {name: [] for name in EXPECTED_CONTROLLERS}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[0] in rows:
            rows[fields[0]].append(fields[-1])
    valid = all(states == ['active'] for states in rows.values())
    return dict(status='PASS' if valid else 'FAIL', terminal_states=rows,
                exact_terminal_token=True, inactive_is_not_active=True)


def command_health(returncode, output):
    health = active_controllers(output)
    health['command_returncode'] = returncode
    if returncode != 0:
        health['status'] = 'FAIL'
    return health


def main():
    root = Path(__file__).resolve().parent
    assert (root / 'run/launch_pid.json').is_file()
    attempts = []
    deadline = time.monotonic() + 90
    health = None
    while time.monotonic() < deadline:
        try:
            response = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True,
                                      text=True, timeout=25)
        except subprocess.TimeoutExpired:
            attempts.append(dict(returncode=None, timeout_wall_s=25, health=dict(status='FAIL')))
            health = None
            time.sleep(1)
            continue
        health = command_health(response.returncode, response.stdout)
        attempts.append(dict(returncode=response.returncode, stdout=response.stdout, stderr=response.stderr,
                             health=health))
        scoped = fresh_gazebo_processes(root)
        if {entry['role'] for entry in scoped} != {'server', 'gui'}:
            health['status'] = 'FAIL'
        if response.returncode == 0 and health['status'] == 'PASS':
            break
        time.sleep(1)
    result = dict(status='PASS' if health and health['status'] == 'PASS' else 'FAIL', attempts=attempts,
                  gazebo_processes=fresh_gazebo_processes(root))
    with (root / 'run/controller_health.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result))
    assert result['status'] == 'PASS'
    response = subprocess.run(['ros2', 'param', 'get', '/controller_manager', 'update_rate'],
                              capture_output=True, text=True, timeout=25)
    with (root / 'run/manager_rate.json').open('x') as stream:
        json.dump(dict(command_returncode=response.returncode, stdout=response.stdout,
                       stderr=response.stderr, status='PASS' if response.returncode == 0 and
                       response.stdout.strip() == 'Integer value is: 500' else 'FAIL'), stream, indent=2)
    verify_manager_rate_output(response.stdout, response.returncode)
    response = subprocess.run(['ros2', 'param', 'get', '/gz_ros_control', 'position_proportional_gain'],
                              capture_output=True, text=True, timeout=25)
    with (root / 'run/position_gain.json').open('x') as stream:
        json.dump(dict(command_returncode=response.returncode, stdout=response.stdout,
                       stderr=response.stderr, read_only_no_parameter_set=True), stream, indent=2)
    verify_gain_output(response.stdout, response.returncode)
    attempts = []
    for side in ('left', 'right'):
        for number in (1, 2):
            node = '/' + side + '_gripper_controller'
            parameter = f'gains.{side}_dex1_finger_{number}_joint.d'
            response = subprocess.run(['ros2', 'param', 'get', node, parameter], capture_output=True,
                                      text=True, timeout=10)
            attempts.append(dict(node=node, parameter=parameter, command_returncode=response.returncode,
                                 stdout=response.stdout, stderr=response.stderr))
    with (root / 'run/gripper_damping_readback.json').open('x') as stream:
        json.dump(dict(attempts=attempts, read_only_no_parameter_set=True), stream, indent=2)
    for attempt in attempts:
        verify_damping_output(attempt['stdout'], attempt['command_returncode'])
    from button_ramp import require_spline_output
    interpolation = []
    for side in ('left', 'right'):
        response = subprocess.run(['ros2', 'param', 'get', '/' + side + '_arm_controller', 'interpolation_method'],
                                  capture_output=True, text=True, timeout=10)
        interpolation.append(dict(side=side, stdout=response.stdout, stderr=response.stderr,
                                  command_returncode=response.returncode))
    with (root / 'run/button_interpolation_readback.json').open('x') as stream:
        json.dump(dict(attempts=interpolation, read_only_no_parameter_set=True), stream, indent=2)
    for attempt in interpolation:
        require_spline_output(attempt['stdout'], attempt['command_returncode'])


if __name__ == '__main__':
    main()

