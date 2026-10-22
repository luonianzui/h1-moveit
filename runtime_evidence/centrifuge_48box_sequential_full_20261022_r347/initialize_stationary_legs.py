import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from leg_initialization_contract import LEG_JOINTS, targets_from_original, endpoint_error
from wall_clock_contract import controllers_active

ROOT = Path(__file__).resolve().parent


def initialize(output):
    if output.exists():
        raise FileExistsError(str(output))
    plan_path = ROOT / 'inputs/plan.json'
    initial = json.loads(plan_path.read_text())['initial_positions']
    rclpy.init()
    node = rclpy.create_node('stationary_original_leg_initialization')
    actual = {}
    samples = []
    latest = dict(stamp=None, received=0.)
    start = time.monotonic()
    report = dict(status='FAIL', scope='fixed_pelvis_original_leg_targets_only',
                  original_plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest(),
                  no_grasp_lid_rotor_commands=True, no_reset_or_state_teleport=True)

    def receive(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        actual.update(zip(message.name, message.position))
        latest.update(stamp=stamp, received=time.monotonic())
        if set(LEG_JOINTS) <= actual.keys() and (not samples or stamp - samples[-1]['sim'] >= .02):
            samples.append(dict(sim=stamp, wall=time.monotonic(), positions=[actual[name] for name in LEG_JOINTS]))

    node.create_subscription(JointState, '/joint_states', receive, qos_profile_sensor_data)

    def wait(predicate, timeout=25):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Stationary initialization did not complete within wall budget')
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        wait(lambda: all(name in actual for name in initial))
        targets, measured = targets_from_original(initial, actual)
        report.update(before=dict(actual), before_sim=latest['stamp'], original_targets=targets)
        assert all(abs(actual[name] - value) < .015 for name, value in initial.items() if name not in LEG_JOINTS)
        command = ['ros2', 'run', 'controller_manager', 'spawner', 'leg_posture_controller',
                   '--controller-manager-timeout', '30', '--service-call-timeout', '30', '--switch-timeout', '30']
        with output.with_suffix('.controller.log').open('x') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            wait(lambda: process.poll() is not None, 40)
        assert process.returncode == 0
        client = ActionClient(node, FollowJointTrajectory, '/leg_posture_controller/follow_joint_trajectory')
        wait(client.server_is_ready)
        assert node.count_publishers('/leg_posture_controller/joint_trajectory') == 0
        wait(lambda: time.monotonic() - latest['received'] < .3)
        targets, measured = targets_from_original(initial, actual)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(LEG_JOINTS)
        first = JointTrajectoryPoint(positions=measured, velocities=[0.] * 12)
        first.time_from_start.nanosec = 50000000
        last = JointTrajectoryPoint(positions=targets, velocities=[0.] * 12)
        last.time_from_start.sec = 3
        last.time_from_start.nanosec = 50000000
        goal.trajectory.points = [first, last]
        goal.goal_tolerance = [JointTolerance(name=name, position=.005, velocity=.03) for name in LEG_JOINTS]
        goal.goal_time_tolerance.sec = 10
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        assert handle.accepted
        response = handle.get_result_async()
        wait(response.done, 30)
        result = response.result().result
        report.update(action_error_code=result.error_code, action_error_string=result.error_string)
        assert result.error_code == 0, result.error_string
        endpoint_stamp = latest['stamp']
        wait(lambda: latest['stamp'] > endpoint_stamp and endpoint_error(initial, actual) < .005)
        hold_start = latest['stamp']
        peak = 0.
        while latest['stamp'] - hold_start < 5.:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() - latest['received'] > .5 or time.monotonic() - start > 70:
                raise TimeoutError('Initialization hold feedback stale or timed out')
            error = endpoint_error(initial, actual)
            peak = max(peak, error)
            assert error < .005
            assert all(abs(actual[name] - value) < .015 for name, value in initial.items() if name not in LEG_JOINTS)
        response = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=10)
        assert response.returncode == 0 and controllers_active(response.stdout, ('leg_posture_controller',))
        report.update(status='PASS_STATIONARY_INITIALIZATION_ONLY', after=dict(actual), after_sim=latest['stamp'],
                      endpoint_error_rad=endpoint_error(initial, actual), hold_peak_error_rad=peak,
                      hold_sim_seconds=latest['stamp'] - hold_start, controller_stdout=response.stdout,
                      no_task_or_loaded_grip_certificate=True)
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        report.update(samples=samples, elapsed_wall_s=time.monotonic() - start)
        with output.open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        node.destroy_node()
        rclpy.shutdown()
    print(report['status'], report.get('error', ''), flush=True)
    return report['status'] == 'PASS_STATIONARY_INITIALIZATION_ONLY'


if __name__ == '__main__':
    assert len(sys.argv) == 2
    raise SystemExit(0 if initialize(Path(sys.argv[1])) else 1)

