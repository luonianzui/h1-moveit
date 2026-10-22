#!/usr/bin/python3
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


COMMAND_TOPIC = '/centrifuge/lid/target_angle'
STATE_TOPIC = '/centrifuge/joint_states'
JOINT_NAME = 'lid_hinge_joint'


def load_limits():
    from ament_index_python.packages import get_package_share_directory
    model = Path(get_package_share_directory('h1_2_description'))
    model = ET.parse(model / 'models/centrifuge/model.sdf').getroot()
    joint = model.find(f"model/joint[@name='{JOINT_NAME}']")
    lower = float(joint.findtext('axis/limit/lower'))
    upper = float(joint.findtext('axis/limit/upper'))
    if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
        raise ValueError('Invalid installed lid limits')
    return lower, upper


def parse_target(value, lower, upper):
    if value == 'open':
        return upper
    if value == 'close':
        return lower
    target = math.radians(float(value))
    if not math.isfinite(target) or not lower <= target <= upper:
        raise ValueError(
            f'Angle must be {math.degrees(lower):g}..{math.degrees(upper):g} degrees')
    return target


def self_test(lower, upper):
    import rclpy
    import sys
    assert parse_target('open', lower, upper) == upper
    assert parse_target('close', lower, upper) == lower
    assert parse_target('60', lower, upper) == math.pi / 3
    rejected = []
    for value in ('-1', '121', '180', 'nan', 'inf', '-inf', 'invalid'):
        try:
            parse_target(value, lower, upper)
        except ValueError:
            rejected.append(value)
        else:
            raise AssertionError(f'Unsafe target accepted: {value}')
    print(json.dumps({
        'status': 'PASS', 'invalid_targets_rejected': rejected,
        'python_executable': sys.executable, 'rclpy_path': rclpy.__file__,
    }))


def run(target, lower, upper, timeout):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, String

    rclpy.init()
    node = rclpy.create_node(
        'centrifuge_lid_command',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    feedback = {}
    machine = {}

    def machine_status(message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload.get('can_open'), bool):
                return
            machine.update(payload=payload, received=time.monotonic())
        except (ValueError, TypeError):
            return

    def receive(message):
        if JOINT_NAME not in message.name:
            return
        index = message.name.index(JOINT_NAME)
        if index >= len(message.position) or index >= len(message.velocity):
            return
        position = message.position[index]
        velocity = message.velocity[index]
        if not (math.isfinite(position) and math.isfinite(velocity)):
            return
        feedback.update(
            position=position, velocity=velocity, received=time.monotonic(),
            stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    subscription = node.create_subscription(
        JointState, STATE_TOPIC, receive, qos_profile_sensor_data)
    machine_subscription = node.create_subscription(
        String, '/centrifuge/machine/status', machine_status, 10)
    publisher = None
    sent_command = False

    def hold_current():
        if publisher is not None and feedback and sent_command:
            held = min(upper, max(lower, feedback['position']))
            publisher.publish(Float64(data=held))
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        discovery_deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if feedback and (target is None or (
                    machine and node.count_subscribers(COMMAND_TOPIC) > 0)):
                break
        if not feedback:
            raise RuntimeError('No fresh centrifuge feedback; start the centrifuge scene first')
        if target is None:
            print(json.dumps({
                'angle_deg': math.degrees(feedback['position']),
                'velocity_rad_s': feedback['velocity'],
                'limits_deg': [math.degrees(lower), math.degrees(upper)],
            }))
            return
        if node.count_publishers(COMMAND_TOPIC) != 0:
            raise RuntimeError('Another lid command publisher is present')
        if not machine or time.monotonic() - machine['received'] > 0.5:
            raise RuntimeError('No fresh centrifuge interlock status')
        if target > math.radians(0.2) and not machine['payload']['can_open']:
            raise RuntimeError('Rotor must be stopped before opening the lid')
        if machine['payload'].get('open_lid_demo') and not machine['payload']['can_open']:
            raise RuntimeError('Rotor must be stopped before moving the lid in demo mode')
        if node.count_subscribers(COMMAND_TOPIC) == 0:
            raise RuntimeError('No lid command bridge')
        publisher = node.create_publisher(Float64, COMMAND_TOPIC, 10)
        connect_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= connect_deadline:
                raise RuntimeError('Lid command connection timed out')
        deadline = time.monotonic() + timeout
        last_publish = 0.0
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now - feedback['received'] > 2.0:
                raise RuntimeError('Lid feedback is stale')
            if now - machine['received'] > 1.0:
                raise RuntimeError('Interlock feedback is stale')
            if not lower - 0.005 <= feedback['position'] <= upper + 0.005:
                raise RuntimeError('Lid exceeded the joint envelope')
            if node.count_publishers(COMMAND_TOPIC) > 1:
                raise RuntimeError('Concurrent lid command detected')
            if now - last_publish >= 0.1:
                publisher.publish(Float64(data=target))
                sent_command = True
                last_publish = now
            settled = (
                abs(feedback['position'] - target) < math.radians(0.2)
                and abs(feedback['velocity']) < 0.01)
            if settled and sent_command:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.5:
                    print(json.dumps({
                        'status': 'PASS', 'target_deg': math.degrees(target),
                        'measured_deg': math.degrees(feedback['position']),
                        'velocity_rad_s': feedback['velocity'],
                    }))
                    return
            else:
                stable_since = None
        raise RuntimeError('Lid movement timed out')
    except (RuntimeError, KeyboardInterrupt):
        hold_current()
        raise
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Gazebo centrifuge lid only; angles in degrees, closed=0')
    parser.add_argument('target', nargs='?', default='status', help='status/open/close/degrees')
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--self-test', action='store_true')
    arguments = parser.parse_args()
    try:
        lower, upper = load_limits()
        if arguments.self_test:
            self_test(lower, upper)
            return
        if not math.isfinite(arguments.timeout) or arguments.timeout <= 0:
            raise ValueError('Timeout must be finite and positive')
        target = None if arguments.target == 'status' else parse_target(
            arguments.target, lower, upper)
        run(target, lower, upper, arguments.timeout)
    except (ValueError, RuntimeError) as error:
        parser.exit(1, f'centrifuge_lid: {error}\n')
    except KeyboardInterrupt:
        parser.exit(130, 'centrifuge_lid: interrupted; requested hold\n')


if __name__ == '__main__':
    main()
