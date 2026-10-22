#!/usr/bin/python3
import json
import math
from pathlib import Path
import signal
import subprocess
import time

from centrifuge_lid import COMMAND_TOPIC, JOINT_NAME, STATE_TOPIC, load_limits


def toggle_target(position, velocity, age, busy, fault, elapsed, lower, upper, writers):
    if fault:
        raise RuntimeError('FAULT_LATCHED')
    if busy:
        raise RuntimeError('BUSY')
    if elapsed < 0.5:
        raise RuntimeError('DEBOUNCE')
    if not all(math.isfinite(value) for value in (position, velocity, age)):
        raise RuntimeError('INVALID_FEEDBACK')
    if not 0.0 <= age <= 0.5:
        raise RuntimeError('STALE_FEEDBACK')
    if not lower - 0.005 <= position <= upper + 0.005:
        raise RuntimeError('OUTSIDE_ENVELOPE')
    if abs(velocity) >= 0.01:
        raise RuntimeError('LID_ALREADY_MOVING')
    if writers:
        raise RuntimeError('EXTERNAL_COMMAND')
    return 'close' if position >= (lower + upper) / 2 else 'open'


def rotor_request(machine, age, busy, fault):
    if not math.isfinite(age) or not math.isfinite(machine.get('rotor_rad_s', math.nan)):
        raise RuntimeError('INVALID_MACHINE_STATE')
    if machine.get('rotor_enabled') or abs(machine.get('rotor_rad_s', 0.0)) >= 0.02:
        return False
    if not machine.get('ready') or age > 0.5 or age < 0:
        raise RuntimeError('NO_FRESH_MACHINE_STATE')
    if busy or fault or not machine.get('can_start'):
        raise RuntimeError('ROTOR_START_NOT_READY')
    return True


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('centrifuge_button_controller')
    lower, upper = load_limits()
    feedback = {}
    machine = {}
    machine_received = -math.inf
    process = None
    requested = None
    last_press = -math.inf
    fault = None
    count = 0
    status = node.create_publisher(String, '/centrifuge/button/status', 10)
    rotor = node.create_publisher(Bool, '/centrifuge/rotor/enable', 10)
    heartbeat = node.create_publisher(Bool, '/centrifuge/machine/heartbeat', 10)

    def report(event, **details):
        payload = {
            'event': event, 'busy': process is not None, 'fault': fault,
            'completed': count, 'target': requested,
            'angle_deg': math.degrees(feedback['position']) if feedback else None,
            'machine': machine,
            **details,
        }
        status.publish(String(data=json.dumps(payload, allow_nan=False)))
        if event != 'STATE':
            node.get_logger().info(json.dumps(payload, allow_nan=False))

    def receive(message):
        if JOINT_NAME not in message.name:
            return
        index = message.name.index(JOINT_NAME)
        if index >= len(message.position) or index >= len(message.velocity):
            return
        position, velocity = message.position[index], message.velocity[index]
        if not (math.isfinite(position) and math.isfinite(velocity)):
            return
        feedback.update(position=position, velocity=velocity, received=time.monotonic())

    def press(message):
        nonlocal process, requested, last_press
        if message.data not in ('lid', 'rotor'):
            return
        now = time.monotonic()
        try:
            if now - last_press < 0.5:
                raise RuntimeError('DEBOUNCE')
            if message.data == 'rotor':
                run = rotor_request(machine, now - machine_received, process is not None, fault)
                rotor.publish(Bool(data=run))
                last_press = now
                report('ROTOR_REQUESTED', run=run)
                return
            if not feedback:
                raise RuntimeError('NO_FEEDBACK')
            target = toggle_target(
                feedback['position'], feedback['velocity'], now - feedback['received'],
                process is not None, fault, now - last_press, lower, upper,
                node.count_publishers(COMMAND_TOPIC))
            if now - machine_received > 0.5 or not machine.get('ready'):
                raise RuntimeError('NO_FRESH_MACHINE_STATE')
            if target == 'open' and not machine.get('can_open'):
                raise RuntimeError('STOP_ROTOR_AND_WAIT_BEFORE_OPEN')
            if machine.get('open_lid_demo') and not machine.get('can_open'):
                raise RuntimeError('STOP_ROTOR_AND_WAIT_BEFORE_LID_MOTION')
            process = subprocess.Popen(
                ['/usr/bin/python3', str(Path(__file__).resolve().with_name('centrifuge_lid.py')),
                 target, '--timeout', '90'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            requested = target
            last_press = now
            report('ACCEPTED')
        except (RuntimeError, OSError) as error:
            report('IGNORED', reason=str(error))

    def monitor():
        nonlocal process, fault, count
        heartbeat.publish(Bool(data=True))
        if process is not None and process.poll() is not None:
            output, _ = process.communicate()
            code = process.returncode
            process = None
            try:
                result = json.loads(output)
                if code != 0 or result.get('status') != 'PASS':
                    raise ValueError('helper did not report PASS')
                target = upper if requested == 'open' else lower
                if abs(math.radians(result['measured_deg']) - target) >= math.radians(0.2):
                    raise ValueError('helper target mismatch')
                count += 1
                report('COMPLETE', helper=result)
            except (ValueError, KeyError, TypeError):
                fault = 'LID_COMMAND_FAILED'
                report('FAULT', returncode=code, output=output)
        report('STATE')

    def machine_status(message):
        nonlocal machine, machine_received
        try:
            payload = json.loads(message.data)
            if not all(math.isfinite(payload[key]) for key in ('rotor_rad_s', 'lid_rad')):
                return
            machine = payload
            machine_received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    node.create_subscription(JointState, STATE_TOPIC, receive, qos_profile_sensor_data)
    node.create_subscription(String, '/centrifuge/button/action', press, 1)
    node.create_subscription(String, '/centrifuge/machine/status', machine_status, 10)
    node.create_timer(0.1, monitor)
    report('READY')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rotor.publish(Bool(data=False))
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=5)
                node.get_logger().info('Button shutdown: ' + output)
            except subprocess.TimeoutExpired:
                node.get_logger().error('Lid helper still stopping; no forced kill')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
