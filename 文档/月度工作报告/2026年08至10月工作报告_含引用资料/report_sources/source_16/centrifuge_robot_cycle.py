#!/usr/bin/python3
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotCycle:
    def __init__(self, plan, evidence):
        self.plan = plan
        self.evidence = evidence
        self.node = rclpy.create_node('centrifuge_robot_press_cycle')
        self.trace = (evidence / 'robot_cycle_trace.jsonl').open('x')
        self.sim = None
        self.phase = 'preflight'
        self.positions = {}
        self.latest = {}
        self.values = {}
        self.events = []
        self.raw_count = 0
        self.deep_contact = False
        self.armed = False
        self.watch_run = False
        self.steady_start = None
        self.motion_count = {'left': 0, 'right': 0}
        self.last_joint_record = -1.0
        self.publishers = {
            side: self.node.create_publisher(JointTrajectory, '/' + side + '_arm_controller/joint_trajectory', 10)
            for side in ('left', 'right')}
        self.emergency_stop = self.node.create_publisher(Bool, '/centrifuge/rotor/enable', 10)
        self.node.create_subscription(Clock, '/clock', self.receive_clock, qos_profile_sensor_data)
        self.node.create_subscription(JointState, '/joint_states', self.receive_joint, qos_profile_sensor_data)
        for kind, topic in [('machine', '/centrifuge/machine/status'),
                            ('button', '/centrifuge/button/status'),
                            ('contact_status', '/centrifuge/button/contact_status'),
                            ('localization', '/mid360_localization/status')]:
            self.node.create_subscription(String, topic,
                lambda message, kind=kind: self.receive(kind, json.loads(message.data)), 10)
        self.node.create_subscription(String, '/centrifuge/button/contact_event',
                                      self.receive_event, 10)
        for button in ('lid', 'rotor'):
            self.node.create_subscription(Contacts, '/centrifuge/' + button + '_button/contacts',
                lambda message, button=button: self.raw_contacts(button, message),
                qos_profile_sensor_data)

    def record(self, kind, payload):
        self.trace.write(json.dumps(dict(kind=kind, phase=self.phase, sim_time=self.sim,
                                        wall_time=time.time(), payload=payload), allow_nan=False) + '\n')
        self.trace.flush()

    def receive_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.sim is not None and stamp < self.sim - .001:
            raise RuntimeError('Simulation clock moved backwards')
        self.sim = stamp
        self.latest['clock'] = time.monotonic()

    def receive_joint(self, message):
        self.positions.update(zip(message.name, message.position))
        self.latest['joint'] = time.monotonic()
        if self.sim is not None and self.sim - self.last_joint_record >= .05:
            self.last_joint_record = self.sim
            self.record('robot_joint', self.positions)

    def receive(self, kind, value):
        self.values[kind] = value
        self.latest[kind] = time.monotonic()
        self.record(kind, value)
        if (kind == 'machine' and self.watch_run and self.steady_start is None and
                value['rotor_enabled'] and 29.8 <= value['rotor_rpm'] <= 30.2):
            self.steady_start = self.sim
            self.record('steady_start', dict(sim_time=self.steady_start))

    def receive_event(self, message):
        event = json.loads(message.data)
        self.events.append(event)
        self.record('physical_press', event)

    def raw_contacts(self, button, message):
        if not message.contacts:
            return
        pairs = []
        for contact in message.contacts:
            depths = list(contact.depths)
            if any(not math.isfinite(depth) or depth > .002 for depth in depths):
                self.deep_contact = True
            pairs.append(dict(first=contact.collision1.name, second=contact.collision2.name,
                              depths=depths, force_norms=[
                                  math.sqrt(wrench.body_1_wrench.force.x ** 2 +
                                            wrench.body_1_wrench.force.y ** 2 +
                                            wrench.body_1_wrench.force.z ** 2)
                                  for wrench in contact.wrenches]))
        self.raw_count += 1
        self.record('raw_physics_contact', dict(button=button, pairs=pairs,
            sensor_stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9))

    def guard(self):
        if not self.armed:
            return
        now = time.monotonic()
        for kind, maximum in [('clock', 1), ('joint', 1), ('machine', 1),
                              ('button', 1), ('contact_status', 2), ('localization', 4)]:
            assert now - self.latest.get(kind, -math.inf) < maximum, 'Stale ' + kind
        assert self.values['machine']['ready']
        assert not self.values['machine']['open_lid_demo']
        assert self.values['button']['fault'] is None
        assert self.values['localization']['valid']
        assert not self.deep_contact, 'Button penetration exceeds 2 mm'
        for name, expected in self.plan['initial_positions'].items():
            if any(token in name for token in ('hip_', 'knee_', 'ankle_', 'torso_', 'dex1_')):
                tolerance = .003 if 'dex1_' in name else .02
                assert abs(self.positions[name] - expected) < tolerance, 'Uncommanded joint moved: ' + name
        for side in ('left', 'right'):
            assert self.node.count_publishers('/' + side + '_arm_controller/joint_trajectory') == 1

    def wait(self, predicate, wall_timeout=180):
        deadline = time.monotonic() + wall_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.01)
            self.guard()
            if predicate():
                return
        raise RuntimeError('Timeout in ' + self.phase)

    def move(self, side, points, fixed_interval=None, scheduled=None):
        self.guard()
        start = self.sim + .3 if scheduled is None else scheduled
        assert start >= self.sim + .15, 'Missed motion schedule'
        message = JointTrajectory()
        message.joint_names = self.plan['arms'][side]['joints']
        nanoseconds = round(start * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1000000000)
        elapsed = .05
        previous = None
        for values in points:
            positions = np.array(values)
            if previous is not None:
                elapsed += fixed_interval if fixed_interval is not None else max(
                    .02, float(np.max(np.abs(positions - previous))) / .22)
            point = JointTrajectoryPoint()
            point.positions = positions.tolist()
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(
                round(elapsed * 1e9), 1000000000)
            message.points.append(point)
            previous = positions
        self.motion_count[side] += 1
        self.record('arm_command', dict(side=side, start=start, duration=elapsed,
                                       points=[dict(positions=list(point.positions),
                                           seconds=point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)
                                           for point in message.points]))
        self.publishers[side].publish(message)
        target = dict(zip(message.joint_names, points[-1]))
        self.wait(lambda: self.sim >= start + elapsed + .1 and all(
            abs(self.positions.get(name, math.inf) - value) < .005 for name, value in target.items()))
        return start

    def press(self, side, return_home=True, scheduled=None):
        arm = self.plan['arms'][side]
        button = arm['button']
        if scheduled is None:
            self.phase = side + '_approach_' + button
            self.move(side, arm['approach'])
        else:
            assert all(abs(self.positions[name] - value) < .005
                       for name, value in zip(arm['joints'], arm['pre']))
        self.wait(lambda: self.values['contact_status']['buttons'][button]['armed'])
        previous = len(self.events)
        self.phase = side + '_press_' + button
        start = self.move(side, arm['push'], fixed_interval=.1, scheduled=scheduled)
        self.wait(lambda: len(self.events) > previous, wall_timeout=10)
        assert len(self.events) == previous + 1
        event = self.events[-1]
        assert event['button'] == button and event['source'] == 'gazebo_physics_contact'
        assert event['sequence'] == len(self.events)
        self.phase = side + '_withdraw_' + button
        self.move(side, list(reversed(arm['push'])), fixed_interval=.07)
        self.wait(lambda: self.values['contact_status']['buttons'][button]['armed'])
        if return_home:
            self.move(side, list(reversed(arm['approach'])))
        return event, start

    def lid_ready(self, angle):
        machine = self.values.get('machine', {})
        return (abs(machine.get('lid_rad', -100) - angle) < math.radians(.2) and
                not self.values.get('button', {}).get('busy', True))

    def execute(self):
        self.wait(lambda: self.sim is not None and all(kind in self.latest for kind in (
                      'clock', 'joint', 'machine', 'button', 'contact_status', 'localization')) and
                  all(name in self.positions for name in self.plan['initial_positions']) and
                  all(publisher.get_subscription_count() > 0 for publisher in self.publishers.values()))
        self.armed = True
        self.guard()
        assert not self.values['machine']['rotor_enabled'] and self.lid_ready(0)
        assert self.values['contact_status']['sequence'] == 0 and not self.events
        assert all(abs(self.positions[name] - value) < .02
                   for name, value in self.plan['initial_positions'].items())
        self.phase = 'robot_opens_for_initial_preparation'
        self.record('task_note', 'Fresh scene is closed; robot first opens it, then performs the requested close/run/stop/open cycle.')
        self.press('left')
        self.phase = 'wait_preparation_open'
        self.wait(lambda: self.lid_ready(math.radians(120)))
        self.record('requested_cycle_begin', dict(sim_time=self.sim))
        self.press('left')
        self.phase = 'wait_closed'
        self.wait(lambda: self.lid_ready(0) and self.values['machine']['can_start'])
        self.watch_run = True
        start_event, start_stamp = self.press('right', return_home=False)
        self.wait(lambda: self.steady_start is not None)
        observed_offset = start_event['sim_time'] - start_stamp
        stop_stamp = self.steady_start + 30.0 - observed_offset + .05
        self.phase = 'rotor_steady_30_sim_seconds'
        self.record('stop_schedule', dict(steady_start=self.steady_start, measured_press_offset=observed_offset,
                                          stop_stroke_start=stop_stamp, target_stop=self.steady_start + 30.0))
        self.wait(lambda: self.sim >= stop_stamp - .5)
        stop_event, _ = self.press('right', return_home=True, scheduled=stop_stamp)
        duration = stop_event['sim_time'] - self.steady_start
        assert 30.0 <= duration <= 30.3, 'Steady run duration not within 30.0..30.3 simulation seconds'
        self.phase = 'wait_rotor_stopped'
        self.wait(lambda: self.values['machine']['can_open'] and
                  not self.values['machine']['rotor_enabled'] and
                  abs(self.values['machine']['rotor_rad_s']) < .02)
        self.record('rotor_stopped_before_lid', self.values['machine'])
        self.press('left')
        self.phase = 'wait_final_open'
        self.wait(lambda: self.lid_ready(math.radians(120)) and self.values['machine']['can_open'])
        assert [event['button'] for event in self.events] == ['lid', 'lid', 'rotor', 'rotor', 'lid']
        assert self.values['machine']['starts'] == 1 and self.values['machine']['stops'] == 1
        result = dict(status='PASS', steady_run_sim_seconds=duration,
                      physical_press_events=self.events, raw_contact_messages=self.raw_count,
                      arm_command_counts=self.motion_count, final_machine=self.values['machine'],
                      final_robot_positions=self.positions, final_localization=self.values['localization'],
                      simulation_only=True, no_gripper_torso_leg_commands=True)
        with (self.evidence / 'robot_cycle_result.json').open('x') as stream:
            json.dump(result, stream, indent=2)
        self.record('PASS', result)
        print(json.dumps(result), flush=True)

    def stop_and_hold(self, error):
        self.record('FAIL_STOP_AND_HOLD', dict(error=str(error)))
        self.emergency_stop.publish(Bool(data=False))
        for side, publisher in self.publishers.items():
            names = self.plan['arms'][side]['joints']
            if not all(name in self.positions for name in names):
                continue
            message = JointTrajectory(joint_names=names)
            point = JointTrajectoryPoint(positions=[self.positions[name] for name in names])
            point.time_from_start.nanosec = 300000000
            message.points = [point]
            publisher.publish(message)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--execute-gazebo-only', action='store_true', required=True)
    arguments = parser.parse_args()
    plan = json.loads(arguments.plan.read_text())
    assert plan['status'] == 'PASS'
    for name, expected in plan['source_hashes'].items():
        assert hashlib.sha256((arguments.root / name).read_bytes()).hexdigest() == expected, name
    with (arguments.evidence / 'execution_claim.json').open('x') as stream:
        json.dump(dict(plan_sha256=hashlib.sha256(arguments.plan.read_bytes()).hexdigest(),
                       claimed_wall_time=time.time()), stream)
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    cycle = RobotCycle(plan, arguments.evidence)
    try:
        cycle.execute()
    except BaseException as error:
        cycle.stop_and_hold(error)
        raise
    finally:
        cycle.trace.close()
        cycle.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
