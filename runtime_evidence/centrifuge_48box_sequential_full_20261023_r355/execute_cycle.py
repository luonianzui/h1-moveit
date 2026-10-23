from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import traceback
from grip_limits import grip_exceeds_limits, CARRY_ANGLE_LIMIT_DEG

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from controller_health import command_health
from leg_posture import LEG_JOINTS, leg_posture_targets, posture_controller_active
from leg_endpoint import LegEndpointGate
from setdown_braking import braking_setdown_segments
from dispatch_gate import validate_dispatch
from empty_speed import motion_rate, minimum_interval, button_travel_rate, EMPTY_PHASES
from scipy.spatial.transform import Rotation
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_freshness import localization_freshness
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from execute_loading import LoadingCycle, GROUP_JOINTS, pose_matrix, fully_inside_bucket, midband_contacts
from alignment_wait import wait_for_planner
from loaded_retention import retention_metrics
from support_continuity import SupportContinuity
from table_return_goal import nearest_table_yaw, table_orientation_error
from soft_grasp import soft_grasp_profile
from extraction_clearance import bucket_rim_bounds, clearance_metrics
from terminal_policy import TERMINAL_POLICY, skip_final_parking, table_terminal_metrics, table_support_valid
from empty_return_policy import skip_verified_empty_return
from scene_contract import read_contract
from interrupt_control import establish_interrupt_handler, verify_interrupt_delivery
from free_carry_rate import carry_rate, carry_floor
from gripper_phase_context import observation_phase
from servo_model import verify_manager_rate_output
from startup_window import validate_startup_window

EVIDENCE = Path(__file__).resolve().parent


class FullCycle(LoadingCycle):
    def gripper_observation_phase(self, phase):
        return observation_phase(phase,getattr(self,'retrieval_closing',False))

    def __init__(self, plan, buttons):
        super().__init__(plan)
        self.buttons = buttons
        self.events = []
        self.leg_posture_active = False
        self.leg_posture_state_wall = 0.
        self.node.create_subscription(JointTrajectoryControllerState, '/leg_posture_controller/controller_state',
                                      self.receive_leg_posture_state, 10)
        self.deep_button_contact = False
        self.steady_start = None
        self.watch_rotor = False
        self.loaded = False
        self.last_retention_record = -1
        self.support_continuity = {payload:SupportContinuity(.02) for payload in ('left', 'right')}
        self.retention_checks = 0
        self.retention_peaks = {}
        self.previous_zero_force = {'left':False, 'right':False}
        self.held_payload = None
        self.return_hands = {}
        self.return_table_yaws = {}
        self.verified_table_payloads = set()
        self.completed_retrieval_retreats = set()
        self.bucket_rim_z = bucket_rim_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')
        self.active_clearance_payload = None
        self.last_clearance_record = -1
        self.localization_pose_sim = None
        self.localization_pose_wall = 0.0
        self.last_localization_health_record = -1.0
        self.phase = 'machine_preparation'
        self.node.create_subscription(PoseWithCovarianceStamped, '/mid360_localization/pose', self.receive_localization_pose, 10)
        self.emergency_stop = self.node.create_publisher(Bool, '/centrifuge/rotor/enable', 10)
        for kind, topic in (('button', '/centrifuge/button/status'),
                            ('contact_status', '/centrifuge/button/contact_status')):
            self.node.create_subscription(String, topic,
                lambda message, kind=kind: self.receive(kind, json.loads(message.data)), 10)
        self.node.create_subscription(String, '/centrifuge/button/contact_event', self.receive_event, 10)
        for button in ('lid', 'rotor'):
            self.node.create_subscription(Contacts, '/centrifuge/' + button + '_button/contacts',
                lambda message, button=button: self.receive_button_contacts(button, message), qos_profile_sensor_data)

    def receive_localization_pose(self, message):
        self.localization_pose_sim = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.localization_pose_wall = time.monotonic()
        self.latest['localization_pose'] = self.localization_pose_wall
        self.record('localization_pose', dict(source_sim=self.localization_pose_sim))

    def localization_is_current(self, now):
        health = localization_freshness(self.sim, self.localization_pose_sim, now, self.localization_pose_wall,
                                       self.latest.get('localization', 0), self.values.get('localization', {}).get('valid'))
        if not health['valid'] or self.sim - self.last_localization_health_record >= .25:
            self.last_localization_health_record = self.sim
            self.record('localization_health', health)
        return health['valid']

    def record(self, kind, payload):
        if kind == 'PASS' and payload.get('scope') == 'two_box_overhead_loading':
            kind = 'LOADING_PASS'
        super().record(kind, payload)

    def gripper_goal_time_tolerance(self, side, phase):
        return 6 if phase == side + '_grasp' else super().gripper_goal_time_tolerance(side, phase)

    def gripper_action_timeout_s(self, side, phase):
        return 240 if getattr(self, 'retrieval_closing', False) else super().gripper_action_timeout_s(side, phase)

    def gripper_trajectory_points(self, side, value, phase, effort):
        if not getattr(self, 'retrieval_closing', False):
            return super().gripper_trajectory_points(side, value, phase, effort)
        assert phase == side + '_grasp' and effort == 16
        names = GROUP_JOINTS[side + '_gripper']
        profile = soft_grasp_profile([self.positions[name] for name in names], value,
                                     self.accepted_gripper_efforts.get(side, 0), effort)
        points = []
        for sample in profile:
            point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'],
                                         effort=sample['effort'])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
            points.append(point)
        self.record('retrieval_soft_grasp_profile', dict(side=side, profile=profile, original_quiet_rpm=.02))
        return points

    def minimum_waypoint_interval(self, phase):
        return carry_floor(phase,getattr(self,'holding',None),getattr(self,'held_payload',None),
                           getattr(self,'accepted_gripper_efforts',{}))

    def move(self, samples, phase, rate):
        self.phase = phase
        if phase in ('left_retrieval_pilot_lift', 'right_retrieval_pilot_lift'):
            requested = rate
            rate = min(rate, .03)
            if rate != requested:
                self.record('retrieval_pilot_retime', dict(phase=phase, original_rate_rad_s=requested,
                    effective_rate_rad_s=rate, unchanged_path=True, original_quiet_rpm=.02))
        original_rate = rate
        holding = getattr(self, 'holding', None)
        held_payload = getattr(self, 'held_payload', None)
        rate = motion_rate(phase, rate, holding, held_payload)
        if rate != original_rate:
            self.record('empty_motion_retime', dict(phase=phase, original_rate_rad_s=original_rate,
                effective_rate_rad_s=rate, minimum_interval_s=self.minimum_waypoint_interval(phase),
                holding=holding, held_payload=held_payload, unchanged_path=True))
        effective_rate = min(rate, .065) if phase in ('left_transfer', 'right_lower') else rate
        if phase in ('left_retrieval_transfer', 'right_retrieval_transfer'):
            effective_rate = min(rate, .10)
        if effective_rate != rate:
            kind = 'retrieval_carry_retime' if '_retrieval_transfer' in phase else 'loading_transfer_retime'
            self.record(kind, dict(phase=phase, original_rate_rad_s=rate,
                        effective_rate_rad_s=effective_rate, unchanged_path=True))
        if phase in ('left_retrieval_setdown', 'right_retrieval_setdown'):
            coarse, fine, split = braking_setdown_segments(samples)
            self.record('retrieval_setdown_braking', dict(phase=phase, split_index=split,
                original_points=len(samples), unchanged_path=True, final_nominal_clearance_m=.010,
                coarse_rate_rad_s=effective_rate, fine_rate_rad_s=.03, original_endpoint_rad=.0003))
            super().move(coarse, phase, effective_rate)
            return super().move(fine, phase, .03)
        proposed=carry_rate(phase,effective_rate,getattr(self,'holding',None),
                           getattr(self,'held_payload',None),getattr(self,'accepted_gripper_efforts',{}))
        if proposed!=effective_rate:
            self.record('free_carry_retime',dict(phase=phase,original_rate_rad_s=effective_rate,
                        effective_rate_rad_s=proposed,original_floor_sim_s=.020,new_floor_sim_s=.020,
                        holding=self.holding,held_payload=getattr(self,'held_payload',None),
                        geometry_and_original_dense_FCL_samples_unchanged=True,
                        tracking_rad=.015,slip_m=.002,carry_deg=5.))
        return super().move(samples, phase, proposed)

    def receive(self, kind, value):
        super().receive(kind, value)
        if kind == 'machine' and self.watch_rotor and self.steady_start is None and value['rotor_enabled'] and 29.8 <= value['rotor_rpm'] <= 30.2:
            self.steady_start = self.sim
            self.record('steady_start', dict(sim_time=self.sim))

    def receive_event(self, message):
        event = json.loads(message.data)
        self.events.append(event)
        self.record('physical_press', event)

    def receive_button_contacts(self, button, message):
        if not message.contacts:
            return
        pairs = []
        for contact in message.contacts:
            depths = list(contact.depths)
            if any(not math.isfinite(depth) or depth > .002 for depth in depths):
                self.deep_button_contact = True
            pairs.append(dict(first=contact.collision1.name, second=contact.collision2.name, depths=depths))
        self.record('raw_button_contact', dict(button=button, pairs=pairs,
                    sensor_sim=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9))

    def safety(self):
        hand = self.holding
        if self.held_payload is not None:
            self.holding = None
        try:
            super().safety()
        finally:
            self.holding = hand
        if self.leg_posture_active and time.monotonic() - self.leg_posture_state_wall > 2:
            raise RuntimeError('Leg posture controller state stale')
        machine = self.values.get('machine', {})
        if machine.get('stopped_velocity_gain') != 80 or machine.get('stopped_position_gain') != 8000 or not math.isfinite(machine.get('motor_torque_nm', math.inf)) or abs(machine['motor_torque_nm']) > 2.000001:
            raise RuntimeError('Experimental finite torque brake configuration invalid')
        if self.phase.find('_retrieval_') >= 0 and not machine.get('brake_active', False):
            raise RuntimeError('Encoder parking reference missing during retrieval')
        if self.active_clearance_payload is not None:
            payload = self.active_clearance_payload
            clearance = clearance_metrics(self.poses, payload, self.contacts[payload], self.bucket_rim_z[payload])
            if not clearance['valid']:
                self.record('extraction_clearance_fault', dict(payload=payload, clearance=clearance, poses=deepcopy(self.poses), contacts=self.contacts[payload]))
                raise RuntimeError('Full extraction clearance or no-basket-contact lost')
            if self.pose_time - self.last_clearance_record >= .025:
                self.last_clearance_record = self.pose_time
                self.record('extraction_clearance', dict(payload=payload, clearance=clearance))
        if self.deep_button_contact:
            raise RuntimeError('Button penetration exceeds 2 mm')
        if 'button' in self.values and self.values['button']['fault'] is not None:
            raise RuntimeError('Button controller fault')
        if self.phase.startswith('machine_'):
            for kind in ('button', 'contact_status'):
                if time.monotonic() - self.latest.get(kind, 0) > 2:
                    raise RuntimeError('Stale physical button ' + kind)
            if self.loaded:
                for payload in ('left', 'right'):
                    metrics = retention_metrics(self.poses, self.contacts[payload], payload)
                    continuity = self.support_continuity[payload].update(self.sim, metrics)
                    metrics['support_continuity'] = continuity
                    self.retention_checks += 1
                    for key in ('side_overhang_m', 'bottom_penetration_m', 'maximum_contact_depth_m'):
                        self.retention_peaks[payload + '_' + key] = max(self.retention_peaks.get(payload + '_' + key, 0), metrics[key])
                    if not metrics['supported'] or self.previous_zero_force[payload]:
                        self.record('support_continuity_sample', dict(payload=payload, metrics=metrics, poses=self.poses, contacts=self.contacts[payload]))
                    self.previous_zero_force[payload] = not metrics['supported']
                    if not continuity['valid']:
                        self.record('loaded_containment_fault', dict(payload=payload, poses=self.poses, metrics=metrics,
                                    contacts=self.contacts[payload]))
                        raise RuntimeError('Loaded payload left its basket during machine cycle')
                    if any('dex1' in pair['first'] + pair['second'] for pair in self.contacts[payload]):
                        raise RuntimeError('Finger contacting loaded payload during machine cycle')
                if self.pose_time - self.last_retention_record >= .025:
                    self.last_retention_record = self.pose_time
                    self.record('loaded_retention', dict(poses=self.poses, contacts=self.contacts,
                                metrics={payload: retention_metrics(self.poses, self.contacts[payload], payload)
                                         for payload in ('left', 'right')}))
        if self.held_payload is not None:
            expected = self.wrist(hand) @ self.grasp_transform
            actual = pose_matrix(self.poses['box48_' + self.held_payload])
            native = pose_matrix(self.poses[hand + '_wrist_yaw_link']) @ self.native_grasp_transform
            slip = float(np.linalg.norm(expected[:3, 3] - actual[:3, 3]))
            native_slip = float(np.linalg.norm(native[:3, 3] - actual[:3, 3]))
            angle = math.degrees(Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude())
            native_angle = math.degrees(Rotation.from_matrix(native[:3, :3].T @ actual[:3, :3]).magnitude())
            self.max_slip = max(self.max_slip, slip)
            self.max_native_slip = max(self.max_native_slip, native_slip)
            if self.pose_time - self.last_grip_record > .02:
                self.last_grip_record = self.pose_time
                self.record('retrieval_grip', dict(hand=hand, payload=self.held_payload, fk_slip_m=slip,
                            native_slip_m=native_slip, fk_angle_deg=angle, native_angle_deg=native_angle))
            if grip_exceeds_limits(max(slip, native_slip), max(angle, native_angle)):
                raise RuntimeError('Retrieval grip exceeds 2 mm / authorized 5 degree gate')

    def wait(self, predicate, timeout_s=600):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin()
            if predicate():
                return
        raise TimeoutError('Cycle wait: ' + self.phase)

    def move_button(self, side, points, fixed_interval=None, scheduled=None):
        self.safety()
        travel_rate = button_travel_rate(self.holding, self.held_payload)
        if fixed_interval is None:
            self.record('empty_button_retime', dict(side=side, original_rate_rad_s=.22,
                effective_rate_rad_s=travel_rate, unchanged_path=True, holding=self.holding,
                held_payload=self.held_payload, fixed_contact_intervals_unchanged=True))
        start = self.sim + .3 if scheduled is None else scheduled
        assert start >= self.sim + .15, 'Missed physical button schedule'
        arm = self.buttons['arms'][side]
        message = JointTrajectory(joint_names=arm['joints'])
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(start * 1e9), 1000000000)
        elapsed = .05
        previous = None
        for values in points:
            positions = np.array(values)
            if previous is not None:
                elapsed += fixed_interval if fixed_interval is not None else max(.02, float(np.max(np.abs(positions - previous))) / travel_rate)
            point = JointTrajectoryPoint(positions=positions.tolist())
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(elapsed * 1e9), 1000000000)
            message.points.append(point)
            previous = positions
        self.publishers[side + '_arm'].publish(message)
        self.motion_count += 1
        self.record('button_arm_command', dict(side=side, start=start, duration=elapsed,
                    points=[dict(positions=list(point.positions), seconds=point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)
                            for point in message.points]))
        target = dict(zip(arm['joints'], points[-1]))
        self.wait(lambda: self.sim >= start + elapsed + .1 and
                  all(abs(self.positions[name] - value) < .005 for name, value in target.items()))
        return start

    def press(self, side, phase, return_home=True, scheduled=None):
        arm = self.buttons['arms'][side]
        button = arm['button']
        self.phase = phase + '_approach'
        if scheduled is None:
            self.move_button(side, arm['approach'])
        else:
            assert all(abs(self.positions[name] - value) < .005 for name, value in zip(arm['joints'], arm['pre']))
        self.wait(lambda: self.values['contact_status']['buttons'][button]['armed'])
        previous = len(self.events)
        self.phase = phase + '_press'
        start = self.move_button(side, arm['push'], fixed_interval=.1, scheduled=scheduled)
        self.wait(lambda: len(self.events) > previous, timeout_s=15)
        assert len(self.events) == previous + 1
        event = self.events[-1]
        assert event['button'] == button and event['source'] == 'gazebo_physics_contact'
        assert event['sequence'] == len(self.events)
        self.phase = phase + '_withdraw'
        self.move_button(side, list(reversed(arm['push'])), fixed_interval=.07)
        self.wait(lambda: self.values['contact_status']['buttons'][button]['armed'])
        if return_home:
            self.move_button(side, list(reversed(arm['approach'])))
        return event, start

    def lid_ready(self, angle):
        return (abs(self.values['machine']['lid_rad'] - angle) < math.radians(.2) and
                not self.values['button']['busy'])

    def initialize_leg_posture(self):
        assert self.holding is None and self.held_payload is None
        targets, measured = leg_posture_targets(self.plan['initial_positions'], self.positions)
        self.phase = 'machine_preparation_leg_posture'
        self.record('leg_posture_declared', dict(joint_names=LEG_JOINTS, targets=targets,
                    measured=measured, smooth_duration_sim_s=1, original_smooth_duration_sim_s=5,
                    unchanged_deviation_gate_rad=.025,
                    scope='fixed_pelvis_stationary_manipulation_not_free_gait'))
        from leg_initialization_runtime import verify_initialization
        verify_initialization(EVIDENCE, self.plan['initial_positions'], self.positions)
        command = ['ros2', 'control', 'list_controllers']
        with (EVIDENCE / 'run/leg_controller_load.log').open('x') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            wait_for_planner(process, self.spin, timeout_s=60)
        assert process.returncode == 0
        status_path = EVIDENCE / 'run/leg_controller_status.log'
        with status_path.open('x') as stream:
            process = subprocess.Popen(['ros2', 'control', 'list_controllers'], stdout=stream,
                                       stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            wait_for_planner(process, self.spin, timeout_s=60)
        assert process.returncode == 0 and posture_controller_active(status_path.read_text())
        assert command_health(process.returncode, status_path.read_text())['status'] == 'PASS'
        self.wait(lambda: self.leg_posture_state_wall > 0 and time.monotonic() - self.leg_posture_state_wall < 2)
        assert self.node.count_publishers('/leg_posture_controller/joint_trajectory') == 0
        self.leg_posture_active = True
        self.leg_posture_client = ActionClient(self.node, FollowJointTrajectory, '/leg_posture_controller/follow_joint_trajectory')
        self.wait(lambda: self.leg_posture_client.server_is_ready())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(LEG_JOINTS)
        first = JointTrajectoryPoint(positions=[self.positions[name] for name in LEG_JOINTS], velocities=[0.] * 12)
        first.time_from_start.nanosec = 50000000
        last = JointTrajectoryPoint(positions=targets, velocities=[0.] * 12)
        last.time_from_start.sec = 1
        last.time_from_start.nanosec = 50000000
        goal.trajectory.points = [first, last]
        goal.goal_tolerance = [JointTolerance(name=name, position=.005, velocity=.03) for name in LEG_JOINTS]
        goal.goal_time_tolerance.sec = 10
        future = self.leg_posture_client.send_goal_async(goal)
        self.wait(future.done)
        handle = future.result()
        assert handle.accepted
        self.record('leg_posture_goal_accepted', dict(joint_names=LEG_JOINTS, targets=targets))
        future = handle.get_result_async()
        self.wait(future.done)
        response = future.result().result
        assert response.error_code == 0, response.error_string
        endpoint = LegEndpointGate(targets, self.joint_time, self.sim, position_limit_rad=.005)
        self.record('leg_endpoint_fresh_wait_declared', dict(previous_joint_stamp=self.joint_time,
                    original_position_limit_rad=.005, maximum_wait_sim_s=3., maximum_wait_wall_s=30.))
        self.wait(lambda: endpoint.observe(self.joint_time, self.sim, self.positions), timeout_s=30)
        self.record('leg_endpoint_fresh_wait_pass', dict(actual_joint_stamp=self.joint_time,
                    maximum_actual_error_rad=endpoint.maximum_error_rad,
                    original_position_limit_rad=.005, new_actual_feedback_required=True))
        assert max(abs(self.positions[name] - target) for name, target in zip(LEG_JOINTS, targets)) < .005
        self.dwell(1)
        self.record('leg_posture_pass', dict(joint_names=LEG_JOINTS, targets=targets,
                    measured=[self.positions[name] for name in LEG_JOINTS], original_deviation_gate_rad=.025,
                    controller='leg_posture_controller', no_extra_fixed_joint=True))

    def receive_leg_posture_state(self, message):
        self.leg_posture_state_wall = time.monotonic()

    def prepare_open(self):
        deadline = time.monotonic() + 120
        ready_since = None
        while time.monotonic() < deadline:
            self.spin(False)
            if len(self.poses) == 13 and all(kind in self.latest for kind in (
                    'clock', 'joints', 'machine', 'localization', 'localization_pose', 'left_contacts', 'right_contacts', 'button', 'contact_status')) and all(
                    name in self.positions and abs(self.positions[name] - value) < .010
                    for name, value in self.plan['initial_positions'].items()):
                ready_since = ready_since or time.monotonic()
                if time.monotonic() - ready_since >= 2:
                    break
            else:
                ready_since = None
        else:
            raise TimeoutError('Passive initial posture did not settle; no task motion issued')
        self.record('passive_posture_settled', dict(positions=self.positions, minimum_stable_wall_s=2,
                    initial_error_limit_rad=.010))
        self.safety()
        self.record('fresh_startup_window_pass', validate_startup_window(self.sim))
        self.initialize_leg_posture()
        assert self.lid_ready(0) and not self.values['machine']['rotor_enabled']
        assert self.values['contact_status']['sequence'] == 0
        assert all(abs(self.positions[name] - value) < .015 for name, value in self.plan['initial_positions'].items())
        for group, publisher in self.publishers.items():
            assert self.node.count_publishers('/' + group + '_controller/joint_trajectory') == 1
            assert publisher.get_subscription_count() == 1
        self.press('left', 'machine_preparation_open')
        self.phase = 'machine_wait_preparation_open'
        self.wait(lambda: self.lid_ready(math.radians(120)))
        self.phase = 'preflight'

    def return_neutral(self):
        for side in ('left', 'right'):
            park = next(stage for stage in self.plan['stages'] if stage['side'] == side and stage['phase'] == 'park')
            self.move(park['samples'][::-1], 'machine_return_neutral_' + side, .25)
            self.gripper(side, -.019, 'machine_empty_gripper_' + side)

    def machine_cycle(self):
        self.loaded = True
        self.return_neutral()
        self.press('left', 'machine_close')
        self.phase = 'machine_wait_closed'
        self.wait(lambda: self.lid_ready(0) and self.values['machine']['can_start'])
        self.record('closed_loaded_pass', self.values['machine'])
        self.watch_rotor = True
        start_event, start_stamp = self.press('right', 'machine_start', return_home=False)
        self.phase = 'machine_running_steady'
        self.wait(lambda: self.steady_start is not None)
        observed_offset = start_event['sim_time'] - start_stamp
        stop_stamp = self.steady_start + 30.0 - observed_offset + .05
        self.record('stop_schedule', dict(steady_start=self.steady_start, stop_stroke_start=stop_stamp,
                    measured_press_offset=observed_offset, target_stop=self.steady_start + 30.0))
        self.wait(lambda: self.sim >= stop_stamp - .5)
        stop_event, _ = self.press('right', 'machine_stop', scheduled=stop_stamp)
        duration = stop_event['sim_time'] - self.steady_start
        assert 30 <= duration <= 30.3, duration
        self.phase = 'machine_stop_wait_quiet'
        self.wait(lambda: self.values['machine']['can_open'] and not self.values['machine']['rotor_enabled'] and
                  abs(self.values['machine']['rotor_rad_s']) < .02)
        self.record('rotor_stopped_before_lid', self.values['machine'])
        self.press('left', 'machine_final_open')
        self.phase = 'machine_wait_final_open'
        self.wait(lambda: self.lid_ready(math.radians(120)))
        self.dwell(3)
        self.record('MACHINE_CYCLE_PASS', dict(steady_run_sim_seconds=duration, final_machine=self.values['machine'], retention_checks=self.retention_checks, retention_peaks=self.retention_peaks, maximum_zero_force_gap_sim_s={payload:guard.maximum_gap_s for payload,guard in self.support_continuity.items()}))
        self.loaded = False
        self.phase = 'retrieval_preparation'
        for side in ('left', 'right'):
            self.gripper(side, -.0199, side + '_retrieval_open_empty')

    def plan_retrieval(self, hand, payload, mode):
        self.phase = hand + '_retrieval_' + mode + '_planning'
        self.dwell(.5)
        request = dict(hand=hand, payload=payload, mode=mode, positions=dict(self.positions), poses=deepcopy(self.poses))
        if mode == 'transport':
            request['grasp_transform'] = self.grasp_transform.tolist()
        prefix = payload + '_' + hand + '_' + mode
        request_path = EVIDENCE / 'run' / (prefix + '_request.json')
        output = EVIDENCE / 'run' / (prefix + '_plan.json')
        with request_path.open('x') as stream:
            json.dump(request, stream, indent=2)
        with (EVIDENCE / 'run' / (prefix + '_worker.log')).open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'retrieval_worker.py'), str(request_path), str(output)],
                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=480)
        result = json.loads(output.read_text())
        assert result['status'] == 'PASS' and result['maximum_joint_sampling_rad'] == .002
        initial = result['stages'][0]['samples'][0]
        assert max(abs(self.positions[name] - value) for name, value in initial.items()) < .001
        actual = pose_matrix(self.poses['box48_' + payload])
        before = pose_matrix(request['poses']['box48_' + payload])
        assert np.linalg.norm(actual[:3, 3] - before[:3, 3]) < .0003
        assert Rotation.from_matrix(before[:3, :3].T @ actual[:3, :3]).magnitude() < math.radians(.1)
        self.record('retrieval_plan_pass', result)
        if mode == 'transport':
            assert result['pilot_method'] == 'world_vertical_radial_centered' and result['pilot_distance_m'] == .015
            yaw = nearest_table_yaw(actual)
            assert abs(result['return_table_yaw_rad'] - yaw) < 1e-10
            assert result['carry_method'] == 'cartesian_upright' and result['transfer_body_height_m'] == 1.180
            self.return_table_yaws[payload] = yaw
            self.record('return_table_target', dict(hand=hand, payload=payload, yaw_rad=yaw,
                        expected_position=[.44, .38 if hand == 'left' else -.38, .84475],
                        grip_angle_limit_deg=CARRY_ANGLE_LIMIT_DEG, final_orientation_limit_deg=2))
        return result

    def grip_retrieval(self, hand, payload):
        self.retrieval_closing = True
        try:
            self.gripper(hand, -.01901, hand + '_grasp')
        finally:
            self.retrieval_closing = False
        self.dwell(.25)
        pairs = self.contacts[payload]
        loads = {}
        for number in (1, 2):
            assert any(hand + f'_dex1_finger_{number}_' in pair['first'] + pair['second'] for pair in pairs)
            loads[number] = sum(abs(float(np.dot(force, normal))) for pair in pairs
                if hand + f'_dex1_finger_{number}_' in pair['first'] + pair['second']
                for force, normal in zip(pair['force_vectors'], pair['normals']))
        assert all(3 <= force <= 20.5 for force in loads.values()), loads
        aliases = dict(self.poses)
        aliases['box48_' + hand] = self.poses['box48_' + payload]
        self.record('retrieval_midband_contact_pass', dict(hand=hand, payload=payload,
                    loads_N=loads, fingers=midband_contacts(aliases, pairs, hand)))
        self.holding = hand
        self.held_payload = payload
        self.return_hands[payload] = hand
        body = pose_matrix(self.poses['box48_' + payload])
        self.grasp_transform = np.linalg.inv(self.wrist(hand)) @ body
        self.native_grasp_transform = np.linalg.inv(pose_matrix(self.poses[hand + '_wrist_yaw_link'])) @ body

    def verify_table(self, payload):
        hand = self.return_hands[payload]
        expected = np.array([.44, .38 if hand == 'left' else -.38, .84475])
        self.dwell(.5)
        assert np.linalg.norm(pose_matrix(self.poses['box48_' + payload])[:3, 3] - expected) < .002
        assert any('table::' in pair['first'] + pair['second'] for pair in self.contacts[payload])
        assert table_orientation_error(pose_matrix(self.poses['box48_' + payload]), self.return_table_yaws[payload]) < math.radians(2)
        hand = self.holding
        self.holding = None
        self.held_payload = None
        self.gripper(hand, -.0199, hand + '_release_on_table')
        start = self.sim
        reference = pose_matrix(self.poses['box48_' + payload])
        while self.sim - start < 3:
            self.spin()
            pairs = self.contacts[payload]
            assert any('table::' in pair['first'] + pair['second'] for pair in pairs)
            assert not any('dex1' in pair['first'] + pair['second'] for pair in pairs)
            body = pose_matrix(self.poses['box48_' + payload])
            assert np.linalg.norm(body[:3, 3] - expected) < .002
            assert np.linalg.norm(body[:3, 3] - reference[:3, 3]) < .0005
            assert table_orientation_error(body, self.return_table_yaws[payload]) < math.radians(2)
        self.record('table_return_pass', dict(payload=payload, duration_sim_s=self.sim - start,
                    hand=hand, expected_position=expected.tolist(), expected_yaw_rad=self.return_table_yaws[payload],
                    pose=self.poses['box48_' + payload]))
        self.verified_table_payloads.add(payload)

    def terminal_hold(self, duration_sim_s=10):
        assert self.holding is None and self.held_payload is None
        assert all(self.accepted_gripper_efforts.get(side) == 0 for side in ('left', 'right'))
        self.phase = 'terminal_table_hold'
        references = {payload: pose_matrix(self.poses['box48_' + payload])[:3, 3].copy() for payload in ('left', 'right')}
        last_positive = {payload: None for payload in ('left', 'right')}
        start = self.sim
        last_record = -1
        peaks = {}
        while self.sim - start < duration_sim_s:
            self.spin()
            rows = {}
            for payload in ('left', 'right'):
                hand = self.return_hands[payload]
                expected = [.44, .38 if hand == 'left' else -.38, .84475]
                metrics = table_terminal_metrics(self.poses, payload, self.contacts[payload],
                    expected, self.return_table_yaws[payload], references[payload])
                if metrics['supported']:
                    last_positive[payload] = self.sim
                continuity = table_support_valid(metrics, self.sim, last_positive[payload])
                if not continuity['valid']:
                    self.record('terminal_table_hold_fault', dict(payload=payload, metrics=metrics, continuity=continuity,
                                poses=deepcopy(self.poses), contacts=self.contacts[payload]))
                    raise RuntimeError('Terminal table support/pose/contact invalid')
                for field in ('position_error_m', 'drift_m', 'orientation_error_deg'):
                    peaks[payload + '_' + field] = max(peaks.get(payload + '_' + field, 0), metrics[field])
                rows[payload] = dict(metrics=metrics, continuity=continuity)
            leg_error = max(abs(self.positions[name] - initial) for name, initial in self.plan['initial_positions'].items()
                            if any(part in name for part in ('hip_', 'knee_', 'ankle_')))
            peaks['passive_leg_deviation_rad'] = max(peaks.get('passive_leg_deviation_rad', 0), leg_error)
            if self.pose_time - last_record >= .025:
                last_record = self.pose_time
                self.record('terminal_table_hold_sample', dict(rows=rows, passive_leg_deviation_rad=leg_error,
                            poses=deepcopy(self.poses), contacts=self.contacts))
        self.record('terminal_table_hold_pass', dict(duration_sim_s=self.sim - start, policy=TERMINAL_POLICY,
                    peaks=peaks, no_held_payload=True, neutral_arms_not_claimed=True))

    def retrieve(self):
        order = sorted(('left', 'right'), key=lambda payload: pose_matrix(self.poses['box48_' + payload])[1, 3])
        for payload in order:
            hand = 'left' if pose_matrix(self.poses['box48_' + payload])[1, 3] >= 0 else 'right'
            pickup = self.plan_retrieval(hand, payload, 'pickup')
            for stage in pickup['stages']:
                self.move(stage['samples'], hand + '_retrieval_' + stage['phase'], .18)
            self.grip_retrieval(hand, payload)
            transport = self.plan_retrieval(hand, payload, 'transport')
            for stage in transport['stages']:
                phase = stage['phase']
                self.phase = hand + '_retrieval_' + phase
                if skip_verified_empty_return(
                        payload == order[-1], phase, self.holding, self.held_payload,
                        payload in self.verified_table_payloads,
                        payload in self.completed_retrieval_retreats):
                    self.record('verified_empty_return_skipped', dict(
                        hand=hand, payload=payload, phase=phase, holding=self.holding,
                        held_payload=self.held_payload, verified_table=True, retreat_complete=True,
                        next_planning='fresh_actual_full_scene_IK_FCL', neutral_arms_not_claimed=True))
                    continue
                if skip_final_parking(payload == order[-1], phase):
                    self.record('optional_parking_skipped', dict(payload=payload, phase=phase, policy=TERMINAL_POLICY))
                    continue
                self.move(stage['samples'], hand + '_retrieval_' + phase, .12 if phase in ('pilot_lift', 'setdown') else .20)
                if phase == 'retreat':
                    self.completed_retrieval_retreats.add(payload)
                if phase == 'lift':
                    clearance = clearance_metrics(self.poses, payload, self.contacts[payload], self.bucket_rim_z[payload])
                    assert clearance['valid'], clearance
                    self.active_clearance_payload = payload
                    self.phase = hand + '_retrieval_static_grip_hold'
                    try:
                        self.dwell(10)
                    finally:
                        self.active_clearance_payload = None
                    self.record('retrieval_static_grip_hold_pass', dict(hand=hand, payload=payload, duration_sim_s=10, clearance=clearance,
                                hold_after_full_lift=True, pilot_not_claimed_fully_extracted=True))
                if phase == 'setdown':
                    self.verify_table(payload)
        self.terminal_hold(10)
        assert self.holding is None
        assert set(self.return_hands.values()) == {'left', 'right'}
        assert [event['button'] for event in self.events] == ['lid', 'lid', 'rotor', 'rotor', 'lid']
        assert self.values['machine']['starts'] == 1 and self.values['machine']['stops'] == 1
        self.record('PASS', dict(scope='two_box_load_close_spin_open_retrieve',
                    physical_press_events=self.events, final_machine=self.values['machine'], poses=self.poses,
                    motions=self.motion_count, max_slip_m=self.max_slip, max_native_slip_m=self.max_native_slip))

    def run_complete(self):
        self.record('simulation_relaxation_declared', dict(user_authorized=True,
            user_request='可放宽限制，先完成全套流程再优化', original_retrieval_quiet_rpm=.02,
            retrieval_transient_limit_rpm=.1, original_machine_stop_rad_s=.02, final_quiet_rpm=.02,
            brake_active_required=True, rotor_disabled_required=True, finite_motor_cap_nm=2,
            carry_angle_limit_deg=CARRY_ANGLE_LIMIT_DEG, original_carry_angle_limit_deg=2,
            other_collision_tracking_slip_and_hold_limits_unchanged=True, original_strict_certificate_not_claimed=True))
        self.record('speed_profile_declared', dict(empty_phases=sorted(EMPTY_PHASES), scale=1.0,
            maximum_rate_rad_s=.36, original_arm_rates_and_dense_timing=False, minimum_empty_interval_sim_s=.020, button_travel_rate_rad_s=.44,
            loaded_paths_holds_physics_and_gates_unchanged=False, only_loaded_rate_change_pilot_rad_s=.03,
            only_gate_change_retrieval_transient_rpm=.1))
        self.record('terminal_policy_declared', dict(policy=TERMINAL_POLICY, observation_sim_s=10, neutral_arms_not_claimed=True))
        self.record('free_carry_profile_declared',dict(scale=1.15,maximum_rate_rad_s=.36,
                    original_floor_sim_s=.020,free_carry_floor_sim_s=.020,
                    scope='four_horizontal_free_carry_phases_only',all_contact_pilot_lower_setdown_grippers_buttons_unchanged=True,
                    all_3_10_30_second_certificates_unchanged=True,prior_observed_lag_s=.035,
                    estimated_lag_error_rad=.0126,estimate_not_physical_certificate=True,
                    original_tracking_slip_angle_gates_unchanged=True))
        self.record('empty_return_policy_declared', dict(
            policy='skip_first_verified_empty_return_after_retreat',
            original_last_empty_parking_policy_unchanged=True,
            original_rates_holds_and_physics_gates_unchanged=False,
            empty_return_rates_holds_and_physics_gates_unchanged=True,
            next_hand_fresh_actual_scene_IK_FCL_required=True))
        self.prepare_open()
        super().run()
        self.machine_cycle()
        self.retrieve()

    def hold(self, error):
        self.emergency_stop.publish(Bool(data=False))
        super().hold(error)
        deadline = time.monotonic() + 120
        quiet_since = None
        while time.monotonic() < deadline:
            try:
                self.spin(False)
                machine = self.values.get('machine', {})
                if (time.monotonic() - self.latest.get('machine', 0) < 2 and
                    not machine.get('rotor_enabled', True) and abs(machine.get('rotor_rpm', math.inf)) <= .02):
                    quiet_since = self.sim if quiet_since is None else quiet_since
                    if self.sim - quiet_since >= .5:
                        self.record('emergency_rotor_quiet', machine)
                        return
                else:
                    quiet_since = None
            except Exception as exception:
                self.record('emergency_hold_observer_error', dict(error=str(exception)))
                break
        self.record('emergency_rotor_quiet_unconfirmed', self.values.get('machine'))


def main():
    establish_interrupt_handler()
    verify_interrupt_delivery(os.getpid())
    read_contract(EVIDENCE)
    manager_rate = json.loads((EVIDENCE / 'run/manager_rate.json').read_text())
    verify_manager_rate_output(manager_rate['stdout'], manager_rate['command_returncode'])
    from gain_contract import verify_gain_output
    gain = json.loads((EVIDENCE / 'run/position_gain.json').read_text())
    verify_gain_output(gain['stdout'], gain['command_returncode'])
    from gripper_damping import verify_damping_output
    damping = json.loads((EVIDENCE / 'run/gripper_damping_readback.json').read_text())
    assert damping['read_only_no_parameter_set'] and len(damping['attempts']) == 4
    for attempt in damping['attempts']:
        verify_damping_output(attempt['stdout'], attempt['command_returncode'])
    from button_ramp import require_spline_output
    interpolation = json.loads((EVIDENCE / 'run/button_interpolation_readback.json').read_text())
    assert interpolation['read_only_no_parameter_set'] and {row['side'] for row in interpolation['attempts']} == {'left', 'right'}
    for attempt in interpolation['attempts']:
        require_spline_output(attempt['stdout'], attempt['command_returncode'])
    assert json.loads((EVIDENCE / 'calibrated_gain_audit.json').read_text())['status'] == 'PASS'
    for relative, expected in json.loads((EVIDENCE / 'identity.json').read_text())['sha256'].items():
        assert hashlib.sha256((EVIDENCE / relative).read_bytes()).hexdigest() == expected, relative
    assert json.loads((EVIDENCE / 'buttons_collision_audit.json').read_text())['status'] == 'PASS'
    dispatch = validate_dispatch(EVIDENCE)
    with (EVIDENCE / 'run/dispatch_gate.json').open('x') as stream:
        json.dump(dispatch, stream, indent=2)
    with (EVIDENCE / 'run/execution.claim').open('x') as stream:
        json.dump(dict(identity_sha256=hashlib.sha256((EVIDENCE / 'identity.json').read_bytes()).hexdigest()), stream)
    plan = json.loads((EVIDENCE / 'inputs/plan.json').read_text())
    buttons = json.loads((EVIDENCE / 'inputs/button_plan.json').read_text())
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    from endpoint_overlap_cycle import EndpointOverlapCycle as DualCycle
    cycle = DualCycle(plan, buttons)
    status, error = 'FAIL', None
    try:
        cycle.run_complete()
        status = 'PASS'
    except (Exception, KeyboardInterrupt) as exception:
        error = traceback.format_exc()
        cycle.hold(exception)
    finally:
        with (EVIDENCE / 'run/result.json').open('x') as stream:
            json.dump(dict(status=status, scope='two_box_load_close_spin_open_retrieve', error=error,
                        holding=cycle.holding, held_payload=cycle.held_payload, motions=cycle.motion_count,
                        max_slip_m=cycle.max_slip, max_native_slip_m=cycle.max_native_slip,
                        legacy_unaligned_fk_max_slip_m=getattr(cycle, 'maximum_legacy_unaligned_slip', None),
                        final_machine=cycle.values.get('machine'), physical_press_events=cycle.events,
                        actual_holder_registry=cycle.holders.snapshot(), paired_actual_cycle=True), stream, indent=2)
        cycle.close()
        rclpy.try_shutdown()
    print(status, error, flush=True)
    assert status == 'PASS'


if __name__ == '__main__':
    main()

