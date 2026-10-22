import hashlib
import itertools
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from rclpy.qos import qos_profile_sensor_data
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation
from batched_trace import BatchedLosslessTrace as LosslessTrace
from grip_limits import grip_exceeds_limits

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = Path(__file__).resolve().parent

GROUP_JOINTS = {'torso': ['torso_joint']}
for side in ('left', 'right'):
    GROUP_JOINTS[side + '_arm'] = [side + '_' + suffix + '_joint' for suffix in (
        'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    GROUP_JOINTS[side + '_gripper'] = [side + f'_dex1_finger_{number}_joint' for number in (1, 2)]

RATE_SCALE = 1.8
LOADED_RATE_SCALE = 2.6
RIGHT_ALIGN_RATE_RAD_S = .10


def velocity_violation(value):
    if not math.isfinite(value):
        return 'nonfinite'
    if abs(value) > .2 + 1e-8:
        return 'exceeds_model_0.2_m_s'
    return None


def trajectory_rate(side, phase):
    if side not in ('left', 'right'):
        raise ValueError('Unknown payload side')
    if side == 'right' and phase == 'align':
        return RIGHT_ALIGN_RATE_RAD_S
    rate = .20
    if phase == 'lift':
        rate = .12
    elif ((side == 'left' and phase in ('reorient', 'transfer')) or
          (side == 'right' and phase in ('transfer', 'lower'))):
        rate = .05
    elif phase in ('descend', 'setdown', 'reorient', 'transfer', 'lower', 'align'):
        rate = .10
    scale = LOADED_RATE_SCALE if phase in ('lift', 'reorient', 'transfer', 'lower') else RATE_SCALE
    return rate * scale


def grouped_names(group, names):
    allowed = {name for group_names in GROUP_JOINTS.values() for name in group_names}
    if not set(names) <= allowed:
        raise ValueError('Non-task joint command refused')
    return [name for name in GROUP_JOINTS[group] if name in names]


def pose_matrix(value):
    from cached_pose import cached_pose_matrix
    return cached_pose_matrix(value)


def synchronized_grip_error(poses, side, reference):
    expected = pose_matrix(poses[side + '_wrist_yaw_link']) @ reference
    actual = pose_matrix(poses['box48_' + side])
    displacement = float(np.linalg.norm(expected[:3, 3] - actual[:3, 3]))
    angle = float(Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude())
    return displacement, angle


def bucket_relative_corners(poses, side):
    from cached_bucket import cached_bucket_corners
    return cached_bucket_corners(poses, side)


def fully_inside_bucket(poses, side):
    corners = bucket_relative_corners(poses, side)
    return bool(np.max(np.abs(corners[:, 0])) <= .045 and
                np.max(np.abs(corners[:, 1])) <= .067 and
                np.min(corners[:, 2]) >= -.0305 and np.max(corners[:, 2]) <= .0305)


def midband_contacts(poses, pairs, side):
    inverse = np.linalg.inv(pose_matrix(poses['box48_' + side]))
    result = {}
    for number in (1, 2):
        points = [point for pair in pairs
                  if side + f'_dex1_finger_{number}_' in pair['first'] + pair['second']
                  for point in pair['positions']]
        if not points:
            raise RuntimeError('Missing midband finger contact points')
        local = np.asarray(points) @ inverse[:3, :3].T + inverse[:3, 3]
        mean = local.mean(axis=0)
        if not np.isfinite(local).all() or np.max(np.abs(local[:, 0])) > .018 or abs(mean[0]) > .004:
            raise RuntimeError('Contact is not on the marked middle band')
        if np.max(np.abs(np.abs(local[:, 1]) - .044070050215)) > .001 or np.max(np.abs(local[:, 2])) > .030:
            raise RuntimeError('Contact is not on the side wall')
        result[str(number)] = dict(centroid_payload_m=mean.tolist(), bounds_payload_m=[local.min(axis=0).tolist(), local.max(axis=0).tolist()])
    if result['1']['centroid_payload_m'][1] * result['2']['centroid_payload_m'][1] >= 0:
        raise RuntimeError('Finger contacts must oppose each other')
    return result


class LoadingCycle:
    def __init__(self, plan):
        self.plan = plan
        self.node = rclpy.create_node('centrifuge_48box_sequential_full_r349')
        self.trace = LosslessTrace(EVIDENCE / 'run')
        self.phase = 'preflight'
        self.sim = None
        self.positions = {}
        self.joint_time = None
        self.latest = {}
        self.values = {}
        self.contacts = {'left': [], 'right': []}
        self.poses = {}
        self.pose_time = None
        self.pose_wall = 0
        self.pose_snapshot = None
        self.native_grasp_transform = None
        self.max_native_slip = 0
        self.last_grip_record = -1
        self.last_record = -1
        self.last_gripper_record = {'left': -1, 'right': -1}
        self.holding = None
        self.grasp_transform = None
        self.max_slip = 0
        self.motion_count = 0
        self.active_gripper_goal = None
        self.accepted_gripper_targets = {}
        self.accepted_gripper_efforts = {}
        self.gripper_velocities = {}
        self.velocity_fault = None
        self.drop_window_until = None
        self.publishers = {group: self.node.create_publisher(JointTrajectory, '/' + group + '_controller/joint_trajectory', 10)
                           for group in ('left_arm', 'right_arm', 'left_gripper', 'right_gripper', 'torso')}
        self.gripper_actions = {side: ActionClient(self.node, FollowJointTrajectory, '/' + side + '_gripper_controller/follow_joint_trajectory')
                                for side in ('left', 'right')}
        self.node.create_subscription(Clock, '/clock', self.receive_clock, qos_profile_sensor_data)
        self.node.create_subscription(JointState, '/joint_states', self.receive_joints, qos_profile_sensor_data)
        for kind, topic in [('machine', '/centrifuge/machine/status'), ('localization', '/mid360_localization/status')]:
            self.node.create_subscription(String, topic, lambda message, kind=kind: self.receive(kind, json.loads(message.data)), 10)
        for side in ('left', 'right'):
            self.node.create_subscription(Contacts, '/lab_48box/' + side + '/contacts',
                                          lambda message, side=side: self.receive_contacts(side, message), qos_profile_sensor_data)
            self.node.create_subscription(JointTrajectoryControllerState, '/' + side + '_gripper_controller/controller_state',
                                          lambda message, side=side: self.receive_gripper_controller(side, message), qos_profile_sensor_data)
        self.model = RobotModel(str(ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf'),
                                str(ROOT / 'src/h1_2_moveit_config/config/h1_2.srdf'))
        self.state = RobotState(self.model)
        self.pose_process = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/h1_world/dynamic_pose/info', '--json-output'],
                                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.pose_thread = threading.Thread(target=self.read_poses, daemon=True)
        self.pose_thread.start()

    def record(self, kind, payload):
        self.trace.write(json.dumps(dict(kind=kind, phase=self.phase, sim=self.sim, wall=time.time(), payload=payload), allow_nan=False) + '\n')
        self.trace.flush()

    def read_poses(self):
        for line in self.pose_process.stdout:
            try:
                value = json.loads(line)
                poses = {pose['name']: pose for pose in value['pose'] if pose.get('name') in
                         ('box48_left', 'box48_right', 'bucket_left', 'bucket_right', 'rotor', 'lid', 'centrifuge',
                          'left_wrist_yaw_link', 'right_wrist_yaw_link',
                          'left_dex1_finger_1_slider_link', 'left_dex1_finger_2_slider_link',
                          'right_dex1_finger_1_slider_link', 'right_dex1_finger_2_slider_link')}
                if len(poses) != 13:
                    continue
                stamp = value['header']['stamp']
                pose_time = int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0)) * 1e-9
                self.pose_snapshot = (pose_time, time.monotonic(), poses)
            except (ValueError, KeyError):
                continue

    def receive_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.sim is not None and stamp < self.sim - .001:
            raise RuntimeError('Clock reversal')
        self.sim = stamp
        self.latest['clock'] = time.monotonic()

    def receive_joints(self, message):
        self.positions.update(zip(message.name, message.position))
        self.gripper_velocities.update((name, value) for name, value in zip(message.name, message.velocity) if 'dex1_finger' in name)
        self.joint_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.latest['joints'] = time.monotonic()
        values = {name: value for name, value in zip(message.name, message.velocity) if 'dex1_finger' in name}
        self.record('raw_gripper_velocity', dict(joint_sim=self.joint_time,
                    positions={name: self.positions[name] for name in values},
                    velocities={name: value if math.isfinite(value) else str(value) for name, value in values.items()}))
        for name, value in values.items():
            if getattr(self, 'gripper_velocity_violation', velocity_violation)(value) and self.velocity_fault is None:
                self.velocity_fault = dict(joint=name, value=value if math.isfinite(value) else str(value),
                                           joint_sim=self.joint_time, reason=getattr(self, 'gripper_velocity_violation', velocity_violation)(value))
                self.record('velocity_fault_sample', self.velocity_fault)
        if self.sim is not None and self.sim - self.last_record > .025:
            self.last_record = self.sim
            self.record('state', dict(joints=self.positions, joint_sim=self.joint_time, poses=self.poses, pose_sim=self.pose_time))

    def receive(self, kind, value):
        self.values[kind] = value
        self.latest[kind] = time.monotonic()
        self.record(kind, value)

    def receive_contacts(self, side, message):
        pairs = []
        for contact in message.contacts:
            forces = [math.sqrt(wrench.body_1_wrench.force.x ** 2 + wrench.body_1_wrench.force.y ** 2 + wrench.body_1_wrench.force.z ** 2)
                      for wrench in contact.wrenches]
            pairs.append(dict(first=contact.collision1.name, second=contact.collision2.name,
                              depth=max(contact.depths, default=0), force=max(forces, default=0),
                              positions=[[point.x, point.y, point.z] for point in contact.positions],
                              normals=[[normal.x, normal.y, normal.z] for normal in contact.normals],
                              force_vectors=[[wrench.body_1_wrench.force.x, wrench.body_1_wrench.force.y, wrench.body_1_wrench.force.z] for wrench in contact.wrenches]))
        self.contacts[side] = pairs
        self.latest[side + '_contacts'] = time.monotonic()
        self.record('contact', dict(side=side, stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9, pairs=pairs))

    def receive_gripper_controller(self, side, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp - self.last_gripper_record[side] < .1:
            return
        self.last_gripper_record[side] = stamp
        self.record('gripper_controller', dict(side=side, stamp=stamp, reference=list(message.reference.positions),
                                             feedback=list(message.feedback.positions), output=list(message.output.positions),
                                             reference_effort=list(message.reference.effort), output_effort=list(message.output.effort)))
        if any(not math.isfinite(value) or abs(value) > 18.01 for value in message.output.effort):
            raise RuntimeError('Effort experiment output exceeds 18 N')

    def wrist(self, side):
        self.state.joint_positions = {name: self.positions[name] for name in self.plan['initial_positions']}
        self.state.update()
        return self.state.get_global_link_transform(side + '_wrist_yaw_link')

    def localization_is_current(self, now):
        return now - self.latest.get('localization', 0) <= 8 and self.values['localization']['valid']

    def safety(self):
        now = time.monotonic()
        for kind in ('clock', 'joints', 'machine', 'left_contacts', 'right_contacts'):
            if now - self.latest.get(kind, 0) > 2:
                raise RuntimeError('Stale ' + kind)
        if not self.localization_is_current(now):
            raise RuntimeError('Localization invalid')
        if now - self.pose_wall > 2 or self.pose_time is None or abs(self.pose_time - self.sim) > getattr(self, 'pose_clock_skew_limit_sim_s', .15):
            self.record('pose_freshness_fault', dict(clock_sim=self.sim, pose_sim=self.pose_time,
                                                   joint_sim=self.joint_time, pose_wall_age_s=now - self.pose_wall,
                                                   clock_wall_age_s=now - self.latest.get('clock', 0),
                                                   difference_limit_sim_s=getattr(self, 'pose_clock_skew_limit_sim_s', .15), wall_age_limit_s=2))
            raise RuntimeError('Named Gazebo poses stale')
        from loading_machine_guard import machine_ready_for_loading
        machine = self.values['machine']
        if not machine_ready_for_loading(machine, self.phase, self.sim, getattr(self, 'drop_window_until', None)):
            self.record('machine_guard_fault', dict(machine=machine, drop_window_until=getattr(self, 'drop_window_until', None)))
            raise RuntimeError('Machine not open and stopped')
        if abs(self.positions.get('torso_joint', 10)) > math.radians(90.5):
            raise RuntimeError('Torso bound')
        if self.velocity_fault is not None:
            raise RuntimeError('Effort gripper velocity fault: ' + json.dumps(self.velocity_fault))
        for name, initial in self.plan['initial_positions'].items():
            if any(part in name for part in ('hip_', 'knee_', 'ankle_')) and abs(self.positions[name] - initial) > .025:
                raise RuntimeError('Passive leg deviation ' + name)
        from loading_contact_guard import contact_violation
        for side, pairs in self.contacts.items():
            for pair in pairs:
                violation = contact_violation(pair, side, self.phase, self.sim, self.drop_window_until)
                if violation:
                    self.record('contact_guard_fault', dict(side=side, pair=pair, violation=violation,
                                                           drop_window_until=self.drop_window_until))
                    raise RuntimeError('Payload contact guard: ' + violation)
        if self.holding:
            expected = self.wrist(self.holding) @ self.grasp_transform
            actual = pose_matrix(self.poses['box48_' + self.holding])
            slip = np.linalg.norm(expected[:3, 3] - actual[:3, 3])
            angle = Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude()
            self.max_slip = max(self.max_slip, float(slip))
            native_slip, native_angle = synchronized_grip_error(self.poses, self.holding, self.native_grasp_transform)
            self.max_native_slip = max(self.max_native_slip, native_slip)
            if self.pose_time - self.last_grip_record > .02:
                self.last_grip_record = self.pose_time
                self.record('synchronized_grip', dict(side=self.holding, pose_sim=self.pose_time, native_slip_m=native_slip,
                                                    native_angle_deg=math.degrees(native_angle), fk_slip_m=float(slip), fk_angle_deg=math.degrees(angle)))
            if grip_exceeds_limits(float(slip), math.degrees(angle)):
                raise RuntimeError(f'Grip slip {slip:.6f} m, {math.degrees(angle):.4f} deg')
            if grip_exceeds_limits(native_slip, math.degrees(native_angle)):
                raise RuntimeError(f'Native synchronized grip slip {native_slip:.6f} m, {math.degrees(native_angle):.4f} deg')

    def check_insertion_alignment(self, side):
        actual = pose_matrix(self.poses['box48_' + side])
        target = np.array([.603178339 if side == 'left' else .657963763,
                           .085730038 if side == 'left' else -.085730037])
        if np.linalg.norm(actual[:2, 3] - target) > .0005:
            raise RuntimeError('Insertion XY alignment exceeds 0.5 mm')
        desired = Rotation.from_euler('z', .309270119).as_matrix()
        if Rotation.from_matrix(desired.T @ actual[:3, :3]).magnitude() > math.radians(.25):
            raise RuntimeError('Insertion angle exceeds 0.25 degrees')
        bucket = pose_matrix(self.poses['bucket_' + side])
        expected_yaw = 1.880066445897898 if side == 'left' else -1.261526207691896
        expected_rotation = Rotation.from_euler('z', expected_yaw).as_matrix()
        if np.linalg.norm(bucket[:2, 3] + [.6, 0] - target) > .0003 or Rotation.from_matrix(expected_rotation.T @ bucket[:3, :3]).magnitude() > math.radians(.2):
            raise RuntimeError('Passive bucket moved away from audited pose')
        bucket[:3, 3] += [.6, 0, .815]
        corners = np.array(list(itertools.product((-1, 1), repeat=3))) * np.array([.1187450051085, .08814010043, .0595]) / 2
        for height in (actual[2, 3], 1.12025):
            centre = actual[:3, 3].copy()
            centre[2] = height
            local_corners = (corners @ actual[:3, :3].T + centre - bucket[:3, 3]) @ bucket[:3, :3]
            if np.max(np.abs(local_corners[:, 0])) > .0448 or np.max(np.abs(local_corners[:, 1])) > .0668:
                raise RuntimeError('Combined bucket/payload alignment leaves less than 0.2 mm side clearance')

    def correct_release_alignment(self, side):
        from alignment_wait import wait_for_planner

        self.dwell(.5)
        self.phase = side + '_release_alignment_planning'
        actual = pose_matrix(self.poses['box48_' + side])
        reference = np.linalg.inv(self.wrist(side)) @ actual
        request = dict(side=side, positions=dict(self.positions), poses=self.poses,
                       actual=actual.tolist(), reference=reference.tolist())
        request_path = EVIDENCE / 'run' / (side + '_alignment_request.json')
        result_path = EVIDENCE / 'run' / (side + '_alignment_result.json')
        assert not result_path.exists()
        with request_path.open('x') as stream:
            json.dump(request, stream, indent=2)
        self.record('alignment_planning_started', dict(side=side, request=str(request_path),
                    clock_sim=self.sim, pose_sim=self.pose_time, joint_sim=self.joint_time))
        with (EVIDENCE / 'run' / (side + '_alignment_worker.log')).open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'alignment_worker.py'),
                                        str(request_path), str(result_path)],
                                       stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=60)
        plan = json.loads(result_path.read_text())
        assert plan['status'] == 'PASS' and plan['planning_only_attachment']
        assert plan['maximum_joint_step_rad'] == .002
        current = pose_matrix(self.poses['box48_' + side])
        drift_m = float(np.linalg.norm(current[:3, 3] - actual[:3, 3]))
        drift_deg = math.degrees(Rotation.from_matrix(actual[:3, :3].T @ current[:3, :3]).magnitude())
        joint_drift = max(abs(self.positions[name] - request['positions'][name]) for name in plan['samples'][0])
        self.record('alignment_snapshot_checked', dict(side=side, drift_m=drift_m,
                    drift_deg=drift_deg, joint_drift_rad=joint_drift))
        if drift_m > .0003 or drift_deg > .1 or joint_drift > .0003:
            raise RuntimeError('Alignment planning snapshot drifted')
        self.record('alignment_correction_plan', plan)
        self.move(plan['samples'], side + '_release_alignment_correction', .05)
        self.dwell(.5)

    def verify_dropped_payload(self, side):
        self.phase = side + '_drop_settling'
        start = self.sim
        deadline = time.monotonic() + 160
        stable_since = None
        reference = None
        while self.sim - start < 8:
            self.spin()
            pairs = self.contacts[side]
            supported = any('bucket_' + side in pair['first'] + pair['second'] for pair in pairs)
            released = not any('dex1' in pair['first'] + pair['second'] for pair in pairs)
            corners = bucket_relative_corners(self.poses, side)
            qualified = supported and released and fully_inside_bucket(self.poses, side) and abs(self.values['machine']['rotor_rpm']) <= .02
            if qualified:
                if reference is None or np.max(np.abs(corners - reference)) > .0005:
                    reference = corners
                    stable_since = self.sim
                if self.sim - stable_since >= 3:
                    self.drop_window_until = None
                    self.record('drop_acceptance_pass', dict(side=side, duration_sim_s=self.sim - stable_since,
                                                            corners_bucket_m=corners.tolist(), rotor_rpm=self.values['machine']['rotor_rpm']))
                    return
            else:
                stable_since = None
                reference = None
            if time.monotonic() > deadline:
                raise TimeoutError('Drop settling wall-clock timeout')
        raise RuntimeError('Payload did not fall completely into the bucket and settle')

    def spin(self, guarded=True):
        if self.pose_snapshot is not None:
            self.pose_time, self.pose_wall, self.poses = self.pose_snapshot
        rclpy.spin_once(self.node, timeout_sec=.01)
        if guarded:
            self.safety()

    def dwell(self, seconds):
        start = self.sim
        deadline = time.monotonic() + max(30, seconds * 20)
        while self.sim - start < seconds:
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Dwell timeout')

    def minimum_waypoint_interval(self, phase):
        return .020

    def move(self, samples, phase, rate=.12):
        self.phase = phase
        self.safety()
        start = self.sim + .15
        names = list(samples[-1])
        first = {name: self.positions[name] for name in names}
        points = [first] + samples
        elapsed = .1
        times = []
        previous = first
        for point in points:
            elapsed += max(self.minimum_waypoint_interval(phase), max(abs(point[name] - previous[name]) / (.0008 if 'dex1_finger' in name else rate) for name in names))
            times.append(elapsed)
            previous = point
        for group, publisher in self.publishers.items():
            group_names = grouped_names(group, names)
            if not group_names:
                continue
            message = JointTrajectory()
            message.header.stamp.sec = int(start)
            message.header.stamp.nanosec = int((start % 1) * 1e9)
            message.joint_names = group_names
            for point, seconds in zip(points, times):
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = [float(point[name]) for name in group_names]
                trajectory_point.time_from_start.sec = int(seconds)
                trajectory_point.time_from_start.nanosec = int((seconds % 1) * 1e9)
                message.points.append(trajectory_point)
            publisher.publish(message)
            self.motion_count += 1
        self.record('command', dict(names=names, samples=samples, start=start, times=times, initial=first))
        deadline = time.monotonic() + max(60, elapsed * 25)
        while self.sim < start + elapsed + .15:
            self.spin()
            relative = self.joint_time - start
            if relative >= times[0]:
                for name in names:
                    if 'dex1_finger' not in name:
                        reference = np.interp(relative, times, [point[name] for point in points])
                        if abs(self.positions[name] - reference) > .015:
                            self.record('tracking_fault_sample', dict(joint=name, actual=self.positions[name],
                                                                      reference=float(reference),
                                                                      error_rad=float(self.positions[name] - reference),
                                                                      joint_sim=self.joint_time, trajectory_start=start,
                                                                      relative_time=relative, limit_rad=.015))
                            raise RuntimeError('In-motion tracking deviation ' + name)
            if time.monotonic() > deadline:
                raise TimeoutError('Motion timeout ' + phase)
        settled_since = None
        settle_start = self.sim
        while self.sim - settle_start < 5:
            self.spin()
            if all(abs(self.positions[name] - samples[-1][name]) < (.00025 if 'dex1_finger' in name else .0003) for name in names):
                settled_since = self.sim if settled_since is None else settled_since
                if self.sim - settled_since > .2:
                    break
            else:
                settled_since = None
        else:
            raise RuntimeError('Joint endpoint did not settle to original precision gate')
        print(phase, 'reached', self.sim, flush=True)

    def gripper_goal_time_tolerance(self, side, phase):
        return 3

    def gripper_action_timeout_s(self, side, phase):
        return 90

    def gripper_trajectory_points(self, side, value, phase, effort):
        names = GROUP_JOINTS[side + '_gripper']
        initial = JointTrajectoryPoint(positions=[self.positions[name] for name in names],
                                       velocities=[0.0, 0.0],
                                       effort=[self.accepted_gripper_efforts.get(side, 0.0)] * 2)
        initial.time_from_start.nanosec = 50000000
        point = JointTrajectoryPoint(positions=[value, value], velocities=[0.0, 0.0], effort=[effort, effort])
        point.time_from_start.sec = 1
        return [initial, point]

    def gripper_observation_phase(self, phase):
        return phase

    def gripper(self, side, value, phase):
        self.phase = self.gripper_observation_phase(phase)
        self.safety()
        client = self.gripper_actions[side]
        assert client.server_is_ready(), 'Gripper action server unavailable'
        names = GROUP_JOINTS[side + '_gripper']
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        effort = 16.0 if phase == side + '_grasp' else 0.0
        goal.trajectory.points = self.gripper_trajectory_points(side, value, phase, effort)
        goal.goal_tolerance = [JointTolerance(name=name, position=.0002) for name in names]
        goal.goal_time_tolerance.sec = self.gripper_goal_time_tolerance(side, phase)
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + self.gripper_action_timeout_s(side, phase)
        while not future.done():
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Gripper goal acknowledgment timeout')
        handle = future.result()
        assert handle.accepted, 'Gripper goal rejected'
        self.active_gripper_goal = handle
        self.accepted_gripper_targets[side] = value
        self.accepted_gripper_efforts[side] = effort
        self.motion_count += 1
        self.record('gripper_goal_accepted', dict(side=side, target=value, feedforward_N=effort))
        completed = handle.get_result_async()
        while not completed.done():
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Gripper action result timeout')
        response = completed.result()
        self.active_gripper_goal = None
        self.record('gripper_result', dict(side=side, target=value, status=response.status,
                                          error_code=response.result.error_code, error=response.result.error_string))
        assert response.result.error_code == 0, response.result.error_string
        self.dwell(.3)
        assert max(abs(self.positions[name] - value) for name in names) < .00025, 'Gripper measured endpoint error'
        print(phase, 'acknowledged and measured', self.sim, flush=True)

    def preflight(self):
        deadline = time.monotonic() + 30
        ready_since = None
        while time.monotonic() < deadline:
            self.spin(False)
            if len(self.poses) == 13 and len(self.positions) >= len(self.plan['initial_positions']) and all(
                    kind in self.latest for kind in ('machine', 'localization', 'left_contacts', 'right_contacts')) and all(
                    publisher.get_subscription_count() == 1 and self.node.count_publishers('/' + group + '_controller/joint_trajectory') == 1
                    for group, publisher in self.publishers.items()):
                ready_since = ready_since or time.monotonic()
                if time.monotonic() - ready_since >= 2:
                    break
            else:
                ready_since = None
        self.safety()
        for group, publisher in self.publishers.items():
            counts = dict(publishers=self.node.count_publishers('/' + group + '_controller/joint_trajectory'), subscriptions=publisher.get_subscription_count())
            self.record('discovery', dict(group=group, **counts))
            assert counts['publishers'] == 1 and counts['subscriptions'] == 1, (group, counts)
        for name, value in self.plan['initial_positions'].items():
            assert abs(self.positions[name] - value) < .015, (name, self.positions[name], value)
        for side in ('left', 'right'):
            pose = pose_matrix(self.poses['box48_' + side])
            expected = [.44, .38 if side == 'left' else -.38, .84475]
            assert np.linalg.norm(pose[:3, 3] - expected) < .0003
            assert Rotation.from_matrix(pose[:3, :3]).magnitude() < .001
        self.dwell(.5)
        self.record('preflight_pass', self.poses)

    def run(self):
        self.preflight()
        for side in ('left', 'right'):
            self.gripper(side, -.0199, side + '_open_empty')
            self.dwell(1)
            self.gripper(side, -.01901, side + '_close_empty_check')
            self.gripper(side, -.0199, side + '_reopen_empty')
        for stage in self.plan['stages']:
            side, phase, samples = stage['side'], stage['phase'], stage['samples']
            samples = [{name: value for name, value in sample.items() if 'dex1_finger' not in name} for sample in samples]
            if phase == 'lift':
                self.gripper(side, -.01901, side + '_grasp')
                self.dwell(.25)
                pairs = self.contacts[side]
                for number in (1, 2):
                    assert any(side + f'_dex1_finger_{number}_' in pair['first'] + pair['second'] for pair in pairs), ('Missing finger contact', pairs)
                loads = {}
                for number in (1, 2):
                    loads[number] = sum(abs(float(np.dot(force, normal))) for pair in pairs
                                        if side + f'_dex1_finger_{number}_' in pair['first'] + pair['second']
                                        for force, normal in zip(pair['force_vectors'], pair['normals']))
                self.record('grip_preload', dict(side=side, normal_loads_N=loads))
                assert all(3 <= force <= 20.5 for force in loads.values()), ('Unqualified grip preload', loads)
                self.record('midband_contact_pass', dict(side=side, fingers=midband_contacts(self.poses, pairs, side)))
                gripped_pose = pose_matrix(self.poses['box48_' + side])
                expected = [.44, .38 if side == 'left' else -.38, .84475]
                assert np.linalg.norm(gripped_pose[:3, 3] - expected) < .0003, 'Payload displaced before lifting'
                assert Rotation.from_matrix(gripped_pose[:3, :3]).magnitude() < math.radians(.2)
                self.holding = side
                self.grasp_transform = np.linalg.inv(self.wrist(side)) @ pose_matrix(self.poses['box48_' + side])
                self.native_grasp_transform = np.linalg.inv(pose_matrix(self.poses[side + '_wrist_yaw_link'])) @ pose_matrix(self.poses['box48_' + side])
                self.move(samples[:7], side + '_pilot_lift', trajectory_rate(side, 'lift'))
                self.dwell(.5)
                assert pose_matrix(self.poses['box48_' + side])[2, 3] > .85475
                assert not any('table::' in pair['first'] + pair['second'] for pair in self.contacts[side])
                self.phase = side + '_static_grip_hold'
                self.dwell(10)
                self.record('static_grip_hold_pass', dict(side=side, duration_sim_s=10, native_slip_m=self.max_native_slip))
                self.record('pilot_grip_pass', dict(side=side, slip=self.max_slip))
                samples = samples[6:]
            rate = trajectory_rate(side, phase)
            self.move(samples, side + '_' + phase, rate)
            if phase == 'align':
                self.dwell(.5)
                try:
                    self.check_insertion_alignment(side)
                except RuntimeError as alignment_error:
                    self.record('alignment_correction_required', dict(side=side, error=str(alignment_error)))
                    self.correct_release_alignment(side)
                    self.check_insertion_alignment(side)
                self.record('overhead_release_gate_pass', dict(side=side, poses=self.poses))
                self.holding = None
                self.drop_window_until = self.sim + 2
                self.gripper(side, -.0199, side + '_release_above_bucket')
                self.verify_dropped_payload(side)
            if phase == 'setdown':
                self.dwell(.5)
                expected = np.array([.44, .38 if side == 'left' else -.38, .84475])
                assert np.linalg.norm(pose_matrix(self.poses['box48_' + side])[:3, 3] - expected) < .002
                assert any('table::' in pair['first'] + pair['second'] for pair in self.contacts[side]), 'No table support before release'
                self.holding = None
                self.gripper(side, -.0199, side + '_release_on_table')
                self.dwell(3)
                assert not any('dex1' in pair['first'] + pair['second'] for pair in self.contacts[side]), 'Finger still supports returned payload'
                assert any('table::' in pair['first'] + pair['second'] for pair in self.contacts[side]), 'Lost table support after release'
                assert np.linalg.norm(pose_matrix(self.poses['box48_' + side])[:3, 3] - expected) < .002
                self.record('table_return_pass', dict(side=side, pose=self.poses['box48_' + side]))
        self.dwell(3)
        for side in ('left', 'right'):
            assert fully_inside_bucket(self.poses, side), 'Final complete containment failed'
            assert any('bucket_' + side in pair['first'] + pair['second'] for pair in self.contacts[side])
            assert not any('dex1' in pair['first'] + pair['second'] for pair in self.contacts[side])
        self.record('PASS', dict(scope='two_box_overhead_loading', loading_complete=True,
                                 motions=self.motion_count, max_slip_m=self.max_slip, poses=self.poses))

    def hold(self, error):
        self.phase = 'STOP_AND_HOLD'
        self.record('FAIL', dict(error=str(error), holding=self.holding))
        if self.active_gripper_goal is not None:
            self.active_gripper_goal.cancel_goal_async()
        for group, publisher in self.publishers.items():
            names = [name for name in GROUP_JOINTS[group] if name in self.positions]
            if not names:
                continue
            message = JointTrajectory(joint_names=names)
            values = [self.positions[name] for name in names]
            if group.endswith('_gripper') and group.split('_')[0] in self.accepted_gripper_targets:
                values = [self.accepted_gripper_targets[group.split('_')[0]]] * len(names)
            point = JointTrajectoryPoint(positions=values)
            if group.endswith('_gripper'):
                point.effort = [self.accepted_gripper_efforts.get(group.split('_')[0], 0.0)] * len(names)
                point.velocities = [0.0] * len(names)
            point.time_from_start.nanosec = 100000000
            message.points = [point]
            publisher.publish(message)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.spin(False)

    def close(self):
        self.pose_process.send_signal(signal.SIGINT)
        self.pose_process.wait(timeout=10)
        self.trace.close()
        self.node.destroy_node()


def main():
    raw = (EVIDENCE / 'inputs/plan.json').read_bytes()
    plan = json.loads(raw)
    assert plan['scope'] == 'two_box_overhead_loading' and plan['loading_complete'] is False
    assert all(stage['phase'] in ('park', 'approach', 'descend', 'lift', 'reorient', 'transfer', 'lower', 'align', 'retreat', 'return') for stage in plan['stages'])
    assert plan['status'] == 'OFFLINE_PASS' and all(not stage['collisions'] for stage in plan['stages'])
    audit = json.loads((EVIDENCE / 'dense_audit_retry1.json').read_text())
    assert audit['status'] == 'PASS' and audit['plan_sha256'] == hashlib.sha256(raw).hexdigest()
    with (EVIDENCE / 'run/execution.claim').open('x') as stream:
        json.dump(dict(plan_sha256=hashlib.sha256(raw).hexdigest(), script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), stream)
    rclpy.init()
    cycle = LoadingCycle(plan)
    status, error = 'FAIL', None
    try:
        cycle.run()
        status = 'PASS'
    except (Exception, KeyboardInterrupt) as exception:
        error = traceback.format_exc()
        cycle.hold(exception)
    finally:
        with (EVIDENCE / 'run/result.json').open('x') as stream:
            json.dump(dict(status=status, scope='two_box_overhead_loading', loading_complete=status == 'PASS', error=error, motions=cycle.motion_count, max_slip_m=cycle.max_slip, max_native_slip_m=cycle.max_native_slip, holding=cycle.holding), stream, indent=2)
        cycle.close()
        rclpy.shutdown()
    print(status, error, flush=True)
    assert status == 'PASS'


if __name__ == '__main__':
    main()
