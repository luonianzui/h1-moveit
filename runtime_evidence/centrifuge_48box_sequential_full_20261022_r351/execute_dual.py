import copy
import itertools
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alignment_wait import wait_for_planner
from coherent_time import newest_pair
from contact_phase_policy import bounded_motion, table_birth_violation
from dual_motion import HoldRegistry, TASK_JOINTS, common_arm_timing, validate_common_start
from execute_coherent import CoherentCycle
from execute_cycle import FullCycle
from execute_loading import GROUP_JOINTS, pose_matrix, midband_contacts, fully_inside_bucket, bucket_relative_corners
from extraction_clearance import clearance_metrics
from table_bucket_clearance import bucket_collision_bounds, table_bucket_clearance, TABLE_SETDOWN_PHASES
from grip_limits import grip_exceeds_limits
from motion_clock_watch import SimulationProgressWatch
from paired_setdown_braking import split_paired_setdown
from stop_phase import schedule_nominal_phase
from table_return_goal import table_orientation_error


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
CONTEXT_FIELDS = ('coherent_reference_key', 'coherent_reference', 'reference_native_bytes',
                  'last_coherent_record', 'grasp_transform', 'native_grasp_transform')


class DualCycle(CoherentCycle):
    def require_gripper_endpoint(self, hand, value, result_clock):
        assert max(abs(self.positions[name] - value) for name in GROUP_JOINTS[hand + '_gripper']) < .00025

    def __init__(self, plan, buttons):
        self.holders = HoldRegistry()
        self.holder_contexts = {}
        self.pair_goal_handles = {}
        self.pair_goal_futures = {}
        self.pair_clearance_payloads = set()
        self.worker_sequence = 0
        self.load_table_guard_active = False
        self.qualified_empty_retreat_guides = None
        super().__init__(plan, buttons)
        self.table_bucket_bounds = bucket_collision_bounds(ROOT / 'src/h1_2_description/models/centrifuge/model.sdf')
        self.model = RobotModel(str(EVIDENCE / 'inputs/robot_dual.urdf'), str(ROOT / 'src/h1_2_moveit_config/config/h1_2.srdf'))
        self.state = RobotState(self.model)

    def synchronize_primary(self):
        if not self.holders.rows:
            self.holding, self.held_payload = None, None
            return
        hand = next(iter(self.holders.rows))
        payload = self.holders.rows[hand]['payload']
        self.holding = hand
        self.held_payload = payload if self.holder_contexts[hand]['retrieval'] else None
        for name in CONTEXT_FIELDS:
            setattr(self, name, self.holder_contexts[hand][name])

    def latest_native_guard(self):
        for hand, held in self.holders.rows.items():
            expected = pose_matrix(self.poses[hand + '_wrist_yaw_link']) @ held['native_reference']
            actual = pose_matrix(self.poses['box48_' + held['payload']])
            slip = float(np.linalg.norm(expected[:3, 3] - actual[:3, 3]))
            angle = math.degrees(Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude())
            if grip_exceeds_limits(slip, angle):
                self.record('paired_immediate_native_fault', dict(hand=hand, payload=held['payload'],
                            slip_m=slip, angle_deg=angle, source_sim=self.pose_time))
                raise RuntimeError('Paired immediate native grip exceeds original 2mm/5deg')

    def refresh_native(self):
        with self.native_lock:
            if self.pose_snapshot is not None:
                self.pose_time, self.pose_wall, self.poses = self.pose_snapshot

    def spin(self, guarded=True):
        self.refresh_native()
        rclpy.spin_once(self.node, timeout_sec=.01)
        self.refresh_native()
        if guarded and self.holders.rows:
            deadline = time.monotonic() + .010
            for attempt in range(64):
                self.latest_native_guard()
                with self.native_lock:
                    pair = newest_pair(list(self.native_frames), self.joint_timeline, self.pose_time)
                if pair is not None or time.monotonic() >= deadline:
                    break
                rclpy.spin_once(self.node, timeout_sec=0)
                self.refresh_native()
        if guarded:
            self.safety()

    def check_coherent_grip(self, hand, payload, retrieval):
        try:
            return super().check_coherent_grip(hand, payload, retrieval)
        except RuntimeError as error:
            if 'No recent actual joint/native bracket' in str(error):
                with self.native_lock:
                    frames = list(self.native_frames)
                    pair = newest_pair(frames, self.joint_timeline, self.pose_time)
                self.record('paired_feedback_pair_fault', dict(hand=hand, payload=payload,
                    exact_requested_native_stamp=self.pose_time, joint_stamp=self.joint_time,
                    clock_sim=self.sim, history_length=len(self.joint_timeline.rows),
                    history_first=self.joint_timeline.rows[0][0] if self.joint_timeline.rows else None,
                    history_last=self.joint_timeline.rows[-1][0] if self.joint_timeline.rows else None,
                    last_native_stamps=[frame[0] for frame in frames[-8:]],
                    no_actual_bracket=pair is None,
                    actual_pair_wall_age_s=None if pair is None else time.monotonic() - pair['pose_wall'],
                    original_maximum_bracket_and_lag_sim_s=.012, original_pair_age_wall_s=2,
                    no_extrapolation_command_substitution_or_reference_rebase=True))
            raise

    def measured_pair(self):
        deadline = time.monotonic() + .010
        for attempt in range(65):
            if self.holders.rows:
                self.spin()
            else:
                rclpy.spin_once(self.node, timeout_sec=0)
                self.refresh_native()
                self.safety()
            with self.native_lock:
                pair = newest_pair(list(self.native_frames), self.joint_timeline, self.pose_time)
            if pair is not None and time.monotonic() - pair['pose_wall'] <= 2:
                return pair
            if attempt == 64 or time.monotonic() >= deadline:
                break
            rclpy.spin_once(self.node, timeout_sec=0)
        self.record('paired_snapshot_pair_fault', dict(requested_native_stamp=self.pose_time,
                    last_joint_stamp=self.joint_time, no_extrapolation=True,
                    no_actual_bracket=pair is None,
                    actual_pair_wall_age_s=None if pair is None else time.monotonic() - pair['pose_wall'],
                    original_bracket_and_lag_limit_sim_s=.012, bounded_actual_callbacks=64))
        raise RuntimeError('No actual bracket for paired snapshot after bounded real callbacks')

    def wait_until_sim(self, target_sim):
        if not math.isfinite(target_sim) or target_sim < self.sim:
            raise ValueError('Finite future actual simulation target required')
        if target_sim == self.sim:
            self.safety()
            return
        duration = target_sim - self.sim
        watcher = SimulationProgressWatch(duration, self.sim, time.monotonic())
        self.record('paired_sim_wait_wall_budget', dict(target_sim=target_sim, required_duration_sim_s=duration,
                    budget_wall_s=watcher.budget_wall_s, actual_simulation_duration_not_shortened=True,
                    unchanged_physical_freshness_and_holding_guards=True))
        while self.sim < target_sim:
            self.spin()
            try:
                watcher.observe(self.sim, time.monotonic())
            except (TimeoutError, ValueError) as error:
                self.record('paired_sim_wait_clock_fault', dict(error=str(error), actual_sim=self.sim,
                            required_target_sim=target_sim, budget_wall_s=watcher.budget_wall_s))
                raise

    def dwell(self, seconds):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('Positive finite actual simulation hold duration required')
        self.wait_until_sim(self.sim + seconds)

    def safety(self):
        if self.native_stream_fault:
            raise RuntimeError(self.native_stream_fault)
        primary = (self.holding, self.held_payload)
        self.holding, self.held_payload = None, None
        try:
            FullCycle.safety(self)
        finally:
            self.holding, self.held_payload = primary
        self.latest_native_guard()
        if self.load_table_guard_active:
            for payload in ('left', 'right'):
                body = pose_matrix(self.poses['box48_' + payload])
                angle = Rotation.from_matrix(body[:3, :3]).magnitude()
                violation = table_birth_violation(payload, body[:3, 3], angle)
                if violation:
                    self.record('paired_empty_table_pose_fault', dict(payload=payload, reason=violation,
                                position=body[:3, 3].tolist(), angle_rad=float(angle), source_sim=self.pose_time))
                    raise RuntimeError(violation)
        for hand, held in self.holders.rows.items():
            context = self.holder_contexts[hand]
            for name in CONTEXT_FIELDS:
                setattr(self, name, context[name])
            self.check_coherent_grip(hand, held['payload'], context['retrieval'])
            for name in CONTEXT_FIELDS:
                context[name] = getattr(self, name)
        self.synchronize_primary()
        for payload in self.pair_clearance_payloads:
            if self.phase in TABLE_SETDOWN_PHASES:
                poses = copy.deepcopy(self.poses)
                pairs = copy.deepcopy(self.contacts[payload])
                clearance = table_bucket_clearance(poses, payload, pairs, self.table_bucket_bounds)
                data = dict(payload=payload, clearance=clearance, poses=poses,
                            contacts=pairs, native_pose_sim=self.pose_time)
                if not clearance['valid']:
                    self.record('table_setdown_bucket_clearance_fault', data)
                    raise RuntimeError('Paired table setdown lost three-mm clearance from both baskets')
                if self.pose_time - self.last_clearance_record >= .025:
                    self.record('table_setdown_bucket_clearance', data)
                continue
            clearance = clearance_metrics(self.poses, payload, self.contacts[payload], self.bucket_rim_z[payload])
            if not clearance['valid']:
                self.record('extraction_clearance_fault', dict(payload=payload, clearance=clearance,
                            poses=copy.deepcopy(self.poses), contacts=self.contacts[payload]))
                raise RuntimeError('Paired full extraction clearance lost')
            if self.pose_time - self.last_clearance_record >= .025:
                self.record('extraction_clearance', dict(payload=payload, clearance=clearance))
        if self.pair_clearance_payloads and self.pose_time - self.last_clearance_record >= .025:
            self.last_clearance_record = self.pose_time

    def prepare_loading_grippers(self):
        for target, phase in ((-.0199, 'paired_load_open_empty'), (-.01901, 'paired_load_close_empty_check'),
                              (-.0199, 'paired_load_reopen_empty')):
            self.paired_grippers(target, 0, phase)

    def paired_gripper_post_action_dwell(self):
        return .3

    def paired_grippers(self, value, effort, phase, retrieval=False):
        self.phase = phase
        self.safety()
        start = self.sim + self.future_start_offset()
        seconds, nanoseconds = validate_common_start(self.sim, start)
        self.retrieval_closing = retrieval
        pending, complete = {}, {}
        for hand in ('left', 'right'):
            assert self.gripper_actions[hand].server_is_ready()
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = GROUP_JOINTS[hand + '_gripper']
            goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = seconds, nanoseconds
            legacy_phase = hand + '_grasp' if effort == 16 else hand + '_paired_empty'
            goal.trajectory.points = self.gripper_trajectory_points(hand, value, legacy_phase, effort)
            goal.goal_tolerance = [JointTolerance(name=name, position=.0002) for name in goal.trajectory.joint_names]
            goal.goal_time_tolerance.sec = 6 if effort else 3
            pending[hand] = self.gripper_actions[hand].send_goal_async(goal)
        self.pair_goal_futures = pending
        deadline = time.monotonic() + (240 if retrieval else 90)
        while not all(future.done() for future in pending.values()):
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Paired gripper acceptance timeout')
        for hand, future in pending.items():
            handle = future.result()
            assert handle.accepted, hand + ' paired gripper goal rejected'
            self.pair_goal_handles[hand] = handle
            self.accepted_gripper_targets[hand] = value
            self.accepted_gripper_efforts[hand] = float(effort)
            self.motion_count += 1
            self.record('gripper_goal_accepted', dict(side=hand, target=value, feedforward_N=effort,
                        common_start_sim=start, actual_pair_action=True))
            complete[hand] = handle.get_result_async()
        while not all(future.done() for future in complete.values()):
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Paired gripper result timeout')
        result_clock = self.sim
        for hand, future in complete.items():
            response = future.result()
            self.record('gripper_result', dict(side=hand, target=value, status=response.status,
                        error_code=response.result.error_code, error=response.result.error_string))
            assert response.result.error_code == 0, response.result.error_string
            self.require_gripper_endpoint(hand, value, result_clock)
        self.pair_goal_handles.clear()
        self.pair_goal_futures.clear()
        self.retrieval_closing = False
        self.dwell(self.paired_gripper_post_action_dwell())

    def capture_pair(self, assignment, retrieval=False):
        self.dwell(.25)
        pair = self.measured_pair()
        self.state.joint_positions = pair['joints']['positions']
        self.state.update()
        for hand, payload in assignment.items():
            pairs = self.contacts[payload]
            loads = {str(number): sum(abs(float(np.dot(force, normal))) for contact in pairs
                if hand + f'_dex1_finger_{number}_' in contact['first'] + contact['second']
                for force, normal in zip(contact['force_vectors'], contact['normals'])) for number in (1, 2)}
            alias = dict(self.poses)
            alias['box48_' + hand] = self.poses['box48_' + payload]
            band = midband_contacts(alias, pairs, hand)
            body = pose_matrix(pair['poses']['box48_' + payload])
            if not retrieval:
                expected = np.array([.44, .38 if payload == 'left' else -.38, .84475])
                assert np.linalg.norm(body[:3, 3] - expected) < .0003
                assert Rotation.from_matrix(body[:3, :3]).magnitude() < math.radians(.2)
            native = np.linalg.inv(pose_matrix(pair['poses'][hand + '_wrist_yaw_link'])) @ body
            fk = np.linalg.inv(self.state.get_global_link_transform(hand + '_wrist_yaw_link')) @ body
            proof = dict(source='gazebo_physics_contact', midband_verified=True, normal_loads_N=loads)
            self.holders.begin(hand, payload, native, fk, pair['pose_stamp'], proof)
            self.holder_contexts[hand] = dict(retrieval=retrieval, coherent_reference_key=None,
                coherent_reference=None, reference_native_bytes=None, last_coherent_record=-1.,
                grasp_transform=self.holders.rows[hand]['fk_reference'],
                native_grasp_transform=self.holders.rows[hand]['native_reference'])
            self.record('grip_preload', dict(side=hand, payload=payload, normal_loads_N=loads))
            self.record('midband_contact_pass', dict(side=hand, payload=payload, fingers=band))
            self.record('dual_holder_begin', dict(hand=hand, payload=payload, certificate=proof,
                        actual_joint_pair=pair['joints'], paired_poses=pair['poses'], frozen=self.holders.snapshot()[hand]))
        self.synchronize_primary()
        self.safety()

    def maximum_loaded_rate(self, phase):
        return .25

    def fine_setdown_rate(self):
        return .03

    def paired_feedback_overlap_enabled(self):
        return False

    def paired_endpoint_stamp(self, target, not_before):
        return self.sim if all(abs(self.positions[name] - value) < .0003 for name, value in target.items()) else None

    def common_timing(self, actual, samples, rate):
        return common_arm_timing(actual, samples, rate)

    def motion_limits(self, phase, requested_rate):
        return bounded_motion(phase, requested_rate)

    def coarse_setdown_rate(self):
        return .20

    def move(self, samples, phase, rate):
        self.phase = phase
        self.safety()
        rate, tracking_limit = self.motion_limits(phase, rate)
        if self.holders.rows and rate > self.maximum_loaded_rate(phase):
            raise ValueError('Paired loaded rate cannot exceed its declared phase cap')
        timing = self.common_timing(self.positions, samples, rate)
        start = self.sim + self.future_start_offset()
        seconds, nanoseconds = validate_common_start(self.sim, start)
        for group, names in timing['groups'].items():
            message = JointTrajectory(joint_names=names)
            message.header.stamp.sec, message.header.stamp.nanosec = seconds, nanoseconds
            for values, stamp in zip(timing['points'], timing['times']):
                point = JointTrajectoryPoint(positions=[float(values[name]) for name in names])
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(stamp * 1e9), 1000000000)
                message.points.append(point)
            self.publishers[group].publish(message)
            self.motion_count += 1
        self.record('command', dict(names=list(samples[0]), samples=samples, start=start, times=timing['times'],
                    initial=timing['points'][0], actual_rate_rad_s=rate, groups=timing['groups'],
                    simultaneous_physical_controllers=True, holders=self.holders.snapshot()))
        wall_watch = SimulationProgressWatch(timing['duration_sim_s'], self.sim, time.monotonic())
        self.record('paired_motion_wall_budget', dict(duration_sim_s=timing['duration_sim_s'],
                    budget_wall_s=wall_watch.budget_wall_s, seconds_per_sim_second=50,
                    future_start_and_completion_margin_sim_s=self.future_start_offset() + .15, minimum_wall_s=60,
                    maximum_wall_s=900, simulation_clock_must_advance_within_wall_s=2,
                    command_paths_rates_and_sim_timestamps_unchanged=self.future_start_offset() == .3))
        stable = None
        while self.sim < start + timing['duration_sim_s'] + .15:
            self.spin()
            try:
                wall_watch.observe(self.sim, time.monotonic())
            except (TimeoutError, ValueError) as error:
                self.record('paired_motion_clock_fault', dict(error=str(error), clock_sim=self.sim,
                            required_finish_sim=start + timing['duration_sim_s'] + .15,
                            budget_wall_s=wall_watch.budget_wall_s))
                raise
            relative = self.joint_time - start
            if relative >= timing['times'][0]:
                for name in samples[0]:
                    reference = float(np.interp(relative, timing['times'], [point[name] for point in timing['points']]))
                    if abs(self.positions[name] - reference) > tracking_limit:
                        self.record('tracking_fault_sample', dict(joint=name, actual=self.positions[name], reference=reference,
                                    error_rad=self.positions[name] - reference, limit_rad=tracking_limit, joint_sim=self.joint_time))
                        if tracking_limit == .002:
                            raise RuntimeError('Paired near-contact tracking exceeds stricter .002rad')
                        raise RuntimeError('Paired actual tracking exceeds original .015rad')
            if self.paired_feedback_overlap_enabled():
                stamp = self.paired_endpoint_stamp(samples[-1], start + timing['duration_sim_s'])
                if stamp is None or (stable is not None and stamp < stable):
                    stable = None
                elif stable is None:
                    stable = stamp
        begin = self.sim
        while self.sim - begin < 5:
            self.spin()
            stamp = self.paired_endpoint_stamp(samples[-1], start + timing['duration_sim_s'])
            if stamp is not None:
                stable = stamp if stable is None else stable
                if stamp - stable > .2:
                    return
            else:
                stable = None
        raise RuntimeError('Paired actual endpoint failed original precision gate')

    def actual_plan(self, mode, assignment, retrieval=False):
        self.safety()
        self.worker_sequence += 1
        pair = self.measured_pair()
        positions, poses = dict(pair['joints']['positions']), copy.deepcopy(pair['poses'])
        self.state.joint_positions = positions
        self.state.update()
        references = {hand: (np.linalg.inv(self.state.get_global_link_transform(hand + '_wrist_yaw_link')) @
                      pose_matrix(poses['box48_' + payload])).tolist() for hand, payload in assignment.items()}
        request = dict(mode=mode, assignment=assignment, positions=positions, poses=poses, retrieval=retrieval,
                       source_sim=pair['pose_stamp'], planning_references=references,
                       original_runtime_holder_references=self.holders.snapshot(), runtime_reference_not_rebased=True)
        if mode in ('empty_gripper_after_retreat', 'empty_return_after_gripper'):
            request['completed_empty_retreat_stages'] = list(self.completed_empty_retreat_stages)
            request['accepted_gripper_efforts'] = dict(self.accepted_gripper_efforts)
        if mode in ('empty_return', 'empty_gripper_after_retreat', 'empty_return_after_gripper'):
            if self.holders.rows or self.qualified_empty_retreat_guides is None:
                raise ValueError('Actual loaded-and-released guides required before empty retreat')
            request['current_run_executed_loading_guides'] = copy.deepcopy(self.qualified_empty_retreat_guides)
            request['current_run_guide_identity'] = EVIDENCE.name
        prefix = str(self.worker_sequence) + '_' + mode
        source, output = EVIDENCE / 'run' / (prefix + '_request.json'), EVIDENCE / 'run' / (prefix + '_result.json')
        with source.open('x') as stream:
            json.dump(request, stream, indent=2)
        self.record('paired_actual_planning_started', dict(mode=mode, request=str(source), source_sim=pair['pose_stamp']))
        with (EVIDENCE / 'run' / (prefix + '_worker.log')).open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'dual_worker.py'), str(source), str(output)],
                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=120)
        result = json.loads(output.read_text())
        assert result['status'] == 'PASS' and result['maximum_joint_step_rad'] == .002, result
        joint_drift = max(abs(self.positions[name] - positions[name]) for name in TASK_JOINTS)
        pose_drift = max(float(np.linalg.norm(pose_matrix(self.poses['box48_' + payload])[:3, 3] -
                    pose_matrix(poses['box48_' + payload])[:3, 3])) for payload in assignment.values())
        angle_drift = max(math.degrees(Rotation.from_matrix(pose_matrix(poses['box48_' + payload])[:3, :3].T @
                    pose_matrix(self.poses['box48_' + payload])[:3, :3]).magnitude()) for payload in assignment.values())
        self.record('paired_actual_snapshot_checked', dict(mode=mode, joint_drift_rad=joint_drift,
                    pose_drift_m=pose_drift, angle_drift_deg=angle_drift))
        assert joint_drift <= .0003 and pose_drift <= .0003 and angle_drift <= .1
        if mode in ('empty_gripper_after_retreat', 'empty_return_after_gripper'):
            from gripper_return_geometry import verify_finger_snapshot
            finger_drift = verify_finger_snapshot(self.positions, positions)
            self.record('post_retreat_actual_finger_snapshot_checked', dict(mode=mode, maximum_drift_m=finger_drift, original_bound_m=.0003))
        self.record('paired_actual_collision_plan', result)
        return result

    def check_pair_release(self):
        for hand, held in self.holders.rows.items():
            payload = held['payload']
            body = pose_matrix(self.poses['box48_' + payload])
            bucket = pose_matrix(self.poses['centrifuge']) @ pose_matrix(self.poses['bucket_' + payload])
            assert np.linalg.norm(body[:2, 3] - bucket[:2, 3]) <= .0005
            assert abs(body[2, 3] - 1.07525) <= .0005
            assert Rotation.from_matrix(Rotation.from_euler('z', .309270119).as_matrix().T @ body[:3, :3]).magnitude() <= math.radians(.25)
            corners = np.array(list(itertools.product((-1, 1), repeat=3))) * np.array([.1187450051085, .08814010043, .0595]) / 2
            local = (corners @ body[:3, :3].T + body[:3, 3] - bucket[:3, 3]) @ bucket[:3, :3]
            assert np.max(np.abs(local[:, 0])) <= .0448 and np.max(np.abs(local[:, 1])) <= .0668
            self.record('lower_release_gate_pass', dict(side=payload, hand=hand, poses=copy.deepcopy(self.poses),
                        nominal_floor_gap_m=.023, no_physical_attachment=True))

    def forget_released_pair(self, reason):
        for hand in tuple(self.holders.rows):
            payload = self.holders.rows[hand]['payload']
            self.record('dual_holder_normal_release', dict(hand=hand, payload=payload, reason=reason))
            self.record('coherent_guard_scope_end', dict(previous_key=[hand, payload], hand=hand,
                        source='qualified_normal_release_not_preclaimed_empty_physics'))
            self.holders.release(hand)
            self.holder_contexts.pop(hand)
        self.synchronize_primary()

    def observe_pair_drop(self):
        self.phase = 'paired_load_drop_settling'
        start = self.sim
        stable, references, accepted = {}, {}, set()
        while self.sim - start < 8:
            self.spin()
            for payload in ('left', 'right'):
                contacts = self.contacts[payload]
                qualified = fully_inside_bucket(self.poses, payload) and abs(self.values['machine']['rotor_rpm']) <= .02
                qualified = qualified and any('bucket_' + payload in item['first'] + item['second'] for item in contacts)
                qualified = qualified and not any('dex1' in item['first'] + item['second'] for item in contacts)
                corners = bucket_relative_corners(self.poses, payload)
                if not qualified:
                    accepted.discard(payload)
                    stable.pop(payload, None)
                    references.pop(payload, None)
                elif payload not in references or np.max(np.abs(corners - references[payload])) > .0005:
                    accepted.discard(payload)
                    references[payload], stable[payload] = corners, self.sim
                elif payload not in accepted and self.sim - stable[payload] >= 3:
                    accepted.add(payload)
                    self.record('drop_acceptance_pass', dict(side=payload, duration_sim_s=self.sim - stable[payload],
                                corners_bucket_m=corners.tolist(), rotor_rpm=self.values['machine']['rotor_rpm']))
            if len(accepted) == 2:
                self.drop_window_until = None
                return
        raise RuntimeError('Both payloads did not naturally fall completely into their own baskets')

    def load_pair(self):
        assignment = dict(left='left', right='right')
        self.load_table_guard_active = True
        self.prepare_loading_grippers()
        pickup = self.actual_plan('load_pickup', assignment)
        for stage in pickup['stages']:
            self.move(stage['samples'], 'paired_load_' + stage['phase'], .8)
        self.paired_grippers(-.01901, 16, 'paired_load_grasp')
        self.capture_pair(assignment)
        self.load_table_guard_active = False
        pilot = self.actual_plan('pilot', assignment)
        self.move(pilot['stages'][0]['samples'], 'paired_load_pilot_lift', .03)
        self.dwell(.5)
        for payload in assignment.values():
            assert pose_matrix(self.poses['box48_' + payload])[2, 3] > .85475
            assert not any('table::' in item['first'] + item['second'] for item in self.contacts[payload])
        self.phase = 'paired_load_static_grip_hold'
        began = self.sim
        self.dwell(10)
        for hand, payload in assignment.items():
            self.record('static_grip_hold_pass', dict(side=hand, payload=payload, duration_sim_s=self.sim - began,
                        each_actual_holder_guarded=True, native_slip_m=self.max_native_slip))
        transport = self.actual_plan('load_transport', assignment)
        retreat_guides = copy.deepcopy(transport['stages'])
        for index, stage in enumerate(transport['stages']):
            self.move(stage['samples'], 'paired_load_' + stage['phase'], .08 if index == len(transport['stages']) - 1 else .25)
        self.dwell(.5)
        try:
            self.check_pair_release()
        except AssertionError:
            correction = self.actual_plan('load_alignment', assignment)
            self.move(correction['stages'][0]['samples'], 'paired_load_alignment_correction', .10)
            retreat_guides.extend(copy.deepcopy(correction['stages']))
            self.dwell(.5)
            self.check_pair_release()
        self.forget_released_pair('both_actual_overhead_gates_passed')
        self.drop_window_until = self.sim + 8
        self.paired_grippers(-.0199, 0, 'paired_load_release_above_bucket')
        self.observe_pair_drop()
        self.qualified_empty_retreat_guides = retreat_guides
        self.record('empty_retreat_current_run_guides_qualified', dict(identity=EVIDENCE.name,
                    loading_and_optional_correction_executed=True, both_actual_drops_passed=True,
                    guides_only_route_proposals_not_measured_state_or_grip_reference=True,
                    stages=len(retreat_guides)))
        self.record('LOADING_PASS', dict(scope='two_box_overhead_loading', loading_complete=True,
                    actual_two_holder_cycle=True, poses=copy.deepcopy(self.poses), motions=self.motion_count,
                    max_slip_m=self.max_slip))

    def return_neutral(self):
        assert not self.holders.rows
        planned = self.actual_plan('empty_return', dict(left='left', right='right'))
        for index, stage in enumerate(planned['stages']):
            self.move(stage['samples'], 'machine_paired_' + stage['phase'], .13 if index == 0 else .8)
        self.paired_grippers(-.019, 0, 'machine_paired_empty_grippers')

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
        self.spin()
        rotor = pose_matrix(self.poses['rotor'])
        phase = math.atan2(rotor[1, 0], rotor[0, 0])
        planned = schedule_nominal_phase(self.steady_start, self.pose_time, phase,
                                        self.values['machine']['rotor_rad_s'], start_event['sim_time'] - start_stamp)
        self.record('phase_aligned_stop_schedule', planned)
        stop_stamp = planned['stroke_start_sim']
        self.wait_until_sim(stop_stamp - .5)
        stop_event, unused = self.press('right', 'machine_stop', scheduled=stop_stamp)
        duration = stop_event['sim_time'] - self.steady_start
        assert duration >= 30 and duration <= 32.4, duration
        self.phase = 'machine_stop_wait_quiet'
        self.wait(lambda: self.values['machine']['can_open'] and not self.values['machine']['rotor_enabled'] and
                  abs(self.values['machine']['rotor_rad_s']) < .02)
        self.record('rotor_stopped_before_lid', self.values['machine'])
        self.press('left', 'machine_final_open')
        self.phase = 'machine_wait_final_open'
        self.wait(lambda: self.lid_ready(math.radians(120)))
        self.dwell(3)
        self.record('MACHINE_CYCLE_PASS', dict(steady_run_sim_seconds=duration, final_machine=self.values['machine'],
                    retention_checks=self.retention_checks, retention_peaks=self.retention_peaks,
                    maximum_zero_force_gap_sim_s={payload: guard.maximum_gap_s for payload, guard in self.support_continuity.items()},
                    phase_aligned_by_real_physical_stop_button=True, original_30_3_window_passed=duration <= 30.3,
                    user_required_minimum_thirty_steady_seconds_preserved=True,
                    actual_final_phase_not_assumed_equal_prediction=True))
        self.loaded = False

    def retrieve(self):
        payloads = sorted(('left', 'right'), key=lambda payload: pose_matrix(self.poses['box48_' + payload])[1, 3])
        assignment = dict(right=payloads[0], left=payloads[1])
        self.phase = 'paired_retrieval_pickup_planning'
        self.paired_grippers(-.0199, 0, 'paired_retrieval_open_empty')
        pickup = self.actual_plan('retrieval_pickup', assignment, retrieval=True)
        for stage in pickup['stages']:
            self.move(stage['samples'], 'paired_retrieval_' + stage['phase'], .8)
        self.paired_grippers(-.01901, 16, 'paired_retrieval_grasp', retrieval=True)
        self.capture_pair(assignment, retrieval=True)
        pilot = self.actual_plan('pilot', assignment, retrieval=True)
        self.move(pilot['stages'][0]['samples'], 'paired_retrieval_pilot_lift', .03)
        transport = self.actual_plan('retrieval_transport', assignment, retrieval=True)
        for stage in transport['stages']:
            phase = stage['phase']
            if phase == 'setdown':
                coarse, fine, split = split_paired_setdown(stage)
                for hand, payload in assignment.items():
                    self.record('retrieval_setdown_braking', dict(phase=hand + '_retrieval_setdown', payload=payload,
                                split_index=split, original_points=len(stage['samples']), unchanged_path=True,
                                fine_rate_rad_s=self.fine_setdown_rate(), coarse_rate_rad_s=self.coarse_setdown_rate(), simultaneous_common_path=True))
                self.move(coarse, 'paired_retrieval_setdown', self.coarse_setdown_rate())
                self.move(fine, 'paired_retrieval_setdown_fine', self.fine_setdown_rate())
                continue
            self.move(stage['samples'], 'paired_retrieval_' + phase, .25)
            if phase == 'lift':
                self.pair_clearance_payloads = set(assignment.values())
                self.safety()
                self.phase = 'paired_retrieval_static_grip_hold'
                began = self.sim
                self.dwell(10)
                for hand, payload in assignment.items():
                    self.record('retrieval_static_grip_hold_pass', dict(hand=hand, payload=payload,
                        duration_sim_s=self.sim - began, hold_after_full_lift=True,
                        clearance=clearance_metrics(self.poses, payload, self.contacts[payload], self.bucket_rim_z[payload])))
        self.pair_clearance_payloads.clear()
        self.return_hands = {payload: hand for hand, payload in assignment.items()}
        self.return_table_yaws = transport['table_yaws']
        self.dwell(.5)
        for hand, payload in assignment.items():
            expected = np.array([.44, .38 if hand == 'left' else -.38, .84475])
            assert np.linalg.norm(pose_matrix(self.poses['box48_' + payload])[:3, 3] - expected) < .002
            assert any('table::' in item['first'] + item['second'] for item in self.contacts[payload])
            assert table_orientation_error(pose_matrix(self.poses['box48_' + payload]), self.return_table_yaws[payload]) < math.radians(2)
            self.record('return_table_target', dict(hand=hand, payload=payload,
                        yaw_rad=self.return_table_yaws[payload], actual_target_not_runtime_reference_rebase=True))
        self.forget_released_pair('both_actual_table_support_and_pose_gates_passed')
        self.paired_grippers(-.0199, 0, 'paired_retrieval_release_on_table')
        references = {payload: pose_matrix(self.poses['box48_' + payload]) for payload in assignment.values()}
        start = self.sim
        while self.sim - start < 3:
            self.spin()
            for payload in assignment.values():
                body = pose_matrix(self.poses['box48_' + payload])
                assert any('table::' in item['first'] + item['second'] for item in self.contacts[payload])
                assert not any('dex1' in item['first'] + item['second'] for item in self.contacts[payload])
                assert np.linalg.norm(body[:3, 3] - references[payload][:3, 3]) < .0005
        for payload in assignment.values():
            self.verified_table_payloads.add(payload)
            self.record('table_return_pass', dict(payload=payload, hand=self.return_hands[payload], duration_sim_s=self.sim - start))
        self.terminal_hold(10)
        assert not self.holders.rows and self.holding is None and self.held_payload is None
        assert set(self.return_hands.values()) == {'left', 'right'}
        assert [event['button'] for event in self.events] == ['lid', 'lid', 'rotor', 'rotor', 'lid']
        assert self.values['machine']['starts'] == self.values['machine']['stops'] == 1
        self.record('PASS', dict(scope='two_box_load_close_spin_open_retrieve', paired_actual_cycle=True,
                    physical_press_events=self.events, final_machine=self.values['machine'], poses=self.poses,
                    motions=self.motion_count, max_slip_m=self.max_slip, max_native_slip_m=self.max_native_slip,
                    actual_holder_registry=self.holders.snapshot()))

    def run_complete(self):
        self.record('simulation_relaxation_declared', dict(user_authorized=True, retrieval_transient_limit_rpm=.1,
                    carry_angle_limit_deg=5, original_strict_certificate_not_claimed=True))
        self.record('paired_runtime_declared', dict(physical_controllers=True, fixed_pelvis_x_m=.12,
                    device_x_m=.52, two_independent_actual_holders=True, no_physical_attach_detach=True,
                    common_arm_timestamps=True, original_20ms_2mm_5deg_and_3_10_30_certificates=True,
                    actual_history_capacity=256, missing_pair_real_callback_drain_bound_wall_s=.010,
                    maximum_actual_callback_drain=64, no_extrapolation_reference_rebase_or_ignored_native=True))
        self.prepare_open()
        self.load_pair()
        self.machine_cycle()
        self.retrieve()

    def hold(self, error):
        self.record('paired_failure_holder_state', dict(actual_holders=self.holders.snapshot(),
                    accepted_gripper_targets=self.accepted_gripper_targets, accepted_efforts=self.accepted_gripper_efforts))
        for handle in self.pair_goal_handles.values():
            handle.cancel_goal_async()
        for future in self.pair_goal_futures.values():
            if future.done() and future.result().accepted:
                future.result().cancel_goal_async()
        return super().hold(error)
