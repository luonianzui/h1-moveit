import copy
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

from alignment_wait import wait_for_planner
from dual_motion import TASK_JOINTS, validate_common_start
from endpoint_overlap_cycle import EndpointOverlapCycle
from execute_dual import DualCycle
from execute_loading import GROUP_JOINTS, fully_inside_bucket, bucket_relative_corners, pose_matrix
from extraction_clearance import clearance_metrics
from fast_profile import RETRIEVAL_SOFT_GRASP_SECONDS
from loaded_retention import retention_metrics
from native_gripper_feedback import wait_original_gripper_endpoint
from sequential_geometry import CONTRACT
from sequential_force import RETRIEVAL_FORCE_N, retrieval_profile
from sequential_support import actual_preload_support, actual_preload_table_continuity, original_basket_preload_gap_valid
from sequential_order import SequentialOrder
from soft_grasp import soft_grasp_profile
from terminal_policy import table_terminal_metrics, table_support_valid
from table_return_goal import table_orientation_error
from bounded_release_alignment import align_before_release
from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from pilot_settle import settle_retrieval_pilot
from single_setdown import execute_single_setdown, single_setdown_timing


EVIDENCE = Path(__file__).resolve().parent


class SequentialCycle(EndpointOverlapCycle):
    def __init__(self, plan, buttons):
        self.closing_floor_history = FloorContactHistory()
        self.order = SequentialOrder()
        self.remaining_guard_enabled = False
        self.release_transition = None
        self.table_references = {}
        self.table_positive = {}
        self.remaining_last_record = -1.
        super().__init__(plan, buttons)

    def safety(self):
        if len(self.holders.rows) > 1:
            raise RuntimeError('Sequential cycle cannot have two actual holders')
        super().safety()
        if not self.remaining_guard_enabled:
            return
        held = {row['payload'] for row in self.holders.rows.values()}
        rows = {}
        for payload, location in self.order.locations.items():
            if payload in held:
                continue
            if self.release_transition is not None and payload == self.release_transition['payload']:
                if self.sim > self.release_transition['deadline_sim']:
                    raise RuntimeError('Qualified release did not finish its original observation window')
                continue
            if location == 'held':
                raise RuntimeError('Unobserved holder disappearance in sequential cycle')
            if location == 'basket':
                pairs = self.contacts[payload]
                active_hand = getattr(self, 'current_grasp_hand', None)
                targeted = (getattr(self, 'grasp_candidate', None) == payload and not self.holders.rows
                            and active_hand in ('left', 'right') and self.phase == active_hand + '_retrieval_grasp')
                own_prefix = str(active_hand) + '_dex1_finger_'
                def current_grasp_contact(pair):
                    names = pair['first'] + ' ' + pair['second']
                    return targeted and own_prefix in names and 'box48_' + payload + '::' in names
                support_pairs = [pair for pair in pairs if not current_grasp_contact(pair)]
                metrics = retention_metrics(self.poses, support_pairs, payload)
                continuity = self.support_continuity[payload].update(self.sim, metrics)
                foreign_finger = any('dex1' in pair['first'] + pair['second'] and not current_grasp_contact(pair) for pair in pairs)
                if (not continuity['valid'] and targeted and not foreign_finger and
                        metrics['sidewall_certified'] and metrics['vertical_inside']):
                    certificate = actual_preload_support(self, payload, retrieval=True)
                    metrics['actual_retrieval_preload_support'] = certificate
                    if certificate['valid']:
                        continuity['valid'] = True
                        continuity['support_source'] = 'actual_opposed_retrieval_preload'
                    elif original_basket_preload_gap_valid(certificate, continuity):
                        continuity['valid'] = True
                        continuity['support_source'] = 'original_basket_positive_history_with_actual_opposed_contacts'
                        continuity['not_a_strong_grip_certificate'] = True
                    if not continuity['valid']:
                        frame = single_missing_floor_frame(self, payload, metrics)
                        metrics['single_actual_floor_frame'] = frame
                        if frame['valid']:
                            self.record('sequential_single_frame_floor_continuity', dict(payload=payload,
                                        hand=active_hand, certificate=frame, metrics=metrics,
                                        actual_holders=self.holders.snapshot()))
                            continuity['valid'] = True
                            continuity['support_source'] = frame['source']
                            continuity['not_a_strong_grip_certificate'] = True
                if not continuity['valid'] or foreign_finger:
                    self.record('sequential_remaining_basket_fault', dict(payload=payload, metrics=metrics,
                                continuity=continuity, poses=copy.deepcopy(self.poses), contacts=self.contacts[payload]))
                    raise RuntimeError('Unheld remaining box lost original basket support/containment')
                rows[payload] = dict(location=location, metrics=metrics, continuity=continuity,
                    qualified_current_grasp_contact_count=len(pairs) - len(support_pairs),
                    all_raw_contacts_still_checked_by_original_force_depth_guards=True)
            elif location == 'table':
                body = pose_matrix(self.poses['box48_' + payload])
                if payload in self.return_hands:
                    hand = self.return_hands[payload]
                    expected = [.44, .38 if hand == 'left' else -.38, .84475]
                    yaw = self.return_table_yaws[payload]
                else:
                    expected = [.44, .38 if payload == 'left' else -.38, .84475]
                    yaw = 0.
                    if np.linalg.norm(body[:3, 3] - expected) >= .0003 or Rotation.from_matrix(body[:3, :3]).magnitude() >= math.radians(.2):
                        raise RuntimeError('Unpicked table box left its original birth pose')
                reference = self.table_references.setdefault(payload, body[:3, 3].copy())
                metrics = table_terminal_metrics(self.poses, payload, self.contacts[payload], expected, yaw, reference)
                if metrics['supported']:
                    self.table_positive[payload] = self.sim
                continuity = table_support_valid(metrics, self.sim, self.table_positive.get(payload))
                candidate = getattr(self, 'grasp_candidate', None)
                if candidate == payload:
                    own = self.current_grasp_hand + '_dex1_finger_'
                    contacts_valid = all('table::' in pair['first'] + pair['second'] or own in pair['first'] + pair['second']
                                         for pair in self.contacts[payload])
                    metrics['geometry_valid'] = bool(np.linalg.norm(body[:3, 3] - expected) < .0003)
                    continuity['valid'] = bool(metrics['geometry_valid'] and metrics['supported'] and contacts_valid)
                    continuity['original_table_support_valid'] = continuity['valid']
                    if not continuity['valid'] and metrics['geometry_valid'] and contacts_valid:
                        certificate = actual_preload_support(self, payload)
                        metrics['actual_preload_support'] = certificate
                        continuity['valid'] = certificate['valid']
                        continuity['support_source'] = 'actual_opposed_preload' if certificate['valid'] else 'unqualified'
                        if not certificate['valid']:
                            original_window = actual_preload_table_continuity(self, payload)
                            metrics['original_preload_table_continuity'] = original_window
                            if original_window['valid']:
                                continuity['valid'] = True
                                continuity['support_source'] = 'original_table_force_continuity'
                if not continuity['valid']:
                    self.record('sequential_remaining_table_fault', dict(payload=payload, metrics=metrics,
                                continuity=continuity, contacts=self.contacts[payload], poses=copy.deepcopy(self.poses)))
                    raise RuntimeError('Unheld table box lost original support/pose/contact')
                rows[payload] = dict(location=location, metrics=metrics, continuity=continuity)
        if self.pose_time - self.remaining_last_record >= .025:
            self.remaining_last_record = self.pose_time
            self.record('sequential_remaining_support', dict(rows=rows, order=self.order.snapshot(),
                        actual_holders=self.holders.snapshot(), native_pose_sim=self.pose_time))

    def paired_feedback_overlap_enabled(self):
        return False

    def paired_endpoint_stamp(self, target, not_before):
        if self.joint_time < not_before or any(abs(self.positions[name] - value) >= .0003 for name, value in target.items()):
            return None
        return self.joint_time

    def maximum_loaded_rate(self, phase):
        return .25

    def motion_limits(self, phase, requested):
        if phase == 'seq_retrieval_retrieve_axis_exit':
            return min(requested, .03), .002
        near = any(token in phase for token in ('descend', 'lower', 'pilot', 'vertical_retreat', 'setdown'))
        cap = .03 if 'pilot' in phase or 'setdown_fine' in phase else .13 if near else .25 if self.holders.rows else .35
        return min(requested, cap), .002 if near else .015

    def move(self, samples, phase, rate):
        return DualCycle.move(self, samples, phase, rate)

    def common_timing(self, actual, samples, rate):
        if self.phase == 'paired_retrieval_setdown':
            return single_setdown_timing(self, actual, samples, rate)
        return super().common_timing(actual, samples, rate)

    def single_gripper(self, hand, target, effort, phase, retrieval=False):
        if hand not in ('left', 'right') or effort not in (0., 16., RETRIEVAL_FORCE_N) or retrieval != (effort == RETRIEVAL_FORCE_N):
            raise ValueError('Explicit original single-hand force phase required')
        self.phase = phase
        self.safety()
        self.retrieval_closing = retrieval
        start = self.sim + self.future_start_offset()
        seconds, nanoseconds = validate_common_start(self.sim, start)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GROUP_JOINTS[hand + '_gripper']
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = seconds, nanoseconds
        if retrieval:
            profile = retrieval_profile([self.positions[name] for name in goal.trajectory.joint_names], target)
            for sample in profile:
                point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'],
                                             effort=sample['effort'])
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
                goal.trajectory.points.append(point)
            self.record('retrieval_soft_grasp_profile', dict(side=hand, profile=profile,
                        final_effort_N=RETRIEVAL_FORCE_N, duration_sim_s=RETRIEVAL_SOFT_GRASP_SECONDS,
                        effort_scale=RETRIEVAL_FORCE_N / 16., parent_feedforward_N=18.,
                        unchanged_output_guard_N=18.01, pid_headroom_experiment_not_guard_relaxation=True))
        else:
            goal.trajectory.points = self.gripper_trajectory_points(hand, target, hand + '_grasp' if effort else hand + '_paired_empty', effort)
        goal.goal_tolerance = [JointTolerance(name=name, position=.0002) for name in goal.trajectory.joint_names]
        goal.goal_time_tolerance.sec = 6 if effort else 3
        if not self.gripper_actions[hand].server_is_ready():
            raise RuntimeError('Single gripper action unavailable')
        pending = self.gripper_actions[hand].send_goal_async(goal)
        self.pair_goal_futures = {hand: pending}
        deadline = time.monotonic() + (240 if retrieval else 90)
        while not pending.done():
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Single gripper acceptance timeout')
        handle = pending.result()
        if not handle.accepted:
            raise RuntimeError('Single gripper goal rejected')
        self.pair_goal_handles = {hand: handle}
        self.accepted_gripper_targets[hand] = target
        self.accepted_gripper_efforts[hand] = float(effort)
        if effort in (16., RETRIEVAL_FORCE_N) and getattr(self, 'grasp_candidate', None) in ('left', 'right'):
            self.preload_grasp_claim = dict(accepted=True, hand=hand, payload=self.grasp_candidate,
                                           accepted_common_start_sim=start)
            self.record('sequential_preload_claim', self.preload_grasp_claim)
        self.motion_count += 1
        self.record('gripper_goal_accepted', dict(side=hand, target=target, feedforward_N=effort,
                    common_start_sim=start, actual_pair_action=False, sequential_single_action=True))
        complete = handle.get_result_async()
        while not complete.done():
            self.spin()
            if time.monotonic() > deadline:
                raise TimeoutError('Single gripper result timeout')
        response = complete.result()
        self.record('gripper_result', dict(side=hand, target=target, status=response.status,
                    error_code=response.result.error_code, error=response.result.error_string))
        if response.result.error_code != 0:
            raise RuntimeError(response.result.error_string)
        wait_original_gripper_endpoint(self, hand, target, self.sim)
        self.pair_goal_handles.clear()
        self.pair_goal_futures.clear()
        self.retrieval_closing = False
        self.dwell(.3)

    def single_plan(self, mode, hand, payload):
        self.safety()
        pair = self.measured_pair()
        positions, poses = dict(pair['joints']['positions']), copy.deepcopy(pair['poses'])
        self.state.joint_positions = positions
        self.state.update()
        reference = np.linalg.inv(self.state.get_global_link_transform(hand + '_wrist_yaw_link')) @ pose_matrix(poses['box48_' + payload])
        request = dict(mode=mode, hand=hand, payload=payload, sequential_contract=CONTRACT,
                       positions=positions, poses=poses, source_sim=pair['pose_stamp'], planning_reference=reference.tolist(),
                       original_runtime_holder_references=self.holders.snapshot(), runtime_reference_not_rebased=True)
        self.worker_sequence += 1
        prefix = str(self.worker_sequence) + '_' + mode + '_' + hand
        source, output = [EVIDENCE / 'run' / (prefix + suffix) for suffix in ('_request.json', '_result.json')]
        with source.open('x') as stream:
            json.dump(request, stream, indent=2)
        self.record('sequential_actual_planning_started', dict(mode=mode, hand=hand, payload=payload,
                    request=str(source), source_sim=pair['pose_stamp']))
        with (EVIDENCE / 'run' / (prefix + '_worker.log')).open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'sequential_worker.py'), str(source), str(output)],
                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=120)
        planned = json.loads(output.read_text())
        if planned['status'] != 'PASS_GEOMETRY_ONLY' or planned['maximum_joint_step_rad'] != .002:
            raise RuntimeError('Single-arm worker did not pass original geometry checks')
        if planned['original_runtime_holder_references'] != self.holders.snapshot():
            raise RuntimeError('Original actual holder anchors changed while planning')
        joint_drift = max(abs(self.positions[name] - positions[name]) for name in TASK_JOINTS)
        pose_drift = max(float(np.linalg.norm(pose_matrix(self.poses['box48_' + current])[:3, 3] -
                         pose_matrix(poses['box48_' + current])[:3, 3])) for current in ('left', 'right'))
        angle_drift = max(math.degrees(Rotation.from_matrix(pose_matrix(poses['box48_' + current])[:3, :3].T @
                          pose_matrix(self.poses['box48_' + current])[:3, :3]).magnitude()) for current in ('left', 'right'))
        finger_drift = max(abs(self.positions[name] - positions[name]) for name in positions if 'dex1_finger' in name)
        self.record('sequential_actual_snapshot_checked', dict(mode=mode, joint_drift_rad=joint_drift,
                    pose_drift_m=pose_drift, angle_drift_deg=angle_drift, finger_drift_m=finger_drift))
        if joint_drift > .0003 or pose_drift > .0003 or angle_drift > .1 or finger_drift > .0003:
            raise RuntimeError('Actual sequential planning snapshot drifted past original bounds')
        self.record('sequential_actual_collision_plan', planned)
        return planned

    def execute_stages(self, planned, prefix):
        for stage in planned['stages']:
            phase = stage['phase']
            if phase == 'retrieve_setdown':
                samples = stage['samples']
                split = max(1, len(samples) - 8)
                self.record('retrieval_setdown_braking', dict(phase=planned['single_hand'] + '_retrieval_setdown',
                    payload=planned['single_payload'], split_index=split, original_points=len(samples),
                    unchanged_path=True, fine_rate_rad_s=.03, coarse_rate_rad_s=.06 if planned['single_hand']=='right' else .13, sequential_single_holder=True))
                execute_single_setdown(self, planned, samples, split)
            else:
                self.move(stage['samples'], prefix + '_' + phase, .25 if self.holders.rows else .35)
            if phase == 'retrieve_lift':
                payload = planned['single_payload']
                self.pair_clearance_payloads = {payload}
                self.safety()
                self.phase = prefix + '_static_grip_hold'
                began = self.sim
                self.dwell(10)
                self.record('retrieval_static_grip_hold_pass', dict(hand=planned['single_hand'], payload=payload,
                            duration_sim_s=self.sim - began, hold_after_full_lift=True,
                            clearance=clearance_metrics(self.poses, payload, self.contacts[payload], self.bucket_rim_z[payload])))

    def observe_drop(self, payload):
        self.phase = payload + '_drop_settling'
        began, stable, reference = self.sim, None, None
        while self.sim - began < 8:
            self.spin()
            pairs = self.contacts[payload]
            qualified = (fully_inside_bucket(self.poses, payload) and abs(self.values['machine']['rotor_rpm']) <= .02
                and any('bucket_' + payload in pair['first'] + pair['second'] for pair in pairs)
                and not any('dex1' in pair['first'] + pair['second'] for pair in pairs))
            corners = bucket_relative_corners(self.poses, payload)
            if not qualified:
                stable, reference = None, None
            elif reference is None or np.max(np.abs(corners - reference)) > .0005:
                stable, reference = self.sim, corners
            elif self.sim - stable >= 3:
                self.record('drop_acceptance_pass', dict(side=payload, duration_sim_s=self.sim - stable,
                            corners_bucket_m=corners.tolist(), rotor_rpm=self.values['machine']['rotor_rpm']))
                return self.sim - stable
        raise RuntimeError('Single payload did not naturally settle completely inside its basket')

    def load_pair(self):
        self.phase = 'sequential_load_preparation'
        self.remaining_guard_enabled = True
        for hand in ('left', 'right'):
            self.single_gripper(hand, -.0199, 0., hand + '_seq_load_open_empty')
        self.execute_stages(self.single_plan('park', 'left', 'left'), 'seq_load')
        for hand in ('left', 'right'):
            pickup = self.single_plan('load_pickup', hand, hand)
            self.execute_stages(pickup, 'seq_load')
            self.grasp_candidate = hand
            self.current_grasp_hand = hand
            self.single_gripper(hand, -.01901, 16., hand + '_seq_load_grasp')
            self.capture_pair({hand: hand})
            self.order.begin(hand, hand, self.holders.snapshot())
            self.preload_grasp_claim = None
            self.grasp_candidate = None
            self.execute_stages(self.single_plan('load_pilot', hand, hand), 'seq_load')
            self.dwell(.5)
            if pose_matrix(self.poses['box48_' + hand])[2, 3] <= .85475 or any('table::' in pair['first'] + pair['second'] for pair in self.contacts[hand]):
                raise RuntimeError('Single pilot lift did not clear table')
            self.phase = hand + '_seq_load_static_grip_hold'
            began = self.sim
            self.dwell(10)
            self.record('static_grip_hold_pass', dict(side=hand, payload=hand, duration_sim_s=self.sim - began,
                        each_actual_holder_guarded=True, native_slip_m=self.max_native_slip))
            self.execute_stages(self.single_plan('load_transport', hand, hand), 'seq_load')
            self.dwell(.5)
            align_before_release(self, hand, hand)
            self.check_pair_release()
            self.release_transition = dict(payload=hand, deadline_sim=self.sim + 12)
            self.forget_released_pair('single_actual_overhead_gate_passed')
            self.drop_window_until = self.sim + 8
            self.single_gripper(hand, -.0199, 0., hand + '_release_above_bucket')
            seconds = self.observe_drop(hand)
            self.order.finish_loading(hand, hand, self.holders.snapshot(), seconds)
            self.release_transition = None
            self.drop_window_until = None
            self.execute_stages(self.single_plan('empty_retreat', hand, hand), 'seq_load')
            self.record('sequential_loaded_one', self.order.snapshot())
        self.record('LOADING_PASS', dict(scope='two_box_overhead_loading', loading_complete=True,
                    actual_two_holder_cycle=False, sequential_cycle=True, poses=copy.deepcopy(self.poses)))

    def return_neutral(self):
        self.execute_stages(self.single_plan('neutral', 'left', 'left'), 'machine_sequential')
        for hand in ('left', 'right'):
            self.single_gripper(hand, -.019, 0., 'machine_sequential_empty_grippers')

    def retrieve(self):
        payloads = sorted(('left', 'right'), key=lambda value: pose_matrix(self.poses['box48_' + value])[1, 3])
        assignment = dict(left=payloads[1], right=payloads[0])
        machine_event = self.machine_cycle_result
        self.order.finish_machine(self.holders.snapshot(), machine_event['steady_run_sim_seconds'], self.values['machine'], assignment)
        self.phase = 'seq_retrieval_prepare'
        for hand in ('left', 'right'):
            self.single_gripper(hand, -.0199, 0., hand + '_retrieval_open_empty')
        self.execute_stages(self.single_plan('park', 'left', assignment['left']), 'seq_retrieval')
        for hand in ('left', 'right'):
            payload = assignment[hand]
            self.execute_stages(self.single_plan('retrieval_pickup', hand, payload), 'seq_retrieval')
            self.grasp_candidate, self.current_grasp_hand = payload, hand
            self.single_gripper(hand, -.01901, RETRIEVAL_FORCE_N, hand + '_retrieval_grasp', retrieval=True)
            self.capture_pair({hand: payload}, retrieval=True)
            self.order.begin(hand, payload, self.holders.snapshot())
            self.grasp_candidate = None
            self.execute_stages(self.single_plan('retrieval_pilot', hand, payload), 'seq_retrieval')
            settle_retrieval_pilot(self, hand, payload)
            planned = self.single_plan('retrieval_transport', hand, payload)
            self.execute_stages(planned, 'seq_retrieval')
            self.pair_clearance_payloads.clear()
            self.return_hands[payload] = hand
            self.return_table_yaws[payload] = planned['table_yaw_rad']
            self.dwell(.5)
            body = pose_matrix(self.poses['box48_' + payload])
            if (np.linalg.norm(body[:3, 3] - [.44, .38 if hand == 'left' else -.38, .84475]) >= .002
                    or not any('table::' in pair['first'] + pair['second'] for pair in self.contacts[payload])
                    or table_orientation_error(body, planned['table_yaw_rad']) >= math.radians(2)):
                raise RuntimeError('Actual single-box table support/pose gate failed')
            self.release_transition = dict(payload=payload, deadline_sim=self.sim + 12)
            self.record('return_table_target', dict(hand=hand, payload=payload,
                        yaw_rad=planned['table_yaw_rad'], actual_target_not_runtime_reference_rebase=True))
            self.forget_released_pair('single_actual_table_support_and_pose_gates_passed')
            self.single_gripper(hand, -.0199, 0., hand + '_retrieval_release_on_table')
            reference = pose_matrix(self.poses['box48_' + payload])[:3, 3].copy()
            began = self.sim
            while self.sim - began < 3:
                self.spin()
                body = pose_matrix(self.poses['box48_' + payload])
                if (not any('table::' in pair['first'] + pair['second'] for pair in self.contacts[payload])
                        or any('dex1' in pair['first'] + pair['second'] for pair in self.contacts[payload])
                        or np.linalg.norm(body[:3, 3] - reference) >= .0005):
                    raise RuntimeError('Single table release did not remain supported and stable')
            self.order.finish_return(hand, payload, self.holders.snapshot(), self.sim - began)
            self.release_transition = None
            self.table_references[payload] = reference
            self.verified_table_payloads.add(payload)
            self.record('table_return_pass', dict(payload=payload, hand=hand, duration_sim_s=self.sim - began))
            self.execute_stages(self.single_plan('empty_retreat', hand, payload), 'seq_retrieval')
        began = self.sim
        self.terminal_hold(10)
        self.order.finish_terminal(self.holders.snapshot(), self.sim - began)
        if ([event['button'] for event in self.events] != ['lid', 'lid', 'rotor', 'rotor', 'lid']
                or self.values['machine']['starts'] != 1 or self.values['machine']['stops'] != 1):
            raise RuntimeError('Original five physical button events missing')
        self.record('PASS', dict(scope='two_box_load_close_spin_open_retrieve', paired_actual_cycle=False,
                    sequential_actual_cycle=True, order=self.order.snapshot(), physical_press_events=self.events,
                    final_machine=self.values['machine'], poses=self.poses, motions=self.motion_count,
                    max_slip_m=self.max_slip, max_native_slip_m=self.max_native_slip,
                    actual_holder_registry=self.holders.snapshot()))

    def record(self, kind, payload):
        if kind == 'contact':
            self.closing_floor_history.observe(payload)
        if kind == 'MACHINE_CYCLE_PASS':
            self.machine_cycle_result = copy.deepcopy(payload)
        return super().record(kind, payload)

    def run_complete(self):
        self.record('simulation_relaxation_declared', dict(user_authorized=True, retrieval_transient_limit_rpm=.1,
                    carry_angle_limit_deg=5, original_strict_certificate_not_claimed=True))
        self.record('sequential_runtime_declared', dict(physical_controllers=True, fixed_pelvis_x_m=.12,
                    device_x_m=.52, maximum_actual_holders=1, both_payloads_required=True,
                    no_physical_attach_detach=True, unchanged_observations_sim_s=[3, 10, 30],
                    original_native_velocity_limit_m_s=.2, original_output_guard_N=18.01))
        self.prepare_open()
        self.load_pair()
        self.machine_cycle()
        self.retrieve()
