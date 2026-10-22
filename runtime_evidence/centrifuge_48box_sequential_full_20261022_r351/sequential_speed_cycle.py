from sequential_cycle import *
from execute_dual import JointTrajectory, SimulationProgressWatch
from rclpy.action import ActionClient
from leg_initialization_contract import LEG_JOINTS, endpoint_error
from leg_initialization_runtime import verify_initialization
from single_setdown import require_single_setdown_holder
from speed_order import SpeedPriorityOrder
from speed_policy import PROFILE, common_timing, dwell_duration, motion_limits, gripper_velocity_violation
from speed_feedback_cycle import SpeedFeedbackMixin
from runtime_performance import RuntimePerformanceMixin
from preload_observation import PreloadObservationMixin
from planning_pipeline import PlanningPipelineMixin
from joined_stages import joined_timing


class SpeedPriorityCycle(PlanningPipelineMixin, RuntimePerformanceMixin, PreloadObservationMixin, SpeedFeedbackMixin, SequentialCycle):
    pose_clock_skew_limit_sim_s = PROFILE['pose_clock_skew_sim_s']

    def gripper_velocity_violation(self, value):
        return gripper_velocity_violation(value)

    def button_noncontact_rate(self):
        return min(super().button_noncontact_rate(), PROFILE['empty_button_rate_cap_rad_s'])

    def __init__(self, plan, buttons):
        super().__init__(plan, buttons)
        self.order = SpeedPriorityOrder()

    def move_button(self, side, points, fixed_interval=None, scheduled=None):
        self.safety()
        assert self.holding is None and self.held_payload is None
        start = self.sim + self.future_start_offset() if scheduled is None else scheduled
        assert start >= self.sim + .15, 'Missed physical button schedule'
        arm = self.buttons['arms'][side]
        message = JointTrajectory(joint_names=arm['joints'])
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(start * 1e9), 1000000000)
        elapsed = .05
        previous = None
        times = []
        travel_rate = self.button_noncontact_rate() if fixed_interval is None else .8
        for values in points:
            positions = np.array(values)
            if previous is not None:
                elapsed += PROFILE['button_contact_interval_sim_s'] if fixed_interval is not None else max(
                    .02, float(np.max(np.abs(positions - previous))) / travel_rate)
            point = JointTrajectoryPoint(positions=positions.tolist())
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(elapsed * 1e9), 1000000000)
            message.points.append(point)
            times.append(elapsed)
            previous = positions
        message.points = self.button_points_before_publish(message.points, fixed_interval)
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        elapsed = times[-1]
        tracking_points = [list(point.positions) for point in message.points]
        self.publishers[side + '_arm'].publish(message)
        self.motion_count += 1
        self.record('button_arm_command', dict(side=side, start=start, duration=elapsed,
            actual_travel_rate_rad_s=travel_rate, actual_fixed_contact_interval_sim_s=PROFILE['button_contact_interval_sim_s'],
            unchanged_geometry=True, points=[dict(positions=list(point.positions), velocities=list(point.velocities), seconds=seconds)
                for point, seconds in zip(message.points, times)]))
        target = dict(zip(arm['joints'], points[-1]))

        def reached():
            relative = self.joint_time - start
            if times[0] <= relative <= times[-1]:
                errors = {name:self.positions[name] - float(np.interp(relative, times,
                    [values[index] for values in tracking_points])) for index, name in enumerate(arm['joints'])}
                if not errors or not all(math.isfinite(value) and abs(value) <= PROFILE['button_tracking_rad'] for value in errors.values()):
                    self.record('speed_button_tracking_fault', dict(side=side,
                                errors_rad={name: value if math.isfinite(value) else str(value) for name, value in errors.items()},
                                limit_rad=PROFILE['button_tracking_rad'], joint_sim=self.joint_time))
                    raise ValueError('Button tracking exceeded relaxed simulation limit')
                self.record('fast_button_tracking_sample', dict(side=side, errors_rad=errors,
                            original_limit_rad=.015, current_limit_rad=PROFILE['button_tracking_rad'], joint_sim=self.joint_time))
            return (self.sim >= start + elapsed + .1 and
                    all(abs(self.positions[name] - value) < .005 for name, value in target.items()))

        self.wait(reached)
        return start

    def initialize_leg_posture(self):
        assert not self.holders.rows and self.holding is None and self.held_payload is None
        self.phase = 'machine_preparation_leg_posture'
        report = verify_initialization(EVIDENCE, self.plan['initial_positions'], self.positions)
        self.wait(lambda: self.leg_posture_state_wall > 0 and time.monotonic() - self.leg_posture_state_wall < 2)
        self.leg_posture_client = ActionClient(self.node, FollowJointTrajectory, '/leg_posture_controller/follow_joint_trajectory')
        self.wait(lambda: self.leg_posture_client.server_is_ready())
        assert self.node.count_publishers('/leg_posture_controller/joint_trajectory') == 0
        error = endpoint_error(self.plan['initial_positions'], self.positions)
        if error >= .005:
            raise RuntimeError('Actual original stationary leg target drifted')
        self.leg_posture_active = True
        self.record('leg_posture_pass', dict(
            joint_names=LEG_JOINTS, targets=[self.plan['initial_positions'][name] for name in LEG_JOINTS],
            measured=[self.positions[name] for name in LEG_JOINTS], controller='leg_posture_controller',
            no_extra_fixed_joint=True, prior_fresh_hold_sim_s=report['hold_sim_seconds'],
            current_error_rad=error, current_limit_rad=.005, redundant_confirmation_action_removed=True))

    def motion_limits(self, phase, requested):
        return motion_limits(phase, bool(self.holders.rows))

    def maximum_loaded_rate(self, phase):
        return PROFILE['loaded_rate_rad_s']

    def paired_endpoint_stamp(self, target, not_before):
        if self.joint_time < not_before or any(abs(self.positions[name] - value) >= PROFILE['endpoint_rad'] for name, value in target.items()):
            return None
        return self.joint_time

    def common_timing(self, actual, samples, rate):
        if self.phase == 'paired_retrieval_setdown':
            context = self.sequential_setdown_context
            holders = require_single_setdown_holder(self, context)
            if samples != context['coarse_samples']:
                raise ValueError('Qualified original setdown polyline required')
            self.record('sequential_single_setdown_timing', dict(
                hand=context['hand'], payload=context['payload'], source_snapshot_sha256=context['source_snapshot_sha256'],
                original_runtime_holder_references=holders, actual_rate_rad_s=rate, ramp_sim_s=.3,
                unchanged_samples=True, legacy_phase_retained_for_original_table_clearance=True,
                paired_profile_not_claimed=True, relaxed_simulation_profile=PROFILE['name']))
            return common_timing(actual, samples, rate, ramp_s=.3)
        if getattr(self, 'joined_stage_segments', None):
            return joined_timing(actual, self.joined_stage_segments, rate)
        return common_timing(actual, samples, rate)

    def dwell(self, seconds):
        return super().dwell(dwell_duration(self.phase, seconds))

    def terminal_hold(self, duration_sim_s=10):
        return super().terminal_hold(PROFILE['terminal_hold_sim_s'])

    def record(self, kind, payload):
        if kind == 'command' and getattr(self, 'joined_stage_segments', None):
            payload = dict(payload, joined_segments=copy.deepcopy(self.joined_stage_segments))
        if kind == 'terminal_table_hold_pass':
            payload = dict(payload, original_policy_metadata=payload.get('policy'),
                           policy=dict(profile=PROFILE['name'], duration_sim_s=PROFILE['terminal_hold_sim_s'],
                                       original_table_support_pose_checks_preserved=True))
        if kind in ('simulation_relaxation_declared', 'sequential_runtime_declared'):
            payload = copy.deepcopy(payload)
            payload.pop('unchanged_observations_sim_s', None)
            payload.update(speed_profile=copy.deepcopy(PROFILE), carry_angle_limit_deg=PROFILE['carry_angle_deg'],
                           original_strict_certificate_not_claimed=True)
        if kind == 'retrieval_setdown_braking':
            payload = dict(payload, coarse_rate_rad_s=PROFILE['near_rate_rad_s'],
                           fine_rate_rad_s=PROFILE['pilot_fine_rate_rad_s'], relaxed_simulation_profile=PROFILE['name'])
        return super().record(kind, payload)

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
            receipt = self.planning_worker.execute(source, output, self.spin)
            stream.write(json.dumps(dict(reused_planner_receipt=receipt)) + '\n')
        planned = json.loads(output.read_text())
        if planned.get('speed_profile') != PROFILE:
            raise RuntimeError('Planner must use the same explicit relaxed holder profile')
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
        if joint_drift > PROFILE['snapshot_joint_rad'] or pose_drift > PROFILE['snapshot_pose_m'] or angle_drift > PROFILE['snapshot_angle_deg'] or finger_drift > PROFILE['snapshot_finger_m']:
            raise RuntimeError('Actual sequential planning snapshot drifted past declared relaxed bounds')
        self.record('sequential_actual_collision_plan', planned)
        return planned

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
            elif self.sim - stable >= PROFILE['release_hold_sim_s']:
                self.record('drop_acceptance_pass', dict(side=payload, duration_sim_s=self.sim - stable,
                            corners_bucket_m=corners.tolist(), rotor_rpm=self.values['machine']['rotor_rpm']))
                return self.sim - stable
        raise RuntimeError('Single payload did not naturally settle completely inside its basket')

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
            while self.sim - began < PROFILE['release_hold_sim_s']:
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
                    future_start_and_completion_margin_sim_s=self.future_start_offset() + .05, minimum_wall_s=60,
                    maximum_wall_s=900, simulation_clock_must_advance_within_wall_s=2,
                    command_paths_preserved_rates_and_timestamps_changed=True))
        stable = None
        while self.sim < start + timing['duration_sim_s'] + .05:
            self.spin()
            try:
                wall_watch.observe(self.sim, time.monotonic())
            except (TimeoutError, ValueError) as error:
                self.record('paired_motion_clock_fault', dict(error=str(error), clock_sim=self.sim,
                            required_finish_sim=start + timing['duration_sim_s'] + .05,
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
                        raise RuntimeError('Speed-priority tracking exceeds declared relaxed limit')
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
                if stamp - stable > PROFILE['endpoint_hold_sim_s']:
                    self.record('interval_endpoint_pass', dict(
                        motion_start_sim=start, scheduled_end_sim=start + timing['duration_sim_s'],
                        actual_joint_stamp_sim=stamp, measured_stable_start_sim=stable,
                        measured_hold_sim_s=stamp - stable,
                        required_hold_sim_s=PROFILE['endpoint_hold_sim_s'],
                        original_endpoint_limit_rad=PROFILE['endpoint_rad'],
                        overlap_enabled=self.paired_feedback_overlap_enabled()))
                    return
            else:
                stable = None
        raise RuntimeError('Speed-priority endpoint failed declared relaxed precision gate')


