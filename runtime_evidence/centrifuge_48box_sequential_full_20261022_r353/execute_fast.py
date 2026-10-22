import numpy as np

from execute_cycle import FullCycle
from execute_loading import GROUP_JOINTS, LoadingCycle
from fast_profile import BUTTON_CONTACT_INTERVAL, RETRIEVAL_SOFT_GRASP_SECONDS
from fast_profile import calibrated_rate, validate_tracking
from setdown_braking import braking_setdown_segments
from soft_grasp import soft_grasp_profile
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FastCycle(FullCycle):
    def record(self, kind, payload):
        if kind in ('speed_profile_declared', 'free_carry_profile_declared'):
            payload = dict(payload, inherited_reference_only=True, actual_runtime_profile=False)
            kind = 'inherited_reference_' + kind
        super().record(kind, payload)

    def run_complete(self):
        self.record('servo_fast_profile_declared', dict(manager_update_rate_hz=500,
            maximum_empty_rad_s=.8, maximum_loaded_free_rad_s=.25,
            failed_half_rad_s_loaded_candidate_withdrawn=True,
            right_loading_lower_rad_s=.065, left_loading_lower_rad_s=.13,
            loading_alignment_rad_s=.10, failed_quarter_alignment_candidate_withdrawn=True,
            retrieval_setdown_coarse_rad_s=.20,
            original_waypoint_floor_sim_s=.020, unchanged_tracking_rad=.015,
            unchanged_slip_m=.002, unchanged_carry_angle_deg=5,
            button_contact_interval_sim_s=BUTTON_CONTACT_INTERVAL,
            retrieval_soft_grasp_sim_s=RETRIEVAL_SOFT_GRASP_SECONDS,
            original_3_10_30_second_certificates_unchanged=True,
            previous_empty_calibration_peak_rad=.006399999290603753,
            loaded_motion_not_previously_certified=True, under_180_not_preclaimed=True))
        super().run_complete()

    def move(self, samples, phase, rate):
        effective = calibrated_rate(phase, self.holding, self.held_payload)
        self.record('servo_motion_retime', dict(phase=phase, original_requested_rate_rad_s=rate,
            actual_rate_rad_s=effective, unchanged_geometric_samples=True,
            holding=self.holding, held_payload=self.held_payload, unchanged_floor_sim_s=.020))
        if phase in ('left_retrieval_setdown', 'right_retrieval_setdown'):
            coarse, fine, split = braking_setdown_segments(samples)
            self.record('retrieval_setdown_braking', dict(phase=phase, split_index=split,
                original_points=len(samples), unchanged_path=True, final_nominal_clearance_m=.010,
                coarse_rate_rad_s=effective, fine_rate_rad_s=.03, original_endpoint_rad=.0003))
            LoadingCycle.move(self, coarse, phase, effective)
            return LoadingCycle.move(self, fine, phase, .03)
        return LoadingCycle.move(self, samples, phase, effective)

    def gripper_trajectory_points(self, side, value, phase, effort):
        if not getattr(self, 'retrieval_closing', False):
            return super().gripper_trajectory_points(side, value, phase, effort)
        assert phase == side + '_grasp' and effort == 16
        profile = soft_grasp_profile([self.positions[name] for name in GROUP_JOINTS[side + '_gripper']],
            value, self.accepted_gripper_efforts.get(side, 0), effort,
            duration=RETRIEVAL_SOFT_GRASP_SECONDS, intervals=60)
        points = []
        for sample in profile:
            point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'],
                                         effort=sample['effort'])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
            points.append(point)
        self.record('retrieval_soft_grasp_profile', dict(side=side, profile=profile,
            duration_sim_s=RETRIEVAL_SOFT_GRASP_SECONDS, previous_duration_sim_s=6.,
            final_effort_N=16, unchanged_effort_limit_N=18, unchanged_velocity_limit_m_s=.2))
        return points

    def button_noncontact_rate(self):
        return .8

    def future_start_offset(self):
        return .3

    def button_points_before_publish(self, points, fixed_interval):
        return points

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
                elapsed += BUTTON_CONTACT_INTERVAL if fixed_interval is not None else max(
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
            actual_travel_rate_rad_s=travel_rate, actual_fixed_contact_interval_sim_s=BUTTON_CONTACT_INTERVAL,
            unchanged_geometry=True, points=[dict(positions=list(point.positions), velocities=list(point.velocities), seconds=seconds)
                for point, seconds in zip(message.points, times)]))
        target = dict(zip(arm['joints'], points[-1]))

        def reached():
            relative = self.joint_time - start
            if times[0] <= relative <= times[-1]:
                errors = {name:self.positions[name] - float(np.interp(relative, times,
                    [values[index] for values in tracking_points])) for index, name in enumerate(arm['joints'])}
                validate_tracking(errors)
                self.record('fast_button_tracking_sample', dict(side=side, errors_rad=errors,
                            original_limit_rad=.015, joint_sim=self.joint_time))
            return (self.sim >= start + elapsed + .1 and
                    all(abs(self.positions[name] - value) < .005 for name, value in target.items()))

        self.wait(reached)
        return start
