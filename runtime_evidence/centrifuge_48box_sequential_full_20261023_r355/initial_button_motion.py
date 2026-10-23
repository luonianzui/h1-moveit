import math

from control_msgs.msg import JointTrajectoryControllerState
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectoryPoint

from button_ramp import ramp_button_positions


INITIAL_WITHDRAW_PHASE = 'machine_preparation_open_withdraw'


def initial_withdraw_points(points, rate):
    profile = ramp_button_positions([list(point.positions) for point in points],
                                    maximum_rate=min(rate, .6), ramp_s=.3)
    result = []
    for sample in profile['points']:
        point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'])
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
        result.append(point)
    return result, profile


def controller_sample(message):
    def point_data(point):
        return dict(positions=list(point.positions), velocities=list(point.velocities),
                    accelerations=list(point.accelerations), effort=list(point.effort),
                    time_from_start_ns=point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec)
    return dict(source_ns=message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec,
                joint_names=list(message.joint_names), reference=point_data(message.reference),
                feedback=point_data(message.feedback), error=point_data(message.error),
                output=point_data(message.output), speed_scaling_factor=message.speed_scaling_factor)


class InitialButtonMotionMixin:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.button_controller_samples = 0
        self.button_controller_subscriptions = [self.node.create_subscription(
            JointTrajectoryControllerState, '/' + side + '_arm_controller/controller_state',
            lambda message, side=side: self.observe_button_controller(side, message), qos_profile_sensor_data)
            for side in ('left', 'right')]

    def observe_button_controller(self, side, message):
        if not self.phase.startswith('machine_') or self.button_controller_samples >= 10000:
            return
        sample = controller_sample(message)
        sample['side'] = side
        sample['read_only_not_used_to_rebase_or_replace_guard'] = True
        self.button_controller_samples += 1
        self.record('button_controller_reference', sample)

    def button_points_before_publish(self, points, fixed_interval):
        if (self.phase != INITIAL_WITHDRAW_PHASE or fixed_interval is not None
                or self.current_damping_empty_approach_completed):
            return super().button_points_before_publish(points, fixed_interval)
        if self.holders.rows or self.holding is not None or self.held_payload is not None:
            raise ValueError('Initial withdrawal requires both hands actually empty')
        rate = self.button_noncontact_rate()
        if not math.isfinite(rate) or rate <= 0 or rate > .6:
            raise ValueError('Initial withdrawal requires the original bounded noncontact rate')
        result, profile = initial_withdraw_points(points, rate)
        self.record('initial_button_withdraw_smoothing', dict(
            maximum_rate_rad_s=profile['maximum_rate_rad_s'],
            maximum_acceleration_rad_s2=profile['maximum_acceleration_rad_s2'],
            ramp_time_sim_s=profile['ramp_time_sim_s'],
            original_vertices=profile['original_vertex_indices'],
            original_joint_polyline_preserved=True, all_original_vertices_zero_velocity=True,
            original_tracking_limit_rad=.15, original_terminal_limit_rad=.005,
            no_contact_timing_grip_or_spin_change=True, actual_physical_pass_not_preclaimed=True))
        return result
