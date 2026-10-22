from trajectory_msgs.msg import JointTrajectoryPoint

from button_ramp import ramp_button_positions
from compression_cycle import CompressionCycle


class RampedButtonCycle(CompressionCycle):
    def button_points_before_publish(self, points, fixed_interval):
        if fixed_interval is not None or not self.current_damping_empty_approach_completed:
            return points
        if self.holders.rows:
            raise ValueError('Ramped button travel requires empty actual hands')
        profile = ramp_button_positions([list(point.positions) for point in points], self.button_noncontact_rate())
        result = []
        for sample in profile['points']:
            point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
            result.append(point)
        self.record('ramped_button_noncontact_profile', dict(
                    maximum_rate_rad_s=profile['maximum_rate_rad_s'],
                    maximum_acceleration_rad_s2=profile['maximum_acceleration_rad_s2'],
                    ramp_time_sim_s=profile['ramp_time_sim_s'], original_vertices=profile['original_vertex_indices'],
                    original_joint_polyline_preserved=True, all_original_vertices_zero_velocity=True,
                    original_tracking_endpoint_contact_interval_and_physical_guards_preserved=True,
                    failed_unsmoothed_command_not_dispatched=True, actual_physical_pass_not_preclaimed=True))
        return result
