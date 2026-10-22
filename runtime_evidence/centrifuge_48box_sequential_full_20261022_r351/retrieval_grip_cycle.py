from execute_fast import RETRIEVAL_SOFT_GRASP_SECONDS
from execute_loading import GROUP_JOINTS
from initial_approach_cycle import InitialApproachCycle
from soft_grasp import soft_grasp_profile
from trajectory_msgs.msg import JointTrajectoryPoint


RETRIEVAL_PHASE = 'paired_retrieval_grasp'
PARENT_EFFORT_N = 16.
RETRIEVAL_EFFORT_N = 18.


class RetrievalGripCycle(InitialApproachCycle):
    def paired_grippers(self, value, effort, phase, retrieval=False):
        if phase != RETRIEVAL_PHASE:
            return super().paired_grippers(value, effort, phase, retrieval)
        if (retrieval is not True or value != -.01901 or effort != PARENT_EFFORT_N
                or not self.current_damping_empty_approach_completed or self.holders.rows
                or any(self.accepted_gripper_efforts.get(hand) != 0 for hand in ('left', 'right'))):
            raise ValueError('Only qualified empty paired retrieval grasp may use bounded eighteen N')
        self.record('retrieval_grip_feedforward_declared', dict(
            parent_effort_N=PARENT_EFFORT_N, actual_effort_N=RETRIEVAL_EFFORT_N,
            unchanged_software_output_guard_N=18.01,
            original_output_clamp_N=32., original_contact_force_limit_N=40.,
            original_native_slip_m=.002, original_native_angle_deg=5.,
            reference_captured_after_actual_fresh_grasp_not_rebased=True,
            duration_waypoints_endpoint_and_all_hold_certificates_preserved=True))
        return super().paired_grippers(value, RETRIEVAL_EFFORT_N, phase, retrieval)

    def gripper_trajectory_points(self, side, value, phase, effort):
        if self.phase != RETRIEVAL_PHASE:
            return super().gripper_trajectory_points(side, value, phase, effort)
        if (side not in ('left', 'right') or phase != side + '_paired_empty'
                or self.retrieval_closing is not True or effort != RETRIEVAL_EFFORT_N
                or value != -.01901 or not self.current_damping_empty_approach_completed
                or self.holders.rows or self.accepted_gripper_efforts.get(side) != 0):
            raise ValueError('Exactly the qualified original paired retrieval profile is required')
        profile = soft_grasp_profile([self.positions[name] for name in GROUP_JOINTS[side + '_gripper']],
                                    value, 0., PARENT_EFFORT_N,
                                    duration=RETRIEVAL_SOFT_GRASP_SECONDS, intervals=60)
        points = []
        for sample in profile:
            sample['effort'] = [entry * RETRIEVAL_EFFORT_N / PARENT_EFFORT_N for entry in sample['effort']]
            point = JointTrajectoryPoint(positions=sample['positions'], velocities=sample['velocities'],
                                         effort=sample['effort'])
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(sample['time_s'] * 1e9), 1000000000)
            points.append(point)
        self.record('retrieval_soft_grasp_profile', dict(
            side=side, profile=profile, duration_sim_s=RETRIEVAL_SOFT_GRASP_SECONDS,
            parent_effort_N=PARENT_EFFORT_N, final_effort_N=RETRIEVAL_EFFORT_N,
            original_output_clamp_N=32., original_contact_force_limit_N=40.,
            original_velocity_limit_m_s=.2, original_positions_velocities_and_times_preserved=True))
        return points
