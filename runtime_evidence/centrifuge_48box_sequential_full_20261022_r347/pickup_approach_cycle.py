from retrieval_grip_cycle import RetrievalGripCycle


PICKUP_PHASE = 'paired_retrieval_approach'


class PickupApproachCycle(RetrievalGripCycle):
    def motion_limits(self, phase, requested_rate):
        inherited = super().motion_limits(phase, requested_rate)
        if phase != PICKUP_PHASE:
            return inherited
        if (requested_rate != .8 or inherited[0] != .4 or self.holders.rows
                or not self.current_damping_empty_approach_completed):
            raise ValueError('Only qualified original empty retrieval approach may be retimed')
        self.record('empty_retrieval_approach_retimed', dict(
            parent_rate_rad_s=.4, actual_rate_rad_s=.35,
            original_tracking_limit_rad=inherited[1], original_native_finger_velocity_limit_m_s=.2,
            original_points_seed_groups_dense_collision_samples_and_floor_preserved=True,
            no_new_grip_reference_or_force_change=True, actual_180_not_preclaimed=True))
        return .35, inherited[1]
