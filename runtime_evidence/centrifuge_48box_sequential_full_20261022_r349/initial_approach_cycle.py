from gentle_setdown_cycle import GentleSetdownCycle


APPROACH_PHASE = 'paired_load_approach'
PARENT_APPROACH_RATE = .8
APPROACH_RATE = .7


class InitialApproachCycle(GentleSetdownCycle):
    def motion_limits(self, phase, requested_rate):
        if phase != APPROACH_PHASE:
            return super().motion_limits(phase, requested_rate)
        if self.holders.rows or requested_rate != PARENT_APPROACH_RATE:
            raise ValueError('Only the original empty initial approach may be retimed')
        inherited = super().motion_limits(phase, requested_rate)
        if inherited[0] != PARENT_APPROACH_RATE:
            raise ValueError('Original initial approach dispatch changed')
        self.record('initial_empty_approach_retimed', dict(
                    parent_rate_rad_s=PARENT_APPROACH_RATE, actual_rate_rad_s=APPROACH_RATE,
                    unchanged_tracking_limit_rad=inherited[1],
                    original_native_finger_velocity_limit_m_s=.2,
                    original_points_actual_seed_groups_floor_and_physical_guards_preserved=True,
                    fresh_physical_completion_and_180_not_preclaimed=True))
        return APPROACH_RATE, inherited[1]
