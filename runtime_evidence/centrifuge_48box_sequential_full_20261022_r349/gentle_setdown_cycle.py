from complete_rate_profile import require_current_pair
from native_gripper_cycle import NativeGripperCycle


COARSE_PHASE = 'paired_retrieval_setdown'
PARENT_COARSE_RATE = .35
COARSE_RATE = .30


class GentleSetdownCycle(NativeGripperCycle):
    def coarse_setdown_rate(self):
        require_current_pair(self)
        return COARSE_RATE

    def maximum_loaded_rate(self, phase):
        if phase == COARSE_PHASE:
            return self.coarse_setdown_rate()
        return super().maximum_loaded_rate(phase)

    def motion_limits(self, phase, requested_rate):
        if phase != COARSE_PHASE:
            return super().motion_limits(phase, requested_rate)
        rate = self.coarse_setdown_rate()
        if requested_rate != rate:
            raise ValueError('Only the declared gentle coarse setdown rate is accepted')
        inherited = super().motion_limits(phase, PARENT_COARSE_RATE)
        if inherited[0] != PARENT_COARSE_RATE:
            raise ValueError('Original coarse setdown dispatch contract changed')
        self.record('gentle_coarse_setdown_selected', dict(
                    parent_rate_rad_s=PARENT_COARSE_RATE, actual_rate_rad_s=rate,
                    unchanged_tracking_limit_rad=inherited[1],
                    original_path_points_actual_seed_common_clock_and_twenty_ms_floor=True,
                    original_native_grip_force_endpoint_and_3_10_30_guards_preserved=True,
                    fresh_physical_setdown_and_complete_180_not_preclaimed=True))
        return rate, inherited[1]
