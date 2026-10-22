from complete_rate_profile import (
    BUTTON_NONCONTACT_RATE, EMPTY_CAPS, FINE_RATE, FREE_TRANSPORT_RATE, LOADED_RATES,
    profile_free_rate, profile_motion_limits, require_current_pair,
)
from efficient_gripper_cycle import EfficientGripperCycle
from handover_policy import FREE_PHASES


class CompressionCycle(EfficientGripperCycle):
    def run_complete(self):
        self.record('complete_rate_profile_declared', dict(
                    effective_profile_overrides_inherited_historical_rate_declarations=True,
                    loaded_phase_rates_rad_s={phase: rates[1] for phase, rates in LOADED_RATES.items()},
                    free_phases=sorted(FREE_PHASES), free_rate_rad_s=FREE_TRANSPORT_RATE,
                    empty_phase_caps_rad_s=EMPTY_CAPS, fine_setdown_rate_rad_s=FINE_RATE,
                    qualified_empty_button_noncontact_rate_rad_s=BUTTON_NONCONTACT_RATE,
                    original_contact_button_interval_unchanged=True,
                    original_complete_cycle_and_3_10_30_second_certificates_preserved=True,
                    actual_physics_and_180_second_completion_not_preclaimed=True))
        return super().run_complete()

    def motion_limits(self, phase, requested_rate):
        original = super().motion_limits(phase, requested_rate)
        selected = profile_motion_limits(self, phase, requested_rate, original)
        if selected != original:
            self.record('complete_profile_motion_retimed', dict(phase=phase,
                        previous_effective_rate_rad_s=original[0], actual_rate_rad_s=selected[0],
                        unchanged_tracking_limit_rad=original[1], original_points_and_geometry=True,
                        original_common_clock_floor_endpoint_native_force_and_grip_guards=True,
                        new_physical_speed_not_preclaimed=True))
        return selected

    def maximum_loaded_rate(self, phase):
        if phase in FREE_PHASES:
            require_current_pair(self)
            return FREE_TRANSPORT_RATE
        if phase in LOADED_RATES:
            require_current_pair(self)
            return LOADED_RATES[phase][1]
        return super().maximum_loaded_rate(phase)

    def free_transport_rate(self, phase, requested):
        return profile_free_rate(self, phase, requested)

    def coarse_setdown_rate(self):
        require_current_pair(self)
        return LOADED_RATES['paired_retrieval_setdown'][1]

    def fine_setdown_rate(self):
        require_current_pair(self)
        return FINE_RATE

    def button_noncontact_rate(self):
        if self.holders.rows:
            raise ValueError('Loaded physical fingers cannot use noncontact button retiming')
        if not self.current_damping_empty_approach_completed:
            return super().button_noncontact_rate()
        return BUTTON_NONCONTACT_RATE
