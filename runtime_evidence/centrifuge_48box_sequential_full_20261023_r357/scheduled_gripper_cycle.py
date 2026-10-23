from ramped_button_cycle import RampedButtonCycle


class ScheduledGripperCycle(RampedButtonCycle):
    def future_start_offset(self):
        return .16 if self.current_damping_empty_approach_completed else super().future_start_offset()

    def qualified_short_stroke_last_time(self, original_last_time):
        if original_last_time != .5:
            raise ValueError('Only original qualified half-second short strokes can be retimed')
        if not self.current_damping_empty_approach_completed:
            return super().qualified_short_stroke_last_time(original_last_time)
        self.record('qualified_zero_effort_short_stroke_declared', dict(
            original_last_time_sim_s=original_last_time, current_last_time_sim_s=.25,
            original_all_four_actual_fingers_one_mm_no_holder_zero_final_force_policy_preserved=True,
            original_first_point_positions_velocities_efforts_endpoint_and_native_guards_preserved=True,
            actual_current_physical_pass_not_preclaimed=True))
        return .25
