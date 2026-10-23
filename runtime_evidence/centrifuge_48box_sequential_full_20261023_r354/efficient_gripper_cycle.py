from gripper_damping_cycle import GripperDampingCycle
from zero_effort_policy import short_stroke_last_time


class EfficientGripperCycle(GripperDampingCycle):
    def qualified_short_stroke_last_time(self, original_last_time):
        return original_last_time

    def move(self, samples, phase, rate):
        if phase == 'paired_retrieval_setdown_fine' and rate != self.fine_setdown_rate():
            raise ValueError('Only the qualified declared fine-setdown rate may be dispatched')
        return super().move(samples, phase, rate)

    def gripper_trajectory_points(self, side, value, phase, effort):
        points = super().gripper_trajectory_points(side, value, phase, effort)
        last_time = short_stroke_last_time(self.phase, value, effort, self.positions,
                                          tuple(self.holders.rows), getattr(self, 'retrieval_closing', False))
        if last_time is None:
            return points
        last_time = self.qualified_short_stroke_last_time(last_time)
        if len(points) != 2 or points[0].time_from_start.sec != 0 or points[0].time_from_start.nanosec != 50000000:
            raise ValueError('Original measured two-point short-stroke trajectory required')
        if points[-1].time_from_start.sec != 1 or points[-1].time_from_start.nanosec != 0:
            raise ValueError('Unexpected parent gripper duration')
        points[-1].time_from_start.sec, points[-1].time_from_start.nanosec = divmod(round(last_time * 1e9), 1000000000)
        self.record('zero_final_effort_short_stroke_retimed', dict(side=side, phase=self.phase,
                    original_last_time_sim_s=1., actual_last_time_sim_s=last_time,
                    original_first_point_sim_s=.05, unchanged_positions_velocities_and_effort=True,
                    previous_feedforward_N=self.accepted_gripper_efforts.get(side, 0.),
                    final_feedforward_N=effort, both_actual_hands_checked=True,
                    no_hold_or_normal_release_gate_removed=True))
        return points

    def fine_setdown_rate(self):
        if not self.current_damping_empty_approach_completed or set(self.holders.rows) != {'left', 'right'}:
            raise ValueError('Fine-setdown retiming requires current qualification and two actual holders')
        if {row['payload'] for row in self.holders.rows.values()} != {'left', 'right'}:
            raise ValueError('Both distinct actual payloads are required')
        return .06
