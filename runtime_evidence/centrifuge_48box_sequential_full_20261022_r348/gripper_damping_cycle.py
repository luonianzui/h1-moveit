from handover_policy import FREE_PHASES
from stop_handover_cycle import StopHandoverCycle


class GripperDampingCycle(StopHandoverCycle):
    def __init__(self, plan, buttons):
        self.current_damping_empty_approach_completed = False
        super().__init__(plan, buttons)

    def move(self, samples, phase, rate):
        if phase in FREE_PHASES and not self.current_damping_empty_approach_completed:
            raise ValueError('Current damping empty approach must finish under original native guards first')
        result = super().move(samples, phase, rate)
        if phase == 'paired_load_approach':
            self.current_damping_empty_approach_completed = True
            self.record('current_damping_initial_empty_approach_pass', dict(gripper_pid_d=6.,
                        original_velocity_limit_m_s=.2, original_tracking_and_endpoint_guards_passed=True,
                        only_current_initial_path_completed_not_general_calibration=True,
                        loaded_motion_still_not_preclaimed=True))
        return result
