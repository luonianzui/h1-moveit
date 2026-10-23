from coarse_ramp_cycle import CoarseRampCycle
from native_gripper_feedback import wait_original_gripper_endpoint


EMPTY_IDLE_PHASES = frozenset(('paired_load_open_empty', 'machine_paired_empty_grippers',
                              'paired_retrieval_open_empty'))


class EmptyProtocolCycle(CoarseRampCycle):
    def prepare_loading_grippers(self):
        if self.holders.rows or any(self.accepted_gripper_efforts.get(hand, 0.) != 0 for hand in ('left', 'right')):
            raise ValueError('Initial empty preparation requires no holder and zero accepted effort')
        self.record('empty_startup_protocol_declared', dict(
            omitted_zero_effort_phases=['paired_load_close_empty_check', 'paired_load_reopen_empty'],
            original_open_target_m=-.0199, original_precision_m=.00025,
            fresh_native_stamp_after_actual_shared_result_required=True,
            original_action_result_safety_and_all_loaded_certificates_preserved=True))
        self.paired_grippers(-.0199, 0, 'paired_load_open_empty')

    def require_gripper_endpoint(self, hand, value, result_clock):
        if self.phase not in EMPTY_IDLE_PHASES:
            return super().require_gripper_endpoint(hand, value, result_clock)
        if (value != -.0199 and not (self.phase == 'machine_paired_empty_grippers' and value == -.019)
                or self.holders.rows or self.accepted_gripper_efforts.get(hand) != 0):
            raise ValueError('Only original zero-effort empty endpoints may use shorter idle')
        return wait_original_gripper_endpoint(self, hand, value, result_clock)

    def paired_gripper_post_action_dwell(self):
        if self.phase not in EMPTY_IDLE_PHASES:
            return super().paired_gripper_post_action_dwell()
        if self.holders.rows or any(self.accepted_gripper_efforts.get(hand) != 0 for hand in ('left', 'right')):
            raise ValueError('No loaded or positive-effort post-action idle may be shortened')
        self.record('empty_post_action_idle_declared', dict(phase=self.phase,
            parent_idle_sim_s=.3, actual_idle_sim_s=.1,
            original_success_result_and_bilateral_fresh_endpoint_completed=True,
            original_three_ten_thirty_sim_certificates_unchanged=True))
        return .1

    def motion_limits(self, phase, requested_rate):
        inherited = super().motion_limits(phase, requested_rate)
        if phase != 'paired_retrieval_approach':
            return inherited
        if (requested_rate != .8 or inherited != (.35, .002) or self.holders.rows
                or not self.current_damping_empty_approach_completed):
            raise ValueError('Only qualified empty retrieval approach may restore the old point-four cap')
        self.record('empty_retrieval_original_rate_restored', dict(parent_rate_rad_s=.35,
            actual_rate_rad_s=.4, original_tracking_limit_rad=.002,
            original_native_finger_velocity_limit_m_s=.2,
            original_geometry_floor_fcl_and_fault_guards_preserved=True,
            requires_new_full_physical_validation=True))
        return .4, .002
