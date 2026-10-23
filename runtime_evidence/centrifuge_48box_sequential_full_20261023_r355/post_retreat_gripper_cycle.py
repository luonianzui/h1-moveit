from pickup_approach_cycle import PickupApproachCycle
from post_retreat_contract import RETREAT_PHASES, expected_retreat_phases


class PostRetreatGripperCycle(PickupApproachCycle):
    def return_neutral(self):
        if self.holders.rows or not self.current_damping_empty_approach_completed:
            raise ValueError('Qualified empty hands required before post-retreat gripper action')
        planned = self.actual_plan('empty_return', dict(left='left', right='right'))
        stages = planned['stages']
        expected = expected_retreat_phases(getattr(self, 'qualified_empty_retreat_guides', None))
        if tuple(stage['phase'] for stage in stages) != expected + ('empty_return',):
            raise ValueError('Exactly the three original actual reverse retreats are required')
        self.completed_empty_retreat_stages = []
        for index, stage in enumerate(stages[:-1]):
            self.move(stage['samples'], 'machine_paired_' + stage['phase'], .13 if stage['phase'] == RETREAT_PHASES[0] else .8)
            self.completed_empty_retreat_stages.append(stage['phase'])
        self.actual_plan('empty_gripper_after_retreat', dict(left='left', right='right'))
        self.record('post_retreat_gripper_action_declared', dict(
            completed_original_retreats=list(self.completed_empty_retreat_stages),
            original_target_m=-.019, original_feedforward_N=0.,
            original_native_finger_velocity_limit_m_s=.2, original_software_output_guard_N=18.01,
            actual_full_state_gripper_FCL_checked_before_action=True,
            actual_remaining_path_replanned_after_original_fresh_endpoint=True,
            no_basket_gripper_reorder_no_holder_release_or_reference_rebase=True))
        self.paired_grippers(-.019, 0, 'machine_paired_empty_grippers')
        remainder = self.actual_plan('empty_return_after_gripper', dict(left='left', right='right'))
        if len(remainder['stages']) != 1 or remainder['stages'][0]['phase'] != 'empty_return':
            raise ValueError('Only the fresh final empty return may follow the gripper action')
        stage = remainder['stages'][0]
        self.move(stage['samples'], 'machine_paired_empty_return', .8)
