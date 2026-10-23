from native_gripper_feedback import wait_original_gripper_endpoint
from scheduled_gripper_cycle import ScheduledGripperCycle


class NativeGripperCycle(ScheduledGripperCycle):
    def require_gripper_endpoint(self, hand, value, result_clock):
        if not self.current_damping_empty_approach_completed:
            return super().require_gripper_endpoint(hand, value, result_clock)
        return wait_original_gripper_endpoint(self, hand, value, result_clock)
