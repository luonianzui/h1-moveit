from complete_rate_profile import require_current_pair
from paired_ramp import ramped_common_timing
from post_retreat_gripper_cycle import PostRetreatGripperCycle


class CoarseRampCycle(PostRetreatGripperCycle):
    def common_timing(self, actual, samples, rate):
        if self.phase != 'paired_retrieval_setdown':
            return super().common_timing(actual, samples, rate)
        require_current_pair(self)
        if rate != .30:
            raise ValueError('Only the original loaded point-three coarse setdown may receive this timing ramp')
        return ramped_common_timing(actual, samples, rate, ramp_s=.6)
