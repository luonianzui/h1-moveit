import math

from coarse_ramp_cycle import CoarseRampCycle
from complete_rate_profile import require_current_pair
from empty_protocol_cycle import EmptyProtocolCycle
from elbow_empty_approach import production_empty_approach_limits
from elbow_transport_rate import RETRIEVAL_TRANSPORT_PHASES, production_retrieval_transport_rate


class EndpointOverlapCycle(EmptyProtocolCycle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.elbow_empty_approach_cap = .5
        self.elbow_retrieval_transport_cap = .3

    def free_transport_rate(self, phase, requested):
        inherited = super().free_transport_rate(phase, requested)
        if phase in RETRIEVAL_TRANSPORT_PHASES and getattr(self, 'elbow_retrieval_transport_cap', None) is not None:
            return production_retrieval_transport_rate(self, phase, inherited)
        return inherited

    def paired_feedback_overlap_enabled(self):
        if not self.current_damping_empty_approach_completed:
            return False
        if self.holders.rows:
            require_current_pair(self)
        return True

    def paired_endpoint_stamp(self, target, not_before):
        if not self.current_damping_empty_approach_completed:
            return super().paired_endpoint_stamp(target, not_before)
        stamp = self.joint_time
        if not math.isfinite(stamp) or stamp < not_before:
            return None
        if any(not math.isfinite(self.positions[name]) or abs(self.positions[name] - value) >= .0003
               for name, value in target.items()):
            return None
        return stamp

    def motion_limits(self, phase, requested_rate):
        if phase == 'paired_load_approach' and getattr(self, 'elbow_empty_approach_cap', None) is not None:
            return production_empty_approach_limits(self, requested_rate, super().motion_limits)
        if phase != 'paired_retrieval_approach':
            return super().motion_limits(phase, requested_rate)
        inherited = CoarseRampCycle.motion_limits(self, phase, requested_rate)
        self.record('empty_retrieval_qualified_parent_cap_preserved', dict(
            candidate_not_executed_cap_rad_s=.4, actual_rate_rad_s=inherited[0],
            original_tracking_limit_rad=inherited[1], original_finger_velocity_limit_m_s=.2,
            only_endpoint_observation_overlap_not_higher_approach_speed=True))
        return inherited
