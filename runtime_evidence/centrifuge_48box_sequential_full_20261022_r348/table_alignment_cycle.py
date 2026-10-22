from right_setdown_cycle import SpeedPriorityCycle as ParentSpeedPriorityCycle
from table_alignment import align_supported_return
from empty_gap_policy import eligible
from prefetch_cycle import PlanningPrefetchMixin


class SpeedPriorityCycle(PlanningPrefetchMixin, ParentSpeedPriorityCycle):
    def future_start_offset(self):
        return .16 if eligible(self.phase, self.holders.rows) else .3

    def paired_feedback_overlap_enabled(self):
        return eligible(self.phase, self.holders.rows)

    def button_noncontact_rate(self):
        rate = super().button_noncontact_rate()
        return min(rate, .6) if self.phase.endswith('_withdraw') else rate

    def execute_stages(self, planned, prefix):
        result = super().execute_stages(planned, prefix)
        if (planned['mode'] == 'retrieval_transport' and
                planned['stages'][-1]['phase'] == 'retrieve_setdown'):
            self.pending_table_alignment = (planned['single_hand'], planned['single_payload'])
        return result

    def dwell(self, seconds):
        result = super().dwell(seconds)
        pending = getattr(self, 'pending_table_alignment', None)
        if pending is not None and seconds == .5 and self.phase == 'paired_retrieval_setdown_fine':
            self.pending_table_alignment = None
            align_supported_return(self, *pending)
        return result
