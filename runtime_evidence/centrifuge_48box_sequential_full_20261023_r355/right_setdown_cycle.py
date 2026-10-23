from sequential_cycle import SequentialCycle
from sequential_speed_cycle import SpeedPriorityCycle as ParentSpeedPriorityCycle
from single_setdown import COARSE_PHASE, require_single_setdown_holder
from speed_policy import PROFILE, common_timing


RIGHT_COARSE_RATE = .2
RIGHT_COARSE_RAMP = .6


class SpeedPriorityCycle(ParentSpeedPriorityCycle):
    def right_coarse_setdown(self, phase):
        if phase != COARSE_PHASE:
            return False
        context = self.sequential_setdown_context
        require_single_setdown_holder(self, context)
        return context['hand'] == 'right'

    def motion_limits(self, phase, requested):
        rate, tracking = super().motion_limits(phase, requested)
        if phase == 'seq_retrieval_single_pilot':
            holders = self.holders.snapshot()
            if (len(holders) != 1 or next(iter(holders)) not in ('left', 'right') or self.order.phase != 'retrieval'
                    or any(hand != row['payload'] for hand, row in holders.items())):
                raise ValueError('Single original retrieval holder required for gentle pilot')
            rate = .05
        if self.right_coarse_setdown(phase):
            rate = RIGHT_COARSE_RATE
        return rate, tracking

    def common_timing(self, actual, samples, rate):
        if not self.right_coarse_setdown(self.phase):
            return super().common_timing(actual, samples, rate)
        context = self.sequential_setdown_context
        if rate != RIGHT_COARSE_RATE or samples != context['coarse_samples']:
            raise ValueError('Exact right coarse setdown rate and original path required')
        holders = require_single_setdown_holder(self, context)
        self.record('sequential_single_setdown_timing', dict(
            hand=context['hand'], payload=context['payload'],
            source_snapshot_sha256=context['source_snapshot_sha256'],
            original_runtime_holder_references=holders, actual_rate_rad_s=rate,
            ramp_sim_s=RIGHT_COARSE_RAMP, unchanged_samples=True,
            legacy_phase_retained_for_original_table_clearance=True,
            paired_profile_not_claimed=True, relaxed_simulation_profile=PROFILE['name'],
            right_setdown_stability_only=True))
        return common_timing(actual, samples, rate, ramp_s=RIGHT_COARSE_RAMP)

    def record(self, kind, payload):
        if kind == 'retrieval_setdown_braking' and set(self.holders.rows) == {'right'}:
            payload = dict(payload, coarse_rate_rad_s=RIGHT_COARSE_RATE,
                           fine_rate_rad_s=PROFILE['pilot_fine_rate_rad_s'],
                           coarse_ramp_sim_s=RIGHT_COARSE_RAMP,
                           right_setdown_stability_only=True,
                           relaxed_simulation_profile=PROFILE['name'])
            return SequentialCycle.record(self, kind, payload)
        return super().record(kind, payload)
