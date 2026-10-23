import hashlib
import json
from pathlib import Path

from speed_policy import common_timing
from test_right_setdown_stability import fixture


ROOT = Path(__file__).resolve().parent
records = []
for hand in ('left', 'right'):
    cycle, samples, source = fixture(hand)
    previous = common_timing(samples[0], samples, .4, ramp_s=.3)
    rate, tracking = cycle.motion_limits(cycle.phase, .06)
    candidate = cycle.common_timing(samples[0], samples, rate)
    assert candidate['points'] == previous['points']
    assert candidate['groups'] == previous['groups'] and tracking == .04
    assert cycle.holders.snapshot() == cycle.sequential_setdown_context['original_holders']
    if hand == 'left':
        assert candidate == previous
    records.append(dict(hand=hand, actual_failed_run_plan=str(source),
        plan_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), samples=len(samples),
        original_duration_sim_s=previous['duration_sim_s'], candidate_duration_sim_s=candidate['duration_sim_s'],
        rate_rad_s=rate, ramp_sim_s=candidate['acceleration_ramp_sim_s'],
        points_groups_and_holder_unchanged=True))
result = dict(status='PASS_ACTUAL_R338_PATH_RETIMING_ONLY_NOT_PHYSICAL',
              records=records, mock='holder, recorder and synthetic start at actual qualified first point',
              all_other_phases_original_parent=True, carry_slip_m=.005, carry_angle_deg=10.,
              no_complete_success_or_runtime_speedup_claim=True)
with (ROOT / 'right_setdown_stability_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result))
