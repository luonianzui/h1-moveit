PROFILE = """
def single_setdown_profile(hand):
    if hand not in ('left', 'right'):
        raise ValueError('Known original single holder required')
    return (.06, 1.2) if hand == 'right' else (.13, .6)
"""

def undo_right_profile(source):
    changes=[
        (PROFILE,''),
        ("    if (not isinstance(context, dict) or cycle.phase != COARSE_PHASE or\n            rate != single_setdown_profile(context['hand'])[0] or",
         "    if (not isinstance(context, dict) or cycle.phase != COARSE_PHASE or rate != .13 or"),
        ("    expected_rate, ramp = single_setdown_profile(context['hand'])\n    timing = ramped_common_timing(actual, samples, expected_rate, ramp_s=ramp)",
         "    timing = ramped_common_timing(actual, samples, .13, ramp_s=.6)"),
        ("actual_rate_rad_s=expected_rate, ramp_sim_s=ramp, unchanged_samples=True,",
         "actual_rate_rad_s=.13, ramp_sim_s=.6, unchanged_samples=True,"),
        ("cycle.move(samples[:split+1], COARSE_PHASE, single_setdown_profile(context['hand'])[0])",
         "cycle.move(samples[:split+1], COARSE_PHASE, .13)")]
    for current,previous in changes:
        assert source.count(current)==1
        source=source.replace(current,previous,1)
    return source
