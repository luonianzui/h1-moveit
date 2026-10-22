import math
import re


def predicted_ramp_error(rate_rad_s, update_hz, proportional_gain):
    if not all(math.isfinite(value) and value > 0 for value in (rate_rad_s, update_hz, proportional_gain)):
        raise ValueError('Finite positive parameters required')
    if not 0 < proportional_gain < 2:
        raise ValueError('Discrete position gain outside stable model bounds')
    return rate_rad_s / (update_hz * proportional_gain)


def verify_rate_fork(original, candidate):
    import copy
    restored = copy.deepcopy(candidate)
    assert original['controller_manager']['ros__parameters']['update_rate'] == 100
    assert restored['controller_manager']['ros__parameters']['update_rate'] == 500
    restored['controller_manager']['ros__parameters']['update_rate'] = 100
    if restored != original:
        raise ValueError('Unreviewed controller change beyond manager update rate')
    return True
def verify_manager_rate_output(output, returncode):
    if returncode != 0 or re.fullmatch(r'\s*Integer value is: 500\s*', output) is None:
        raise ValueError('Fresh manager did not confirm the exact 500Hz parameter')
    return 500


