import math

from dual_motion import TASK_JOINTS


RETREAT_PHASES = ('empty_reverse_forward_inside_basket_raise',
                  'empty_reverse_forward_above_housing_outboard',
                  'empty_reverse_forward_outside_housing_table_transfer')
FORWARD_PHASES = ('forward_table_endpoint_descent',
                  'forward_outside_housing_table_transfer',
                  'forward_above_housing_outboard',
                  'forward_inside_basket_raise')


def expected_retreat_phases(guides=None):
    if guides is None:
        return RETREAT_PHASES
    if not isinstance(guides, list):
        raise ValueError('Original current-run executed loading guides required')
    phases = tuple(stage.get('phase') for stage in guides)
    if phases not in (FORWARD_PHASES, FORWARD_PHASES + (FORWARD_PHASES[-1],)):
        raise ValueError('Only the original route and at most its actual optional final alignment are reviewed')
    for stage in guides:
        samples = stage.get('samples')
        if not isinstance(samples, list) or not samples:
            raise ValueError('Original nonempty executed guide samples required')
        for sample in samples:
            if (not isinstance(sample, dict) or set(sample) != TASK_JOINTS
                    or not all(math.isfinite(value) for value in sample.values())):
                raise ValueError('Original complete finite current-run guide joints required')
    return tuple('empty_reverse_' + phase for phase in reversed(phases[1:]))
