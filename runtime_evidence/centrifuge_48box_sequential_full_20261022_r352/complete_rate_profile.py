import math

from handover_policy import FREE_PHASES, transport_rate


EMPTY_CAPS = {
    'paired_load_descend': .26,
    'paired_retrieval_descend': .26,
    'machine_paired_empty_reverse_forward_inside_basket_raise': .26,
    'paired_retrieval_approach': .4,
    'machine_paired_empty_return': .6,
}
LOADED_RATES = {
    'paired_load_pilot_lift': (.03, .08),
    'paired_retrieval_pilot_lift': (.03, .08),
    'paired_load_forward_table_endpoint_descent': (.25, .45),
    'paired_load_forward_inside_basket_raise': (.08, .14),
    'paired_retrieval_lift': (.25, .4),
    'paired_retrieval_setdown': (.20, .35),
}
FREE_TRANSPORT_RATE = .6
FINE_RATE = .09
BUTTON_NONCONTACT_RATE = 1.2


def require_current_pair(cycle):
    if not cycle.current_damping_empty_approach_completed:
        raise ValueError('Current original-native qualification is required before profile retiming')
    if set(cycle.holders.rows) != {'left', 'right'}:
        raise ValueError('Two actual holders are required by the loaded profile')
    if {row['payload'] for row in cycle.holders.rows.values()} != {'left', 'right'}:
        raise ValueError('Two distinct actual payloads are required')


def profile_motion_limits(cycle, phase, requested, original_limits):
    if not math.isfinite(requested) or requested <= 0:
        raise ValueError('Finite positive requested trajectory rate required')
    if phase in EMPTY_CAPS:
        if cycle.holders.rows or not cycle.current_damping_empty_approach_completed:
            raise ValueError('Qualified empty actual hands required for empty retiming')
        return min(requested, EMPTY_CAPS[phase]), original_limits[1]
    if phase in LOADED_RATES:
        require_current_pair(cycle)
        previous, candidate = LOADED_RATES[phase]
        if requested not in (previous, candidate):
            raise ValueError('Only the declared original or candidate loaded rate is accepted')
        return candidate, original_limits[1]
    return original_limits


def profile_free_rate(cycle, phase, requested):
    previous = transport_rate(phase, requested, cycle.holders.rows)
    if phase not in FREE_PHASES:
        return previous
    require_current_pair(cycle)
    return FREE_TRANSPORT_RATE


DUAL_LIMIT_METHOD = '''    def motion_limits(self, phase, requested_rate):
        return bounded_motion(phase, requested_rate)

'''
DUAL_COARSE_METHOD = '''    def coarse_setdown_rate(self):
        return .20

'''
STOP_FREE_METHOD = '''    def free_transport_rate(self, phase, requested):
        return transport_rate(phase, requested, self.holders.rows)

'''
BUTTON_RATE_METHOD = '''    def button_noncontact_rate(self):
        return .8

'''


def restore_profile_source(name, source):
    if name == 'execute_dual.py':
        if source.count(DUAL_LIMIT_METHOD) != 1 or source.count(DUAL_COARSE_METHOD) != 1:
            raise ValueError('Only the declared original-default dual hooks may be reversed')
        source = source.replace(DUAL_LIMIT_METHOD, '', 1).replace(DUAL_COARSE_METHOD, '', 1)
        source = source.replace('self.motion_limits(phase, rate)', 'bounded_motion(phase, rate)')
        source = source.replace('self.coarse_setdown_rate()', '.20')
    elif name == 'stop_handover_cycle.py':
        if source.count(STOP_FREE_METHOD) != 1:
            raise ValueError('Only the declared original-default free-rate hook may be reversed')
        source = source.replace(STOP_FREE_METHOD, '', 1)
        source = source.replace('self.free_transport_rate(phase, rate)', 'transport_rate(phase, rate, self.holders.rows)')
    elif name == 'execute_fast.py':
        if source.count(BUTTON_RATE_METHOD) != 1:
            raise ValueError('Only the declared original-default noncontact button hook may be reversed')
        source = source.replace(BUTTON_RATE_METHOD, '', 1)
        source = source.replace('        travel_rate = self.button_noncontact_rate() if fixed_interval is None else .8\n', '', 1)
        source = source.replace(' / travel_rate)', ' / .8)')
        source = source.replace('actual_travel_rate_rad_s=travel_rate,', 'actual_travel_rate_rad_s=.8,')
    return source
