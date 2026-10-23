import math


FREE_PHASES = frozenset(('paired_load_forward_outside_housing_table_transfer',
                        'paired_load_forward_above_housing_outboard',
                        'paired_retrieval_outboard', 'paired_retrieval_transfer'))


def transport_rate(phase, requested, holders):
    if phase not in FREE_PHASES:
        return requested
    if set(holders) != {'left', 'right'} or not math.isfinite(requested) or requested != .25:
        raise ValueError('Speed fork requires both actual holders and the original free phase')
    return .4


def require_stop_pre(positions, arm):
    if len(arm['joints']) != len(arm['pre']) or not arm['joints']:
        raise ValueError('Incomplete original stop-button pre pose')
    if any(name not in positions or not math.isfinite(positions[name]) or not math.isfinite(target) or
           abs(positions[name] - target) >= .005 for name, target in zip(arm['joints'], arm['pre'])):
        raise ValueError('Actual hand did not finish the original physical withdrawal')


def require_snapshot_drift(current, original):
    if not original or any(name not in current or not math.isfinite(value) or not math.isfinite(current[name])
                           for name, value in original.items()):
        raise ValueError('Incomplete actual handover snapshot')
    drift = max(abs(current[name] - value) for name, value in original.items())
    if not math.isfinite(drift) or drift > .0003:
        raise ValueError('Actual handover state changed during dense collision audit')
    return drift
