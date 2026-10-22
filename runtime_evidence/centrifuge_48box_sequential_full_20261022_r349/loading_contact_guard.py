import math


def contact_violation(pair, side, phase, current_sim, drop_window_until=None):
    if not math.isfinite(pair['force']) or not math.isfinite(pair['depth']):
        return 'nonfinite_contact'
    names = pair['first'] + ' ' + pair['second']
    passive_drop = (drop_window_until is not None and current_sim <= drop_window_until and
                    phase in (side + '_release_above_bucket', side + '_drop_settling',
                              'paired_load_release_above_bucket', 'paired_load_drop_settling') and
                    'box48_' + side + '::' in names and 'bucket_' + side + '::' in names and
                    'dex1' not in names)
    depth_limit = .0015 if passive_drop else .001
    if pair['depth'] > depth_limit:
        return 'depth_exceeds_' + str(depth_limit)
    if 'dex1' in names and pair['force'] > 40:
        return 'finger_force_exceeds_40_N'
    return None

