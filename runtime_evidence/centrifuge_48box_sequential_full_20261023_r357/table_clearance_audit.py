import itertools
import math

import numpy as np

from table_bucket_clearance import TABLE_SETDOWN_PHASES


def recompute_table_clearance(poses, payload, pairs, bounds):
    from execute_loading import pose_matrix

    body = pose_matrix(poses['box48_' + payload])
    machine = pose_matrix(poses['centrifuge'])
    corners = np.array(list(itertools.product((-1., 1.), repeat=3)))
    corners *= np.array([.1187450051085, .08814010043, .0595]) / 2
    distances = {}
    for bucket in ('left', 'right'):
        relative = np.linalg.inv(machine @ pose_matrix(poses['bucket_' + bucket])) @ body
        local = corners @ relative[:3, :3].T + relative[:3, 3]
        if not np.isfinite(local).all():
            return dict(valid=False, clearance_m=0., per_bucket_clearance_m={})
        gaps = []
        for axis in (0, 1):
            lower, upper = min(local[:, axis]), max(local[:, axis])
            basket_lower, basket_upper = bounds[bucket]['minimum'][axis], bounds[bucket]['maximum'][axis]
            if lower > basket_upper:
                gaps.append(float(lower - basket_upper))
            elif basket_lower > upper:
                gaps.append(float(basket_lower - upper))
            else:
                gaps.append(0.)
        distances[bucket] = math.hypot(*gaps)
    no_contact = not any('centrifuge::bucket_' in pair['first'] + pair['second'] for pair in pairs)
    minimum = min(distances.values())
    return dict(valid=minimum >= .003 and no_contact, clearance_m=minimum,
                per_bucket_clearance_m=distances, no_basket_contact=no_contact)


def verify_table_record(phase, data, bounds):
    if phase not in TABLE_SETDOWN_PHASES or data.get('payload') not in ('left', 'right'):
        return dict(valid=False, clearance_m=0.)
    actual = recompute_table_clearance(data['poses'], data['payload'], data['contacts'], bounds)
    reported = data['clearance']
    truth = (actual['valid'] and reported.get('valid') is True and reported.get('finite') is True and
             reported.get('required_margin_m') == .003 and
             reported.get('no_basket_contact') == actual['no_basket_contact'] and
             abs(actual['clearance_m'] - reported['clearance_m']) < 1e-10 and
             set(reported['per_bucket_clearance_m']) == {'left', 'right'} and
             all(abs(actual['per_bucket_clearance_m'][bucket] - reported['per_bucket_clearance_m'][bucket]) < 1e-10
                 for bucket in ('left', 'right')))
    return dict(valid=bool(truth), clearance_m=actual['clearance_m'])
