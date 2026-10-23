import math

import numpy as np
from scipy.spatial.transform import Rotation


def alignment_stage_parameters(request, phase, first, last, count, guides):
    if request.get('mode')!='load_alignment' or phase!='forward_inside_basket_raise':
        return count,guides
    holders = request.get('original_runtime_holder_references')
    if not holders:
        return count,guides
    if (request.get('retrieval') is not False or not isinstance(holders,dict) or set(holders)!={'left','right'}
            or count!=81 or not isinstance(guides,list) or not guides
            or set(first)!={'left','right'} or set(last)!=set(first)
            or set(request.get('assignment',{}))!=set(first)
            or set(request['assignment'].values())!={'left','right'}):
        raise ValueError('Only original actual paired optional alignment may use micro parameters')
    source = request.get('source_sim')
    if source is None or not math.isfinite(source):
        raise ValueError('Finite actual alignment snapshot required')
    maximum_distance = maximum_angle = 0.
    for hand in first:
        row = holders[hand]
        if (row.get('payload')!=request['assignment'][hand]
                or not math.isfinite(row.get('source_sim',math.nan)) or row['source_sim']>source):
            raise ValueError('Original physical holder and reference epoch required')
        origin,goal = np.asarray(first[hand]),np.asarray(last[hand])
        if origin.shape!=(4,4) or goal.shape!=(4,4) or not np.isfinite(origin).all() or not np.isfinite(goal).all():
            raise ValueError('Finite actual paired body matrices required')
        maximum_distance = max(maximum_distance,float(np.linalg.norm(goal[:3,3]-origin[:3,3])))
        maximum_angle = max(maximum_angle,float(Rotation.from_matrix(origin[:3,:3].T@goal[:3,:3]).magnitude()))
    if maximum_distance>.005 or maximum_angle>math.radians(1.):
        return count,guides
    return 11,None
