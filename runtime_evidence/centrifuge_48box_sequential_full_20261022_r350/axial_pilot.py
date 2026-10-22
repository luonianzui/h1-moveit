import math

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_planning import result


def axial_pilot_target(body, poses, payload):
    if payload not in ('left', 'right'):
        raise ValueError('Explicit single payload required')
    body = np.asarray(body)
    if body.shape != (4, 4) or not np.isfinite(body).all():
        raise ValueError('Finite actual rigid body required')
    if body[2, 2] <= math.cos(math.radians(2)):
        raise ValueError('Original two-degree initial payload bound not met')
    bucket = pose_matrix(poses['centrifuge']) @ pose_matrix(poses['bucket_' + payload])
    local = np.linalg.inv(bucket) @ body
    target = body.copy()
    target[:3, 3] += bucket[:3, 2] * .015 - bucket[:3, 0] * local[0, 3]
    return target


def retrieval_pilot(request, reference):
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    target = axial_pilot_target(body, request['poses'], request['payload'])
    stage = planner.stage(body, target, seed, 'single_pilot', count=21,
                          support_start='basket', refine=False)
    return dict(result(planner, [stage], reference),
                actual_basket_axis_lift_with_x_relief=True,
                local_y_offset_and_orientation_preserved=True,
                pilot_distance_m=.015, leveling_deferred_until_clear=True)
