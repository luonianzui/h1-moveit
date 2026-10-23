import numpy as np

from alignment_scene import pose_matrix


def centered_pilot_target(body, poses, payload):
    if payload not in ('left', 'right'):
        raise ValueError('Explicit single payload required')
    bucket = pose_matrix(poses['centrifuge']) @ pose_matrix(poses['bucket_' + payload])
    local = np.linalg.inv(bucket) @ body
    target = body.copy()
    target[:3, 3] = (body[:3, 3] - bucket[:3, 0] * local[0, 3]
                    - bucket[:3, 1] * local[1, 3] + bucket[:3, 2] * .015)
    if not np.isfinite(target).all():
        raise ValueError('Finite actual centered basket pilot required')
    return target
