import math

import numpy as np
from scipy.spatial.transform import Rotation

from pilot_centering import centered_pilot_target


def leveled_retrieval_pilot_target(body, poses, payload):
    target = centered_pilot_target(body, poses, payload)
    if body[2, 2] <= math.cos(math.radians(2)):
        raise ValueError('Original two-degree initial payload bound not met')
    yaw = math.atan2(body[1, 0], body[0, 0])
    target[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    if not np.isfinite(target).all():
        raise ValueError('Finite leveled retrieval pilot required')
    return target
