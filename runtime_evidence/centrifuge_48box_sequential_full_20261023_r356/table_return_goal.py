import math

import numpy as np
from scipy.spatial.transform import Rotation


def nearest_table_yaw(body):
    if not np.isfinite(body).all():
        raise ValueError('Nonfinite payload pose')
    yaw = math.atan2(body[1, 0], body[0, 0])
    return math.pi if abs(yaw) > math.pi / 2 else 0.0


def table_orientation_error(body, yaw):
    if not np.isfinite(body).all() or not math.isfinite(yaw) or yaw not in (0.0, math.pi):
        raise ValueError('Invalid aligned table target')
    target = Rotation.from_euler('z', yaw).as_matrix()
    return Rotation.from_matrix(target.T @ body[:3, :3]).magnitude()

