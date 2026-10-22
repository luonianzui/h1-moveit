from functools import lru_cache
import struct

import numpy as np
from scipy.spatial.transform import Rotation


@lru_cache(maxsize=512)
def matrix_from_bytes(packed):
    values = struct.unpack('7d', packed)
    matrix = np.eye(4)
    matrix[:3, 3] = values[:3]
    matrix[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite body pose')
    matrix.setflags(write=False)
    return matrix


def cached_pose_matrix(value):
    position = value.get('position', {})
    orientation = value.get('orientation', {})
    packed = struct.pack('7d', position.get('x', 0), position.get('y', 0), position.get('z', 0),
                         orientation.get('x', 0), orientation.get('y', 0),
                         orientation.get('z', 0), orientation.get('w', 0))
    return matrix_from_bytes(packed).copy()
