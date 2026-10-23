from functools import lru_cache
import itertools
import struct

import numpy as np

from cached_pose import matrix_from_bytes


BASE_CORNERS = np.array(list(itertools.product((-1, 1), repeat=3))) * np.array([.1187450051085, .08814010043, .0595]) / 2
BASE_CORNERS.setflags(write=False)


def pose_key(value):
    position = value.get('position', {})
    orientation = value.get('orientation', {})
    return struct.pack('7d', position.get('x', 0), position.get('y', 0), position.get('z', 0),
                       orientation.get('x', 0), orientation.get('y', 0),
                       orientation.get('z', 0), orientation.get('w', 0))


@lru_cache(maxsize=512)
def bucket_from_keys(assembly_key, bucket_key):
    bucket = matrix_from_bytes(assembly_key) @ matrix_from_bytes(bucket_key)
    bucket.setflags(write=False)
    return bucket


@lru_cache(maxsize=512)
def corners_from_keys(body_key, assembly_key, bucket_key):
    body = matrix_from_bytes(body_key)
    bucket = bucket_from_keys(assembly_key, bucket_key)
    relative = np.linalg.inv(bucket) @ body
    corners = BASE_CORNERS @ relative[:3, :3].T + relative[:3, 3]
    corners.setflags(write=False)
    return corners


def cached_bucket_corners(poses, side):
    return corners_from_keys(pose_key(poses['box48_' + side]), pose_key(poses['centrifuge']),
                             pose_key(poses['bucket_' + side])).copy()
