from pathlib import Path
import struct
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.spatial.transform import Rotation


TABLE_SETDOWN_PHASES = frozenset(('paired_retrieval_setdown', 'paired_retrieval_setdown_fine'))
REQUIRED_MARGIN_M = .003


def bucket_collision_bounds(model_path):
    model_path = Path(model_path)
    model = element_tree.parse(model_path).getroot().find('model')
    result = {}
    for bucket in ('left', 'right'):
        collision = model.find("./link[@name='bucket_" + bucket + "']/collision")
        mesh = collision.find('geometry/mesh')
        path = model_path.parents[2] / mesh.findtext('uri').split('h1_2_description/', 1)[1]
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        if count == 0 or len(data) != 84 + 50 * count:
            raise ValueError('Invalid original bucket collision STL')
        vertices = np.array([struct.unpack_from('<9f', data, 84 + 50 * index + 12)
                             for index in range(count)]).reshape(-1, 3)
        scale = np.array([float(value) for value in mesh.findtext('scale', '1 1 1').split()])
        pose = np.array([float(value) for value in collision.findtext('pose', '0 0 0 0 0 0').split()])
        vertices = (vertices * scale) @ Rotation.from_euler('xyz', pose[3:]).as_matrix().T + pose[:3]
        if not np.isfinite(vertices).all():
            raise ValueError('Nonfinite original bucket collision geometry')
        result[bucket] = dict(minimum=vertices.min(axis=0).tolist(), maximum=vertices.max(axis=0).tolist())
    return result


def footprint_clearance(corners_by_bucket, bounds, pairs):
    failed = dict(valid=False, finite=False, clearance_m=0., required_margin_m=REQUIRED_MARGIN_M,
                  no_basket_contact=False, per_bucket_clearance_m={})
    if set(corners_by_bucket) != {'left', 'right'} or set(bounds) != {'left', 'right'}:
        return failed
    distances = {}
    for bucket in ('left', 'right'):
        corners = np.asarray(corners_by_bucket[bucket], dtype=float)
        lower = np.asarray(bounds[bucket]['minimum'], dtype=float)
        upper = np.asarray(bounds[bucket]['maximum'], dtype=float)
        if corners.shape != (8, 3) or lower.shape != (3,) or upper.shape != (3,):
            return failed
        if not all(np.isfinite(value).all() for value in (corners, lower, upper)) or not np.all(lower < upper):
            return failed
        box_lower, box_upper = corners[:, :2].min(axis=0), corners[:, :2].max(axis=0)
        separation = np.maximum(np.maximum(box_lower - upper[:2], lower[:2] - box_upper), 0.)
        distances[bucket] = float(np.linalg.norm(separation))
    if any(not isinstance(pair, dict) or not isinstance(pair.get('first'), str) or
           not isinstance(pair.get('second'), str) for pair in pairs):
        return failed
    no_contact = not any('centrifuge::bucket_' in pair['first'] + pair['second'] for pair in pairs)
    minimum = min(distances.values())
    return dict(valid=minimum >= REQUIRED_MARGIN_M and no_contact, finite=True, clearance_m=minimum,
                required_margin_m=REQUIRED_MARGIN_M, no_basket_contact=no_contact,
                per_bucket_clearance_m=distances)


def table_bucket_clearance(poses, payload, pairs, bounds):
    from execute_loading import bucket_relative_corners, pose_matrix

    if payload not in ('left', 'right'):
        raise ValueError('Unknown physical payload')
    own_corners = bucket_relative_corners(poses, payload)
    machine = pose_matrix(poses['centrifuge'])
    own_bucket = machine @ pose_matrix(poses['bucket_' + payload])
    corners = {}
    for bucket in ('left', 'right'):
        target = machine @ pose_matrix(poses['bucket_' + bucket])
        relative = np.linalg.inv(target) @ own_bucket
        corners[bucket] = own_corners @ relative[:3, :3].T + relative[:3, 3]
    return footprint_clearance(corners, bounds, pairs)
