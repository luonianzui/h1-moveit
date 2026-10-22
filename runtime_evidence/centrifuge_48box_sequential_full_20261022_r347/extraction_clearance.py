from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from execute_loading import bucket_relative_corners


def bucket_rim_bounds(model_path):
    model_path = Path(model_path)
    model = ET.parse(model_path).getroot().find('model')
    bounds = {}
    for payload in ('left', 'right'):
        collision = model.find("./link[@name='bucket_" + payload + "']/collision")
        mesh = collision.find('geometry/mesh')
        path = model_path.parents[2] / mesh.findtext('uri').split('h1_2_description/', 1)[1]
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) != 84 + 50 * count or count == 0:
            raise ValueError('Invalid basket collision STL')
        vertices = np.array([struct.unpack_from('<9f', data, 84 + 50 * index + 12)
                             for index in range(count)]).reshape(-1, 3)
        scale = np.array([float(value) for value in mesh.findtext('scale', '1 1 1').split()])
        pose = np.array([float(value) for value in collision.findtext('pose', '0 0 0 0 0 0').split()])
        vertices = (vertices * scale) @ Rotation.from_euler('xyz', pose[3:]).as_matrix().T + pose[:3]
        if not np.isfinite(vertices).all():
            raise ValueError('Nonfinite basket geometry')
        bounds[payload] = float(vertices[:, 2].max())
    return bounds


def corner_clearance(corners, pairs, rim_z, required_margin=.003):
    corners = np.asarray(corners)
    finite = bool(corners.shape == (8, 3) and np.isfinite(corners).all() and np.isfinite(rim_z))
    minimum_z = float(corners[:, 2].min()) if finite else float('-inf')
    clearance = minimum_z - rim_z
    no_basket_contact = not any('centrifuge::bucket_' in pair['first'] + pair['second'] for pair in pairs)
    return dict(valid=bool(finite and clearance >= required_margin and no_basket_contact),
                bottom_corner_z_m=minimum_z, basket_collision_rim_z_m=rim_z,
                clearance_m=clearance, required_margin_m=required_margin, no_basket_contact=no_basket_contact)


def clearance_metrics(poses, payload, pairs, rim_z):
    return corner_clearance(bucket_relative_corners(poses, payload), pairs, rim_z)

