import math

import numpy as np

from execute_loading import bucket_relative_corners, pose_matrix

CONTACT_GEOMETRY_ROUNDING_M = .000005
ORIGINAL_CONTACT_DEPTH_LIMIT_M = .001


def supported_corner_retention(corners, pairs, payload, bucket_rotation):
    expected_box = 'box48_' + payload + '::'
    expected_bucket = 'centrifuge::bucket_' + payload + '::'
    supported = False
    contacts_valid = bool(pairs)
    maximum_depth = 0.0
    wall_contact = [False, False]
    floor_contact = False
    for pair in pairs:
        names = (pair['first'], pair['second'])
        matching = ((names[0].startswith(expected_box) and names[1].startswith(expected_bucket)) or
                    (names[1].startswith(expected_box) and names[0].startswith(expected_bucket)))
        finite = all(math.isfinite(pair.get(key, math.nan)) for key in ('depth', 'force'))
        supported = supported or (matching and finite and pair['force'] > 0)
        contacts_valid = contacts_valid and matching and finite and 0 <= pair['depth'] <= ORIGINAL_CONTACT_DEPTH_LIMIT_M
        maximum_depth = max(maximum_depth, pair['depth'])
        for normal, force in zip(pair.get('normals', []), pair.get('force_vectors', [])):
            local_normal = np.asarray(normal) @ bucket_rotation
            local_force = np.asarray(force) @ bucket_rotation
            if not np.isfinite(local_normal).all() or not np.isfinite(local_force).all():
                contacts_valid = False
                continue
            floor_contact = floor_contact or bool(matching and abs(local_normal[2]) >= .95)
            for axis in (0, 1):
                wall_contact[axis] = wall_contact[axis] or bool(matching and abs(local_normal[axis]) >= .95)
    side_overhangs = [max(0.0, float(np.max(np.abs(corners[:, axis])) - bound))
                     for axis, bound in enumerate((.045, .067))]
    penetration_bound = ORIGINAL_CONTACT_DEPTH_LIMIT_M
    sidewall_certified = all(overhang <= penetration_bound and
                            (overhang <= CONTACT_GEOMETRY_ROUNDING_M or wall_contact[axis])
                            for axis, overhang in enumerate(side_overhangs))
    bottom_min = float(np.min(corners[:, 2]))
    bottom_penetration = max(0.0, -.0300 - bottom_min)
    bottom_certified = bool(bottom_min >= -.0305 or
                            (bottom_penetration <= ORIGINAL_CONTACT_DEPTH_LIMIT_M and floor_contact))
    vertical_inside = bool(bottom_certified and np.max(corners[:, 2]) <= .0305)
    return dict(valid=bool(np.isfinite(corners).all() and sidewall_certified and vertical_inside and
                           supported and contacts_valid),
                side_overhang_m=max(side_overhangs), side_overhangs_m=side_overhangs,
                maximum_contact_depth_m=maximum_depth, contact_certified_bound_m=penetration_bound,
                contact_geometry_rounding_m=CONTACT_GEOMETRY_ROUNDING_M,
                original_contact_depth_limit_m=ORIGINAL_CONTACT_DEPTH_LIMIT_M,
                sidewall_certified=sidewall_certified, wall_contact=wall_contact,
                floor_contact=floor_contact, bottom_penetration_m=bottom_penetration,
                vertical_inside=vertical_inside, supported=supported, contacts_valid=contacts_valid,
                corners_min=corners.min(axis=0).tolist(), corners_max=corners.max(axis=0).tolist())


def retention_metrics(poses, pairs, payload):
    bucket = pose_matrix(poses['centrifuge']) @ pose_matrix(poses['bucket_' + payload])
    return supported_corner_retention(bucket_relative_corners(poses, payload), pairs, payload, bucket[:3, :3])

