"""Versioned AprilTag-map loading, validation, and content hashing."""

from __future__ import annotations

import hashlib
from pathlib import Path

import yaml

from .tag_geometry import transform_from_xyz_rpy


def load_tag_map(path: Path, texture_roots=()):
    path = Path(path)
    payload = path.read_bytes()
    raw = yaml.safe_load(payload)
    if raw.get('schema') != 'h1_2_apriltag_map/v1':
        raise ValueError('unsupported tag-map schema')
    if raw.get('family') != 'tag36h11':
        raise ValueError('only tag36h11 is accepted by contract v1')
    ids = set()
    tags = {}
    digest = hashlib.sha256()
    digest.update(payload)
    roots = [Path(value) for value in texture_roots]
    for object_name in ('cabinet', 'door', 'cube'):
        value = raw.get('objects', {}).get(object_name)
        if not value or not value.get('tags'):
            raise ValueError(f'missing required object {object_name}')
        correction = value.get('pose_correction_in_object', {
            'xyz': [0.0, 0.0, 0.0],
            'rpy': [0.0, 0.0, 0.0],
        })
        if (len(correction.get('xyz', [])) != 3 or
                len(correction.get('rpy', [])) != 3):
            raise ValueError(f'invalid pose correction for {object_name}')
        value['object_pose_correction'] = transform_from_xyz_rpy(
            correction['xyz'], correction['rpy'])
        for record in value['tags']:
            tag_id = int(record['id'])
            if tag_id in ids or tag_id < 0:
                raise ValueError(f'duplicate or invalid tag id {tag_id}')
            ids.add(tag_id)
            size = float(record['size_m'])
            visual = float(record['visual_size_m'])
            if not 0.015 <= size <= 0.100 or abs(visual / size - 1.25) > 1e-9:
                raise ValueError(f'invalid physical/visual size for tag {tag_id}')
            pose = record['pose_in_object']
            if len(pose['xyz']) != 3 or len(pose['rpy']) != 3:
                raise ValueError(f'invalid pose for tag {tag_id}')
            tags[tag_id] = {
                'id': tag_id,
                'object_name': object_name,
                'object_frame': str(value['frame']),
                'size_m': size,
                'visual_size_m': visual,
                'object_from_tag': transform_from_xyz_rpy(pose['xyz'], pose['rpy']),
                'texture': str(record['texture']),
            }
            texture = next(
                (root / record['texture'] for root in roots
                 if (root / record['texture']).is_file()), None)
            if roots and texture is None:
                raise ValueError(f'missing texture for tag {tag_id}')
            if texture is not None:
                digest.update(f'\0tag:{tag_id}\0'.encode('ascii'))
                digest.update(texture.read_bytes())
    raw['tags_by_id'] = tags
    raw['config_hash'] = digest.hexdigest()
    return raw
