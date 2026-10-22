"""Pure-data contracts for the candidate-3 plan-only replanner."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np


SCHEMA = 'h1_candidate3_dynamic_plan/v1'
PLANNER_VERSION = 'h1_candidate3_replanner/826-g5'
AUDITOR_VERSION = 'h1_candidate3_auditor/826-h1'
TOPOLOGY = 'candidate3_cabinet_front_positive_x_parallel90_dynamic_v1'
DOOR_ARC_TARGET_COUNT = 601
MAXIMUM_ADJACENT_CHANGE_RAD = math.radians(4.9)
DENSE_RESOLUTION_RAD = math.radians(0.49)
DEFAULT_CLEARANCE_M = 0.010

ASSET_ORDER = (
    'handle_position_only.trajectory.json',
    'left_clear.trajectory.json',
    'door.trajectory.json',
    'cube.parallel90.neutral.trajectory.json',
    'door_close.trajectory.json',
    'left_regrasp.trajectory.json',
    'handle_relock.trajectory.json',
)

STAGE_NAMES = (
    'handle_unlock',
    'left_clear',
    'door_open_601',
    'cube_pick_parallel90_insert_release',
    'door_close_601_reverse',
    'left_regrasp',
    'handle_relock',
)

ARM_LIMITS = {
    'shoulder_pitch': (1.0, 2.0),
    'shoulder_roll': (1.0, 2.0),
    'shoulder_yaw': (1.2, 2.5),
    'elbow': (1.2, 2.5),
    'wrist_roll': (1.5, 3.0),
    'wrist_pitch': (1.5, 3.0),
    'wrist_yaw': (1.5, 3.0),
}


def canonical_bytes(value) -> bytes:
    """Return the sole canonical JSON encoding used by plans and assets."""
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'),
        ensure_ascii=True, allow_nan=False).encode('utf-8')


def canonical_hash(value) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def load_sealed_assets(root: Path):
    """Load the seven immutable 826.2 trajectory assets fail closed."""
    root = Path(root)
    assets = []
    for name in ASSET_ORDER:
        path = root / name
        document = json.loads(path.read_text(encoding='utf-8'))
        if set(document) != {'payload', 'sha256'}:
            raise ValueError(f'asset wrapper contract: {name}')
        actual = canonical_hash(document['payload'])
        if actual != document['sha256']:
            raise ValueError(f'asset payload hash: {name}')
        if document['payload'].get('schema') != 'h1_joint_trajectory_asset/v1':
            raise ValueError(f'asset schema: {name}')
        assets.append({
            'name': name,
            'payload_hash': actual,
            'payload': document['payload'],
        })
    cache = root / 'left_rack_q60_parallel90_neutral_candidate3.cache'
    cache_hash = hashlib.sha256(cache.read_bytes()).hexdigest()
    cube_contract = assets[3]['payload'].get('cube_cache_contract', {})
    if cube_contract.get('cache_sha256') != cache_hash:
        raise ValueError('candidate-3 cache hash')
    provenance = cube_contract.get('route_contract', {}).get('provenance_name')
    if provenance != 'candidate3_cabinet_front_positive_x_parallel90_v1':
        raise ValueError('candidate-3 provenance')
    counts = cube_contract.get('stage_point_counts')
    if counts != {'approach': 32, 'carry': 360,
                  'support': 28, 'retreat': 413}:
        raise ValueError('candidate-3 cache stage counts')
    return assets, cache_hash


def pose_matrix(pose) -> np.ndarray:
    """Convert a geometry pose-like object to an SE(3) matrix."""
    q = np.asarray([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ], dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(result).all():
        raise ValueError('non-finite pose')
    return result


def yaw_transform(xyz, yaw_rad) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    result[:3, 3] = xyz
    return result


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Return an x,y,z,w quaternion with a deterministic sign."""
    value = np.empty(4, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        value[:] = [
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ]
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            value[:] = [
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ]
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            value[:] = [
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ]
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            value[:] = [
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
    value /= np.linalg.norm(value)
    if value[3] < 0:
        value *= -1
    return value


def interpolate_transform(first: np.ndarray, second: np.ndarray,
                          fraction: float) -> np.ndarray:
    """Interpolate SE(3) using linear translation and quaternion SLERP."""
    fraction = min(1.0, max(0.0, float(fraction)))
    qa = quaternion_from_matrix(first[:3, :3])
    qb = quaternion_from_matrix(second[:3, :3])
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb = -qb
        dot = -dot
    if dot > 0.9995:
        q = qa + fraction * (qb - qa)
        q /= np.linalg.norm(q)
    else:
        theta = math.acos(min(1.0, max(-1.0, dot)))
        first_weighted = math.sin((1 - fraction) * theta) * qa
        second_weighted = math.sin(fraction * theta) * qb
        q = (first_weighted + second_weighted) / math.sin(theta)
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])
    result[:3, 3] = (
        (1 - fraction) * first[:3, 3] + fraction * second[:3, 3])
    return result


def select_knots(positions, threshold_rad=math.radians(2.0)):
    """Retain endpoints and enough source states to bound seed spacing."""
    values = np.asarray(positions, dtype=float)
    if values.ndim != 2 or len(values) == 0:
        raise ValueError('empty trajectory positions')
    selected = [0]
    for index in range(1, len(values) - 1):
        if float(np.max(np.abs(values[index] - values[selected[-1]]))) >= (
                threshold_rad):
            selected.append(index)
    if len(values) > 1:
        selected.append(len(values) - 1)
    return sorted(set(selected))


def dense_interpolate(states, resolution_rad=DENSE_RESOLUTION_RAD):
    """Densify a joint path without silently dropping either endpoint."""
    values = [np.asarray(value, dtype=float) for value in states]
    if not values:
        raise ValueError('empty IK path')
    output = [values[0].copy()]
    for target in values[1:]:
        change = float(np.max(np.abs(target - output[-1])))
        count = max(1, int(math.ceil(change / resolution_rad)))
        start = output[-1].copy()
        output.extend(
            start + (target - start) * (step / count)
            for step in range(1, count + 1))
    return output


def reverse_stage(forward, template, name):
    """Construct an exact inverse using the reverse-stage envelope."""
    if not template.get('messages'):
        raise ValueError(f'reverse template missing: {name}')
    template_widths = {len(value['joint_names'])
                       for value in template['messages']}
    forward_widths = {len(value['joint_names'])
                      for value in forward['messages']}
    if template_widths != forward_widths:
        raise ValueError(f'reverse topology groups: {name}')
    messages = []
    for sequence, source in enumerate(reversed(forward['messages'])):
        message = copy.deepcopy(source)
        message['sequence'] = sequence
        message['points'] = list(reversed(message['points']))
        messages.append(message)
    return {'name': name, 'messages': messages}


def stage_hash(stage) -> str:
    return canonical_hash(stage)
