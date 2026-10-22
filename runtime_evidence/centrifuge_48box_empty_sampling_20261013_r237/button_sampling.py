import hashlib
import importlib.util
import json
import math
from pathlib import Path


REFERENCE = Path(__file__).resolve().parent.parent / 'centrifuge_48box_alignment_cap_20261013_r236'
SUFFIXES = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
GEOMETRIC_TOLERANCE_RAD = .000025
FLOOR_SIM_S = .020


def reducer():
    source = REFERENCE / 'command_sampling.py'
    expected = json.loads((REFERENCE / 'identity.json').read_text())['sha256'][source.name]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == expected
    specification = importlib.util.spec_from_file_location('verified_sampling', source)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def propose(side, names, points, holding=None, payload=None, fixed_interval=None):
    if side not in ('left', 'right'):
        raise ValueError('Only the exact seven physical arm joints are allowed')
    expected = [side + '_' + suffix + '_joint' for suffix in SUFFIXES]
    if list(names) != expected:
        raise ValueError('Only the exact seven physical arm joints are allowed')
    if holding is not None or payload is not None:
        raise ValueError('Button sampling requires both empty-hand indicators')
    if len(points) < 2 or any(len(point) != 7 or not all(math.isfinite(value) for value in point) for point in points):
        raise ValueError('Finite seven-joint waypoints required')
    if fixed_interval is not None:
        return dict(points=[list(point) for point in points], original_indices=list(range(len(points))),
                    original_points=len(points), retained_points=len(points), maximum_joint_deviation_rad=0.,
                    contact_stroke_unchanged=True, dense_FCL_required=True, physical_tracking_required=True)
    result = reducer().reduce_polyline([dict(zip(names, point)) for point in points], GEOMETRIC_TOLERANCE_RAD)
    return dict(points=[[sample[name] for name in names] for sample in result['samples']],
                original_indices=result['original_indices'], original_points=result['original_points'],
                retained_points=result['retained_points'], maximum_joint_deviation_rad=result['maximum_joint_deviation_rad'],
                contact_stroke_unchanged=True, dense_FCL_required=True, physical_tracking_required=True)


def commanded_duration(points, rate=.8):
    if not math.isfinite(rate) or not 0 < rate <= .8:
        raise ValueError('Rate must remain within the existing empty-arm calibration')
    if len(points) < 2 or any(len(point) != 7 or not all(math.isfinite(value) for value in point) for point in points):
        raise ValueError('Finite seven-joint waypoints required')
    elapsed = .05
    for previous, current in zip(points, points[1:]):
        elapsed += max(FLOOR_SIM_S, max(abs(value - previous_value) / rate
                                     for value, previous_value in zip(current, previous)))
    return elapsed
