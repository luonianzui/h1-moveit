import math

import numpy as np


ARMS = {side: [side + '_' + suffix + '_joint' for suffix in (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    for side in ('left', 'right')}
TASK_JOINTS = frozenset(['torso_joint'] + ARMS['left'] + ARMS['right'])


def rigid_reference(value):
    matrix = np.asarray(value, dtype=float).copy()
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Held reference must be finite 4x4')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError('Invalid held homogeneous transform')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0) or abs(np.linalg.det(rotation) - 1) > 1e-9:
        raise ValueError('Non-rigid held reference')
    matrix.setflags(write=False)
    return matrix


class HoldRegistry:
    def __init__(self):
        self.rows = {}

    def begin(self, hand, payload, native_reference, fk_reference, source_sim, certificate):
        if hand not in ARMS or payload not in ARMS or hand in self.rows:
            raise ValueError('Unknown hand/payload or attempted held-reference rebase')
        if any(row['payload'] == payload for row in self.rows.values()):
            raise ValueError('The same payload cannot be owned by two hands')
        if not math.isfinite(source_sim) or source_sim < 0:
            raise ValueError('Invalid actual grasp source time')
        if certificate.get('source') != 'gazebo_physics_contact' or certificate.get('midband_verified') is not True:
            raise ValueError('No actual opposed midband contact certificate')
        loads = certificate.get('normal_loads_N', {})
        if set(loads) != {'1', '2'} or not all(math.isfinite(value) and 3 <= value <= 20.5 for value in loads.values()):
            raise ValueError('Both original 3..20.5N preload gates are required')
        self.rows[hand] = dict(payload=payload, source_sim=float(source_sim),
                               native_reference=rigid_reference(native_reference), fk_reference=rigid_reference(fk_reference))

    def release(self, hand):
        if hand not in self.rows:
            raise ValueError('Cannot preclaim release of an unobserved holder')
        return self.rows.pop(hand)

    def snapshot(self):
        return {hand: dict(payload=row['payload'], source_sim=row['source_sim'],
                           native_reference=row['native_reference'].tolist(), fk_reference=row['fk_reference'].tolist())
                for hand, row in self.rows.items()}


def common_arm_timing(actual, samples, rate):
    if not samples or not math.isfinite(rate) or not 0 < rate <= .8:
        raise ValueError('Invalid bounded paired motion rate')
    names = frozenset(samples[0])
    if names != TASK_JOINTS or not names <= actual.keys():
        raise ValueError('Paired motion needs exactly torso and fourteen arm joints')
    points = [{name: float(actual[name]) for name in samples[0]}] + [dict(sample) for sample in samples]
    if any(frozenset(point) != names or not all(math.isfinite(value) for value in point.values()) for point in points):
        raise ValueError('Incomplete or nonfinite paired trajectory')
    elapsed, times = .1, []
    previous = None
    for point in points:
        if previous is not None:
            elapsed += max(.020, max(abs(point[name] - previous[name]) for name in names) / rate)
        times.append(elapsed)
        previous = point
    groups = {'torso': ['torso_joint'], 'left_arm': ARMS['left'], 'right_arm': ARMS['right']}
    return dict(times=times, points=points, groups=groups, duration_sim_s=elapsed,
                single_common_clock=True, original_floor_sim_s=.020)


def validate_common_start(current_sim, start_sim):
    if not all(math.isfinite(value) for value in (current_sim, start_sim)) or current_sim < 0 or start_sim - current_sim < .15:
        raise ValueError('Missed or invalid common actual simulation start')
    return divmod(round(start_sim * 1e9), 1000000000)
