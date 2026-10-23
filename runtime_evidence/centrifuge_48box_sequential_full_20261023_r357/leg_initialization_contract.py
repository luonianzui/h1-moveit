import math

LEG_JOINTS = tuple(side + '_' + suffix + '_joint' for side in ('left', 'right')
                   for suffix in ('hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll'))


def targets_from_original(initial, measured):
    if set(LEG_JOINTS) - measured.keys():
        raise ValueError('All twelve measured leg joints required')
    targets = [float(initial[name]) for name in LEG_JOINTS]
    actual = [float(measured[name]) for name in LEG_JOINTS]
    if not all(math.isfinite(value) for value in targets + actual):
        raise ValueError('Finite targets and original measured positions required')
    if max(abs(left - right) for left, right in zip(targets, actual)) > .15:
        raise ValueError('Outside the bounded fixed-pelvis initialization envelope')
    return targets, actual


def endpoint_error(initial, measured):
    targets, actual = targets_from_original(initial, measured)
    return max(abs(left - right) for left, right in zip(targets, actual))

