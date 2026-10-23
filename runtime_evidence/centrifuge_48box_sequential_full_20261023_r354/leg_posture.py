import math


LEG_JOINTS = tuple(side + '_' + suffix + '_joint' for side in ('left', 'right')
                   for suffix in ('hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll'))


def leg_posture_targets(initial, actual):
    targets = [float(initial[name]) for name in LEG_JOINTS]
    measured = [float(actual[name]) for name in LEG_JOINTS]
    if not all(math.isfinite(value) for value in targets + measured):
        raise ValueError('Nonfinite leg posture')
    if max(abs(target - value) for target, value in zip(targets, measured)) >= .010:
        raise ValueError('Original passive startup posture not settled')
    return targets, measured


def posture_controller_active(output):
    rows = [line.split() for line in output.splitlines()
            if line.split() and line.split()[0] == 'leg_posture_controller']
    return len(rows) == 1 and len(rows[0]) >= 3 and rows[0][-1] == 'active'

