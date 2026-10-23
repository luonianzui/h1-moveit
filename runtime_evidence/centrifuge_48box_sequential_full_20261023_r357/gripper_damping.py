import copy
import re

import yaml


DAMPING = 6.


def verify_damping_fork(original, candidate):
    old, new = b'        d: 3.0\n', b'        d: 6.0\n'
    if original.count(old) != 4 or candidate.count(new) != 4 or candidate.replace(new, old) != original:
        raise ValueError('Only the four declared gripper PID derivative fields may change')
    previous, current = yaml.safe_load(original), yaml.safe_load(candidate)
    restored = copy.deepcopy(current)
    for side in ('left', 'right'):
        parameters = current[side + '_gripper_controller']['ros__parameters']
        if parameters['command_interfaces'] != ['effort']:
            raise ValueError('Original effort-only controller must remain')
        for number in (1, 2):
            name = side + f'_dex1_finger_{number}_joint'
            gain = parameters['gains'][name]
            if gain['d'] != DAMPING or gain['p'] != 500. or gain['i'] != 1000. or gain['u_clamp_min'] != -2. or gain['u_clamp_max'] != 2.:
                raise ValueError('Original P/I/feedback force clamps must remain')
            restored[side + '_gripper_controller']['ros__parameters']['gains'][name]['d'] = 3.
    if restored != previous:
        raise ValueError('Undeclared controller change')
    return True


def verify_damping_output(output, returncode):
    if returncode != 0 or re.fullmatch(r'\s*Double value is: 6\.0\s*', output) is None:
        raise ValueError('Fresh gripper PID D must read exactly 6.0')
    return DAMPING
