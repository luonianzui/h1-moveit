def allow_planning_grasp_touch(mode):
    if mode in ('load_pickup', 'retrieval_pickup', 'empty_return', 'empty_gripper_after_retreat', 'empty_return_after_gripper'):
        return False
    if mode in ('pilot', 'load_transport', 'load_alignment', 'retrieval_transport'):
        return True
    raise ValueError('Unknown physical planning phase')


def bounded_motion(phase, requested_rate):
    if phase == 'machine_paired_empty_return':
        return min(requested_rate, .4), .002
    if phase == 'paired_retrieval_approach':
        return min(requested_rate, .4), .002
    if phase in ('paired_load_descend', 'paired_retrieval_descend',
                 'machine_paired_empty_reverse_forward_inside_basket_raise'):
        return min(requested_rate, .13), .002
    return requested_rate, .015


def table_birth_violation(payload, position, angle_rad):
    import math

    if payload not in ('left', 'right'):
        raise ValueError('Unknown table payload')
    expected = (.44, .38 if payload == 'left' else -.38, .84475)
    if len(position) != 3 or not all(math.isfinite(value) for value in (*position, angle_rad)):
        return 'invalid_native_table_pose'
    if math.dist(position, expected) >= .0003:
        return 'table_payload_displaced_before_grasp'
    if abs(angle_rad) >= math.radians(.2):
        return 'table_payload_rotated_before_grasp'
    return None
