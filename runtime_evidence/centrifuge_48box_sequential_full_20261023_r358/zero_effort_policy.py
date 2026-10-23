import math


SHORT_STROKE_PHASES = frozenset((
    'paired_load_open_empty', 'paired_load_close_empty_check', 'paired_load_reopen_empty',
    'paired_load_release_above_bucket', 'machine_paired_empty_grippers',
    'paired_retrieval_open_empty', 'paired_retrieval_release_on_table',
))
GRIPPER_JOINTS = tuple(side + '_dex1_finger_' + str(number) + '_joint'
                       for side in ('left', 'right') for number in (1, 2))


def short_stroke_last_time(phase, target, effort, positions, held_hands, retrieval_closing):
    if phase not in SHORT_STROKE_PHASES or effort != 0 or retrieval_closing:
        return None
    if held_hands:
        raise ValueError('Short zero-final-effort stroke requires qualified normal release or empty hands')
    if not math.isfinite(target) or not all(name in positions and math.isfinite(positions[name])
                                           for name in GRIPPER_JOINTS):
        raise ValueError('All four finite actual finger positions are required')
    if max(abs(positions[name] - target) for name in GRIPPER_JOINTS) > .001:
        return None
    return .5


def restore_fine_setdown_source(source):
    method = '    def fine_setdown_rate(self):\n        return .03\n\n'
    if source.count(method) != 1 or source.count('self.fine_setdown_rate()') != 2:
        raise ValueError('Only the declared default-preserving fine-setdown hook may be reversed')
    return source.replace(method, '', 1).replace('self.fine_setdown_rate()', '.03')
