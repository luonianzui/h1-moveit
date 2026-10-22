import math


MAXIMUM_EMPTY_RATE = .8
MAXIMUM_LOADED_RATE = .25
BUTTON_CONTACT_INTERVAL = .02
RETRIEVAL_SOFT_GRASP_SECONDS = 2.


def calibrated_rate(phase, holding, payload):
    if not isinstance(phase, str) or not phase:
        raise ValueError('Motion phase required')
    if phase.endswith('_release_alignment_correction'):
        return .05
    if phase in ('right_align', 'left_align'):
        if holding != phase.split('_')[0] or payload is not None:
            raise ValueError('Loading alignment requires its actual matching holder')
        return .10
    if 'pilot_lift' in phase:
        return .03 if '_retrieval_' in phase else .06
    if phase.endswith('_descend'):
        return .25 if '_retrieval_' in phase else .36
    if phase in ('right_lower', 'left_lower'):
        if holding != phase.split('_')[0] or payload is not None:
            raise ValueError('Loading descent requires its actual matching holder')
        return .065 if holding == 'right' else .13
    if holding is None and payload is None:
        return MAXIMUM_EMPTY_RATE
    if holding not in ('left', 'right'):
        raise ValueError('Unknown actual holder')
    if phase.endswith('_setdown'):
        return .20
    return MAXIMUM_LOADED_RATE


def validate_tracking(errors):
    if not errors or not all(math.isfinite(value) and abs(value) <= .015 for value in errors.values()):
        raise ValueError('Original 0.015rad in-motion tracking guard failed')
