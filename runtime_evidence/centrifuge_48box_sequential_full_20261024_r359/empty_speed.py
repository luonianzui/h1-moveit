import math


EMPTY_PHASES = frozenset()
EMPTY_RATE_SCALE = 1.0
EMPTY_MAXIMUM_RATE_RAD_S = .36
EMPTY_MINIMUM_INTERVAL_S = .020
LOADED_MINIMUM_INTERVAL_S = .020
EMPTY_BUTTON_RATE_RAD_S = .44


def empty_motion(phase, holding, held_payload):
    return phase in EMPTY_PHASES and holding is None and held_payload is None


def motion_rate(phase, requested, holding, held_payload):
    if not math.isfinite(requested) or requested <= 0:
        raise ValueError('Positive finite trajectory rate required')
    if empty_motion(phase, holding, held_payload):
        return min(requested * EMPTY_RATE_SCALE, EMPTY_MAXIMUM_RATE_RAD_S)
    return requested


def minimum_interval(phase, holding, held_payload):
    if empty_motion(phase, holding, held_payload):
        return EMPTY_MINIMUM_INTERVAL_S
    return LOADED_MINIMUM_INTERVAL_S


def button_travel_rate(holding, held_payload):
    if holding is not None or held_payload is not None:
        raise ValueError('Physical button travel requires two empty hands')
    return EMPTY_BUTTON_RATE_RAD_S
