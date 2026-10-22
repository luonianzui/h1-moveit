import math

import numpy as np


MAXIMUM_GEOMETRIC_DEVIATION_RAD = .000025
MINIMUM_CONTROL_INTERVAL_S = .020
MAXIMUM_COLLISION_STEP_RAD = .002


def reduce_polyline(samples, tolerance=MAXIMUM_GEOMETRIC_DEVIATION_RAD):
    if not samples or not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Finite nonnegative tolerance and nonempty path required')
    names = list(samples[0])
    if not names or any(set(sample) != set(names) for sample in samples):
        raise ValueError('Every waypoint must contain the same joints')
    values = np.array([[sample[name] for name in names] for sample in samples], dtype=float)
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite geometry')
    parameters = np.r_[0., np.cumsum(np.max(np.abs(np.diff(values, axis=0)), axis=1))]
    retained = {0, len(samples) - 1}
    pending = [(0, len(samples) - 1)]
    while pending:
        first, last = pending.pop()
        if last <= first + 1:
            continue
        length = parameters[last] - parameters[first]
        fractions = ((parameters[first:last + 1] - parameters[first]) / length
                     if length else np.zeros(last - first + 1))
        estimated = values[first] + fractions[:, None] * (values[last] - values[first])
        errors = np.max(np.abs(values[first:last + 1] - estimated), axis=1)
        middle = int(np.argmax(errors)) + first
        if float(np.max(errors)) > tolerance:
            assert first < middle < last
            retained.add(middle)
            pending.extend(((first, middle), (middle, last)))
    indices = sorted(retained)
    maximum_error = 0.
    for first, last in zip(indices, indices[1:]):
        length = parameters[last] - parameters[first]
        fractions = ((parameters[first:last + 1] - parameters[first]) / length
                     if length else np.zeros(last - first + 1))
        estimated = values[first] + fractions[:, None] * (values[last] - values[first])
        maximum_error = max(maximum_error, float(np.max(np.abs(values[first:last + 1] - estimated))))
    assert maximum_error <= tolerance + 1e-14
    return dict(samples=[dict(samples[index]) for index in indices], original_indices=indices,
                original_points=len(samples), retained_points=len(indices),
                maximum_joint_deviation_rad=maximum_error, tolerance_rad=tolerance,
                dense_collision_reaudit_still_required=True, original_endpoints_unchanged=True)


def duration(samples, rate, floor=MINIMUM_CONTROL_INTERVAL_S):
    if not math.isfinite(rate) or rate <= 0 or not math.isfinite(floor) or floor < MINIMUM_CONTROL_INTERVAL_S:
        raise ValueError('Invalid rate or shortened 20ms floor')
    reduced = reduce_polyline(samples, 0.)
    assert reduced['original_points'] == len(samples)
    return .1 + sum(max(floor, max(abs(current[name] - previous[name]) / rate for name in current))
                    for previous, current in zip(samples, samples[1:]))


def dense_segments(samples):
    reduce_polyline(samples, 0.)
    yield dict(samples[0])
    for previous, current in zip(samples, samples[1:]):
        steps = max(1, math.ceil(max(abs(current[name] - previous[name]) for name in current)
                                 / MAXIMUM_COLLISION_STEP_RAD))
        for fraction in np.linspace(0, 1, steps + 1)[1:]:
            yield {name:previous[name] + float(fraction) * (current[name] - previous[name]) for name in current}
