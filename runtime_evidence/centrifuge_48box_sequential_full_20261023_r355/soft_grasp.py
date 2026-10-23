import math


def soft_grasp_profile(initial_positions, target, initial_effort, final_effort, duration=6.0, intervals=60):
    values = list(initial_positions) + [target, initial_effort, final_effort, duration]
    if len(initial_positions) != 2 or not all(math.isfinite(value) for value in values):
        raise ValueError('Finite two-jaw profile required')
    if duration < 1 or intervals < 20 or not 0 <= initial_effort <= final_effort <= 18:
        raise ValueError('Invalid soft grasp timing or effort')
    profile = []
    for index in range(intervals + 1):
        fraction = index / intervals
        progress = 10 * fraction ** 3 - 15 * fraction ** 4 + 6 * fraction ** 5
        speed = (30 * fraction ** 2 - 60 * fraction ** 3 + 30 * fraction ** 4) / duration
        profile.append(dict(time_s=.05 + duration * fraction,
            positions=[position + (target - position) * progress for position in initial_positions],
            velocities=[(target - position) * speed for position in initial_positions],
            effort=[initial_effort + (final_effort - initial_effort) * progress] * 2))
    return profile

