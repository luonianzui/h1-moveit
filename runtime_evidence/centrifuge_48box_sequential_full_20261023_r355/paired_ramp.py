import math

from dual_motion import common_arm_timing


def arc_time(distance, total_distance, maximum_rate, ramp_s):
    if total_distance == 0:
        return 0.
    acceleration = maximum_rate / ramp_s
    acceleration_time = min(ramp_s, math.sqrt(total_distance / acceleration))
    peak = acceleration * acceleration_time
    acceleration_distance = .5 * acceleration * acceleration_time ** 2
    cruise_distance = max(0., total_distance - 2 * acceleration_distance)
    duration = 2 * acceleration_time + cruise_distance / peak
    if distance <= acceleration_distance:
        return math.sqrt(max(0., 2 * distance / acceleration))
    if distance >= total_distance - acceleration_distance:
        return duration - math.sqrt(max(0., 2 * (total_distance - distance) / acceleration))
    return acceleration_time + (distance - acceleration_distance) / peak


def ramped_common_timing(actual, samples, maximum_rate, ramp_s=.2):
    original = common_arm_timing(actual, samples, maximum_rate)
    if not math.isfinite(ramp_s) or ramp_s < .2:
        raise ValueError('Declared finite paired startup/deceleration ramp required')
    distances = [max(abs(last[name] - first[name]) for name in first)
                 for first, last in zip(original['points'], original['points'][1:])]
    total_distance = sum(distances)
    elapsed = original['times'][0]
    times = [elapsed]
    previous_arc_time = 0.
    cumulative_distance = 0.
    for distance in distances:
        cumulative_distance = min(total_distance, cumulative_distance + distance)
        current_arc_time = arc_time(cumulative_distance, total_distance, maximum_rate, ramp_s)
        elapsed += max(.020, current_arc_time - previous_arc_time)
        times.append(elapsed)
        previous_arc_time = current_arc_time
    return dict(original, times=times, duration_sim_s=elapsed,
                paired_global_ramp_time_sim_s=ramp_s,
                maximum_arc_acceleration_rad_s2=maximum_rate / ramp_s,
                all_original_points_groups_first_time_and_floor_preserved=True,
                position_only_original_joint_polyline_not_changed=True)


PAIR_TIMING_HOOK = '''    def common_timing(self, actual, samples, rate):
        return common_arm_timing(actual, samples, rate)

'''


def restore_pair_ramp_source(name, source):
    if name != 'execute_dual.py':
        return source
    if source.count(PAIR_TIMING_HOOK) != 1:
        raise ValueError('Only the declared default-preserving common timing hook may be reversed')
    return source.replace(PAIR_TIMING_HOOK, '', 1).replace(
        'self.common_timing(self.positions, samples, rate)', 'common_arm_timing(self.positions, samples, rate)')
