import math

from dual_motion import ARMS, TASK_JOINTS


PROFILE = dict(
    name='simulation_speed_priority_v1',
    empty_rate_rad_s=1.2, loaded_rate_rad_s=.8,
    near_rate_rad_s=.4, pilot_fine_rate_rad_s=.15,
    point_floor_sim_s=.002, first_point_sim_s=.04,
    tracking_near_rad=.04, tracking_free_rad=.08,
    button_tracking_rad=.15, button_contact_interval_sim_s=.02,
    gripper_velocity_monitor_m_s=.6, original_model_gripper_velocity_m_s=.2,
    endpoint_rad=.002, endpoint_hold_sim_s=.05,
    static_grip_hold_sim_s=1., release_hold_sim_s=.5,
    terminal_hold_sim_s=1., steady_spin_sim_s=30.,
    pose_clock_skew_sim_s=.75, pose_wall_age_s=2.,
    carry_slip_m=.005, carry_angle_deg=10.,
    snapshot_joint_rad=.003, snapshot_pose_m=.002,
    snapshot_angle_deg=1., snapshot_finger_m=.001,
    coherent_native_lag_sim_s=.05, coherent_joint_bracket_sim_s=.012,
    trace_gzip_level=1,
    retrieval_commanded_tilt_deg=5.,
    retrieval_pilot_rate_rad_s=.1,
    empty_button_rate_cap_rad_s=.8,
)


def motion_limits(phase, loaded):
    near = any(token in phase for token in (
        'descend', 'lower', 'pilot', 'vertical_retreat', 'setdown', 'axis_exit'))
    if 'pilot' in phase and phase.startswith('seq_retrieval_'):
        rate = PROFILE['retrieval_pilot_rate_rad_s']
    elif 'pilot' in phase or 'setdown_fine' in phase or 'axis_exit' in phase:
        rate = PROFILE['pilot_fine_rate_rad_s']
    elif near:
        rate = PROFILE['near_rate_rad_s']
    else:
        rate = PROFILE['loaded_rate_rad_s' if loaded else 'empty_rate_rad_s']
    return rate, PROFILE['tracking_near_rad' if near else 'tracking_free_rad']


def common_timing(actual, samples, rate, ramp_s=.15):
    if not samples or not math.isfinite(rate) or not 0 < rate <= PROFILE['empty_rate_rad_s']:
        raise ValueError('Invalid speed-priority simulation rate')
    if not math.isfinite(ramp_s) or ramp_s < .1:
        raise ValueError('Finite acceleration ramp required')
    names = frozenset(samples[0])
    if names != TASK_JOINTS or not names <= actual.keys():
        raise ValueError('Exactly torso and fourteen arm joints required')
    points = [{name: float(actual[name]) for name in samples[0]}] + [dict(sample) for sample in samples]
    if any(frozenset(point) != names or not all(math.isfinite(value) for value in point.values()) for point in points):
        raise ValueError('Incomplete or nonfinite path')
    lengths = [max(abs(second[name] - first[name]) for name in names)
               for first, second in zip(points, points[1:])]
    total = sum(lengths)
    acceleration = rate / ramp_s
    peak = min(rate, math.sqrt(total * acceleration))
    ramp_distance = peak * peak / (2 * acceleration)
    total_time = 2 * peak / acceleration + max(0., total - 2 * ramp_distance) / max(peak, 1e-15)
    def at_distance(distance):
        if distance <= ramp_distance:
            return math.sqrt(2 * distance / acceleration)
        if distance >= total - ramp_distance:
            return total_time - math.sqrt(max(0., 2 * (total - distance) / acceleration))
        return peak / acceleration + (distance - ramp_distance) / peak
    elapsed = PROFILE['first_point_sim_s']
    times = [elapsed]
    distance = 0.
    for length in lengths:
        previous = at_distance(distance)
        distance += length
        elapsed += max(PROFILE['point_floor_sim_s'], length / rate, at_distance(distance) - previous)
        times.append(elapsed)
    return dict(times=times, points=points, duration_sim_s=elapsed,
                groups={'torso': ['torso_joint'], 'left_arm': ARMS['left'], 'right_arm': ARMS['right']},
                single_common_clock=True, point_floor_sim_s=PROFILE['point_floor_sim_s'],
                original_floor_sim_s=.020, acceleration_ramp_sim_s=ramp_s)


def dwell_duration(phase, requested):
    if requested == 10 and 'static_grip_hold' in phase:
        return PROFILE['static_grip_hold_sim_s']
    return requested


def configure_simulation_profile():
    import grip_limits
    grip_limits.CARRY_SLIP_LIMIT_M = PROFILE['carry_slip_m']
    grip_limits.CARRY_ANGLE_LIMIT_DEG = PROFILE['carry_angle_deg']


def gripper_velocity_violation(value):
    if not math.isfinite(value):
        return 'nonfinite'
    if abs(value) > PROFILE['gripper_velocity_monitor_m_s'] + 1e-8:
        return 'exceeds_relaxed_simulation_0.6_m_s'
    return None

