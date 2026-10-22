import math


def production_empty_approach_limits(cycle, requested_rate, inherited_limits):
    cap = cycle.elbow_empty_approach_cap
    if not math.isfinite(cap) or cap != .5 or cycle.holders.rows or requested_rate != .8:
        raise ValueError('Only the declared empty elbow approach may use the fixed 0.5rad/s cap')
    inherited = inherited_limits('paired_load_approach', requested_rate)
    if inherited[0] != .7:
        raise ValueError('The original 0.7rad/s empty approach contract changed')
    cycle.record('elbow_empty_approach_velocity_qualification', dict(
        parent_actual_rate_rad_s=.7, actual_rate_rad_s=.5,
        prior_fault_m_s=.20197374630967288, unchanged_native_finger_velocity_limit_m_s=.2,
        unchanged_tracking_limit_rad=inherited[1], all_original_joint_samples_common_clock_floor_preserved=True,
        no_force_pid_or_collision_guard_change=True, new_actual_physical_qualification_required=True))
    return .5, inherited[1]
