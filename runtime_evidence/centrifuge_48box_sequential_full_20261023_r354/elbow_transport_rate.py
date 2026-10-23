import math

from complete_rate_profile import require_current_pair


RETRIEVAL_TRANSPORT_PHASES = frozenset(('paired_retrieval_outboard', 'paired_retrieval_transfer'))


def production_retrieval_transport_rate(cycle, phase, inherited_rate):
    cap = cycle.elbow_retrieval_transport_cap
    if phase not in RETRIEVAL_TRANSPORT_PHASES or not math.isfinite(cap) or cap != .3 or inherited_rate != .6:
        raise ValueError('Only the declared elbow retrieval transport may use the fixed 0.3rad/s cap')
    require_current_pair(cycle)
    cycle.record('elbow_retrieval_transport_effort_qualification', dict(
        phase=phase, parent_actual_rate_rad_s=.6, actual_rate_rad_s=.3,
        prior_peak_output_effort_N=18.03919692455446, unchanged_output_guard_N=18.01,
        unchanged_retrieval_feedforward_N=18., unchanged_native_slip_m=.002,
        unchanged_native_angle_deg=5., all_original_joint_samples_common_clock_floor_preserved=True,
        no_force_pid_or_collision_guard_change=True, new_actual_physical_qualification_required=True))
    return .3
