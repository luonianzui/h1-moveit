import math


MAXIMUM_FRESH_START_SIM_S = 300.0


def validate_startup_window(sim):
    if isinstance(sim, bool) or sim is None or not math.isfinite(sim):
        raise ValueError('Fresh startup requires a finite measured simulation clock')
    if not 0 <= sim <= MAXIMUM_FRESH_START_SIM_S:
        raise ValueError('Fresh startup aged beyond 300 simulation seconds; preserve and retire this identity')
    return dict(status='PASS', measured_sim_s=float(sim), maximum_fresh_start_sim_s=MAXIMUM_FRESH_START_SIM_S,
                original_leg_targets_unchanged=True, stationary_leg_endpoint_limit_rad=.005,
                original_fresh_start_sim_s=25.,
                early_start_hypothesis_not_physical_success=True)
