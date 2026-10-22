import math


def localization_freshness(sim, source_sim, now, pose_wall, status_wall, status_valid):
    if source_sim is None or sim is None:
        return dict(valid=False, reason='missing_stamped_localization_pose', pose_age_sim_s=None,
                    pose_age_wall_s=None, status_age_wall_s=None)
    values = (sim, source_sim, now, pose_wall, status_wall)
    if not all(isinstance(value, (float, int)) and math.isfinite(value) for value in values):
        return dict(valid=False, reason='nonfinite_localization_time', pose_age_sim_s=None,
                    pose_age_wall_s=None, status_age_wall_s=None)
    sim_age = sim - source_sim
    pose_age = now - pose_wall
    status_age = now - status_wall
    valid = (status_valid is True and -.15 <= sim_age <= 1.0 and
             0 <= pose_age <= 60.0 and 0 <= status_age <= 60.0)
    return dict(valid=valid, reason='pass' if valid else 'stale_or_invalid_localization',
                pose_age_sim_s=sim_age, pose_age_wall_s=pose_age, status_age_wall_s=status_age,
                maximum_pose_age_sim_s=1.0, maximum_wall_age_s=60.0,
                source_sim=source_sim, clock_sim=sim)

