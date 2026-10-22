import math

from speed_policy import PROFILE


def select_coherent_pair(frames, timeline, latest_native_stamp):
    if not math.isfinite(latest_native_stamp) or latest_native_stamp < 0:
        raise ValueError('Finite actual native timestamp required')
    if not 0 < timeline.maximum_gap_s <= PROFILE['coherent_joint_bracket_sim_s']:
        raise ValueError('Original actual-joint interpolation gap must remain bounded')
    for stamp, wall, poses in reversed(list(frames)):
        if stamp > latest_native_stamp:
            continue
        lag = latest_native_stamp - stamp
        if lag > PROFILE['coherent_native_lag_sim_s'] + 1e-9:
            break
        measured = timeline.at(stamp)
        if measured is not None:
            return dict(pose_stamp=stamp, pose_wall=wall, poses=poses,
                        joints=measured, native_lag_s=lag)
    return None
