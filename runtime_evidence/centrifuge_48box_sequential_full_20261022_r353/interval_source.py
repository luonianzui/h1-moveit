ENDPOINT_OBSERVATION = """                    self.record('interval_endpoint_pass', dict(
                        motion_start_sim=start, scheduled_end_sim=start + timing['duration_sim_s'],
                        actual_joint_stamp_sim=stamp, measured_stable_start_sim=stable,
                        measured_hold_sim_s=stamp - stable,
                        required_hold_sim_s=PROFILE['endpoint_hold_sim_s'],
                        original_endpoint_limit_rad=PROFILE['endpoint_rad'],
                        overlap_enabled=self.paired_feedback_overlap_enabled()))
"""


def restore_interval_observation(source):
    if source.count(ENDPOINT_OBSERVATION) != 1:
        raise ValueError('Exact declared endpoint observation required')
    return source.replace(ENDPOINT_OBSERVATION, '', 1)
