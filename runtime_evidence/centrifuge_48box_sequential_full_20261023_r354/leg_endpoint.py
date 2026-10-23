import math

from leg_posture import LEG_JOINTS


WAIT_SOURCE = '''        endpoint = LegEndpointGate(targets, self.joint_time, self.sim)
        self.record('leg_endpoint_fresh_wait_declared', dict(previous_joint_stamp=self.joint_time,
                    original_position_limit_rad=.001, maximum_wait_sim_s=3., maximum_wait_wall_s=30.))
        self.wait(lambda: endpoint.observe(self.joint_time, self.sim, self.positions), timeout_s=30)
        self.record('leg_endpoint_fresh_wait_pass', dict(actual_joint_stamp=self.joint_time,
                    maximum_actual_error_rad=endpoint.maximum_error_rad,
                    original_position_limit_rad=.001, new_actual_feedback_required=True))
'''


class LegEndpointGate:
    def __init__(self, targets, previous_stamp, start_sim, position_limit_rad=.001):
        if position_limit_rad not in (.001, .002, .005):
            raise ValueError('Only original or declared stationary leg endpoint limit allowed')
        self.position_limit_rad = position_limit_rad
        if len(targets) != len(LEG_JOINTS) or not all(math.isfinite(value) for value in targets):
            raise ValueError('All original finite leg targets are required')
        if not all(math.isfinite(value) and value >= 0 for value in (previous_stamp, start_sim)):
            raise ValueError('Actual finite nonnegative feedback and clock stamps are required')
        self.targets = dict(zip(LEG_JOINTS, targets))
        self.previous_stamp = previous_stamp
        self.start_sim = start_sim
        self.maximum_error_rad = None

    def observe(self, stamp, current_sim, actual):
        if not all(math.isfinite(value) for value in (stamp, current_sim)):
            raise ValueError('Actual feedback and clock stamps must remain finite')
        if current_sim - self.start_sim > 3.:
            raise TimeoutError('Original leg endpoint did not qualify within three actual sim seconds')
        if not all(name in actual and math.isfinite(actual[name]) for name in LEG_JOINTS):
            raise ValueError('All twelve actual native leg positions are required')
        self.maximum_error_rad = max(abs(actual[name] - target) for name, target in self.targets.items())
        return stamp > self.previous_stamp and self.maximum_error_rad < self.position_limit_rad


def restore_leg_endpoint_source(source):
    import_line = 'from leg_endpoint import LegEndpointGate\n'
    if source.count(WAIT_SOURCE) != 1 or source.count(import_line) != 1:
        raise ValueError('Only the declared fresh-feedback wait may be reversed')
    return source.replace(WAIT_SOURCE, '', 1).replace(import_line, '', 1)
