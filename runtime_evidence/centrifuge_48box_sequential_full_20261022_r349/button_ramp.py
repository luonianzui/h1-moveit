import math

import numpy as np


def require_spline_output(output, returncode):
    if returncode != 0 or output.strip() != 'String value is: splines':
        raise ValueError('Actual arm controller spline interpolation readback is required')
    return True


def ramp_button_positions(points, maximum_rate=1.2, ramp_s=.1, minimum_interval=.020):
    if not all(math.isfinite(value) and value > 0 for value in (maximum_rate, ramp_s, minimum_interval)):
        raise ValueError('Finite positive ramp parameters required')
    if maximum_rate > 1.2 or minimum_interval != .020 or ramp_s < .1:
        raise ValueError('Original twenty-ms floor and bounded ramp profile required')
    positions = [np.asarray(point, dtype=float) for point in points]
    if not positions or any(point.shape != (7,) or not np.isfinite(point).all() for point in positions):
        raise ValueError('Complete finite seven-joint original button points required')
    elapsed = .05
    records = [dict(time_s=elapsed, positions=positions[0].tolist(), velocities=[0.] * 7)]
    vertices = [0]
    maximum_acceleration = maximum_rate / ramp_s
    for previous, target in zip(positions, positions[1:]):
        delta = target - previous
        distance = float(np.max(np.abs(delta)))
        if distance == 0:
            elapsed += minimum_interval
            records.append(dict(time_s=elapsed, positions=target.tolist(), velocities=[0.] * 7))
            vertices.append(len(records) - 1)
            continue
        if distance >= maximum_rate * ramp_s:
            acceleration_time = ramp_s
            cruise_time = distance / maximum_rate - acceleration_time
            if cruise_time < minimum_interval:
                acceleration_time = distance / maximum_rate
                cruise_time = 0.
            peak = maximum_rate
        else:
            acceleration_time = max(math.sqrt(distance / maximum_acceleration), minimum_interval)
            cruise_time = 0.
            peak = distance / acceleration_time
        acceleration = peak / acceleration_time
        duration = 2 * acceleration_time + cruise_time
        boundaries = sorted(set((0., acceleration_time, acceleration_time + cruise_time, duration)))
        for begin, end in zip(boundaries, boundaries[1:]):
            intervals = max(1, math.floor((end - begin) / minimum_interval + 1e-12))
            for index in range(1, intervals + 1):
                local_time = begin + (end - begin) * index / intervals
                if local_time <= acceleration_time:
                    travel = .5 * acceleration * local_time ** 2
                    speed = acceleration * local_time
                elif local_time <= acceleration_time + cruise_time:
                    travel = .5 * acceleration * acceleration_time ** 2 + peak * (local_time - acceleration_time)
                    speed = peak
                else:
                    remaining = duration - local_time
                    travel = distance - .5 * acceleration * remaining ** 2
                    speed = acceleration * remaining
                final = index == intervals and end == duration
                records.append(dict(time_s=elapsed + local_time,
                                    positions=target.tolist() if final else (previous + delta * travel / distance).tolist(),
                                    velocities=[0.] * 7 if final else (delta * speed / distance).tolist()))
        elapsed += duration
        vertices.append(len(records) - 1)
    return dict(points=records, original_vertex_indices=vertices, maximum_rate_rad_s=maximum_rate,
                maximum_acceleration_rad_s2=maximum_acceleration, ramp_time_sim_s=ramp_s,
                minimum_interval_sim_s=minimum_interval, original_joint_polyline_preserved=True,
                all_original_vertices_commanded_zero_velocity=True)


BUTTON_POINT_HOOK = '''    def button_points_before_publish(self, points, fixed_interval):
        return points

'''
BUTTON_REBUILD = '''        message.points = self.button_points_before_publish(message.points, fixed_interval)
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        elapsed = times[-1]
        tracking_points = [list(point.positions) for point in message.points]
'''
INTERPOLATION_GATE = '''    from button_ramp import require_spline_output
    interpolation = json.loads((EVIDENCE / 'run/button_interpolation_readback.json').read_text())
    assert interpolation['read_only_no_parameter_set'] and {row['side'] for row in interpolation['attempts']} == {'left', 'right'}
    for attempt in interpolation['attempts']:
        require_spline_output(attempt['stdout'], attempt['command_returncode'])
'''


def restore_button_ramp_source(name, source):
    if name != 'execute_fast.py':
        return source
    if source.count(BUTTON_POINT_HOOK) != 1 or source.count(BUTTON_REBUILD) != 1:
        raise ValueError('Only the declared default-preserving button publication hook can be reversed')
    source = source.replace(BUTTON_POINT_HOOK, '', 1).replace(BUTTON_REBUILD, '', 1)
    source = source.replace('[values[index] for values in tracking_points]', '[values[index] for values in points]')
    source = source.replace('dict(positions=list(point.positions), velocities=list(point.velocities), seconds=seconds)',
                            'dict(positions=list(point.positions), seconds=seconds)')
    return source
