import math


def schedule_nominal_phase(steady_start, pose_stamp, rotor_yaw, speed, observed_press_offset, minimum_steady_seconds=30.):
    values = (steady_start, pose_stamp, rotor_yaw, speed, observed_press_offset, minimum_steady_seconds)
    if not all(math.isfinite(value) for value in values) or steady_start < 0 or pose_stamp < 0:
        raise ValueError('Finite actual encoder timing is required')
    if minimum_steady_seconds < 30 or not 29.8 <= speed * 60 / (2 * math.pi) <= 30.2:
        raise ValueError('Cannot shorten the 30sim steady certificate or use unsteady speed')
    if not 0 < observed_press_offset < 2:
        raise ValueError('Actual physical start-button offset is required')
    earliest = max(steady_start + minimum_steady_seconds + .05, pose_stamp + observed_press_offset + .5)
    phase_at_earliest = rotor_yaw + speed * (earliest - pose_stamp)
    extra = (-phase_at_earliest) % (2 * math.pi) / speed
    trigger = earliest + extra
    return dict(estimated_physical_stop_trigger_sim=trigger, stroke_start_sim=trigger - observed_press_offset,
                estimated_steady_duration_sim_s=trigger - steady_start, extra_phase_wait_sim_s=extra,
                actual_native_pose_stamp=pose_stamp, actual_native_rotor_yaw=rotor_yaw,
                measured_speed_rad_s=speed, measured_physical_button_offset_sim_s=observed_press_offset,
                target_rotor_phase_mod_2pi=0., estimate_not_actual_encoder_stop_certificate=True,
                no_angle_command_pose_reset_or_artificial_attachment=True)
