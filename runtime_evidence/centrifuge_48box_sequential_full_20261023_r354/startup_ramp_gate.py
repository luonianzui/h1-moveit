import math


def verify_startup_status(value):
    if not isinstance(value, dict):
        raise ValueError('Current native machine status required')
    numeric = dict(startup_ramp_rad_s2=.8, stopped_velocity_gain=80, stopped_position_gain=8000,
                   starts=0, stops=0)
    for name, expected in numeric.items():
        actual = value.get(name)
        if isinstance(actual, bool) or not isinstance(actual, (int, float)) or not math.isfinite(actual) or actual != expected:
            raise ValueError('Wrong current startup field: ' + name)
    if value.get('ready') is not True or value.get('rotor_enabled') is not False or value.get('can_start') is not True:
        raise ValueError('Fresh ready stopped closed machine required')
    for name, limit in (('lid_rad', .003490658503988659), ('rotor_rpm', .02)):
        actual = value.get(name)
        if isinstance(actual, bool) or not isinstance(actual, (int, float)) or not math.isfinite(actual) or abs(actual) >= limit:
            raise ValueError('Fresh initial machine bound: ' + name)
    return dict(status='PASS', declared_startup_ramp_rad_s2=.8, original_stop_and_position_gains_preserved=True,
                no_machine_command_published=True, actual_startup_motion_not_certified=True)
