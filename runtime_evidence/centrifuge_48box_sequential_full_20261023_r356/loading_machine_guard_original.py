import math


def machine_ready_for_loading(machine, phase, current_sim, drop_window_until=None):
    rotor_rpm = machine['rotor_rpm']
    lid_rad = machine['lid_rad']
    if not machine['ready'] or machine['rotor_enabled']:
        return False
    if not math.isfinite(rotor_rpm) or not math.isfinite(lid_rad):
        return False
    if abs(lid_rad - math.radians(120)) > .005:
        return False
    passive_settling = (drop_window_until is not None and current_sim <= drop_window_until
                        and phase.endswith(('_release_above_bucket', '_drop_settling')))
    return abs(rotor_rpm) <= (2.0 if passive_settling else .02)

