import math


RETRIEVAL_MICROMOTION_RPM = .1


def retrieval_ready_for_demo(machine):
    if not machine.get('ready') or machine.get('rotor_enabled') or machine.get('open_lid_demo'):
        return False
    fields = ('lid_rad', 'rotor_rpm', 'rotor_rad_s', 'motor_torque_nm')
    if not all(math.isfinite(machine.get(key, math.nan)) for key in fields):
        return False
    return (machine.get('brake_active') is True and
            machine.get('stopped_velocity_gain') == 80 and machine.get('stopped_position_gain') == 8000 and
            abs(machine['motor_torque_nm']) <= 2.000001 and
            abs(machine['lid_rad'] - math.radians(120)) <= .005 and
            abs(machine['rotor_rpm']) <= RETRIEVAL_MICROMOTION_RPM and
            abs(machine['rotor_rad_s']) <= RETRIEVAL_MICROMOTION_RPM * 2 * math.pi / 60 + 1e-8)


def machine_ready_for_cycle(machine, phase, current_sim, drop_window_until=None):
    from loading_machine_guard_original import machine_ready_for_loading
    if '_retrieval_' in phase:
        return retrieval_ready_for_demo(machine)
    if not phase.startswith('machine_'):
        return machine_ready_for_loading(machine, phase, current_sim, drop_window_until)
    if not machine.get('ready') or machine.get('open_lid_demo'):
        return False
    if not all(math.isfinite(machine.get(key, math.nan)) for key in ('lid_rad', 'rotor_rpm', 'rotor_rad_s')):
        return False
    if abs(machine['rotor_rpm']) > 30.3:
        return False
    if machine['rotor_enabled'] or abs(machine['rotor_rad_s']) >= .02:
        return phase.startswith(('machine_start', 'machine_running', 'machine_stop')) and abs(machine['lid_rad']) < math.radians(.2)
    return -.005 <= machine['lid_rad'] <= math.radians(120) + .005

