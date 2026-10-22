import inspect
import math

import pytest

from cycle_guard import machine_ready_for_cycle
from execute_cycle import FullCycle


def machine(rpm=.05, **changes):
    result = dict(ready=True, rotor_enabled=False, open_lid_demo=False, lid_rad=math.radians(120),
                  rotor_rpm=rpm, rotor_rad_s=rpm * 2 * math.pi / 60, motor_torque_nm=.2,
                  brake_active=True, stopped_velocity_gain=80, stopped_position_gain=8000)
    result.update(changes)
    return result


@pytest.mark.parametrize('phase', ['left_retrieval_descend', 'right_retrieval_pilot_lift',
                                    'left_retrieval_lift', 'right_retrieval_setdown'])
@pytest.mark.parametrize('rpm', [-.1, -.05, .020560172597269843, .05, .1])
def test_only_authorized_retrieval_transient_micro_motion_is_allowed(phase, rpm):
    assert machine_ready_for_cycle(machine(rpm), phase, 0)
    assert not machine_ready_for_cycle(machine(rpm), 'preflight', 0)
    assert not machine_ready_for_cycle(machine(rpm), 'terminal_table_hold', 0)


@pytest.mark.parametrize('changes', [dict(rotor_enabled=True), dict(brake_active=False),
    dict(motor_torque_nm=2.001), dict(lid_rad=0), dict(stopped_velocity_gain=0),
    dict(stopped_position_gain=0), dict(rotor_rpm=.10001), dict(rotor_rad_s=.1),
    dict(rotor_rpm=math.nan), dict(lid_rad=math.inf), dict(open_lid_demo=True)])
def test_motor_brake_lid_and_transient_cap_still_fail_closed(changes):
    assert not machine_ready_for_cycle(machine(**changes), 'left_retrieval_pilot_lift', 0)


def test_user_permission_is_declared_without_fake_original_strict_pass():
    source = inspect.getsource(FullCycle.run_complete)
    assert "simulation_relaxation_declared" in source
    assert '可放宽限制，先完成全套流程再优化' in source
    assert 'original_strict_certificate_not_claimed=True' in source
    assert 'loaded_paths_holds_physics_and_gates_unchanged=False' in source
