import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from centrifuge_button import rotor_request, toggle_target


def decide(**overrides):
    parameters = dict(
        position=0.0, velocity=0.0, age=0.05, busy=False, fault=None,
        elapsed=1.0, lower=0.0, upper=math.radians(120), writers=0)
    parameters.update(overrides)
    return toggle_target(**parameters)


def test_first_press_opens():
    assert decide() == 'open'


def test_second_press_closes_from_feedback():
    assert decide(position=math.radians(120)) == 'close'


def test_restart_uses_measured_state():
    assert decide(position=math.radians(100)) == 'close'
    assert decide(position=math.radians(20)) == 'open'


@pytest.mark.parametrize('overrides,reason', [
    ({'busy': True}, 'BUSY'),
    ({'fault': 'error'}, 'FAULT_LATCHED'),
    ({'elapsed': 0.49}, 'DEBOUNCE'),
    ({'age': 0.51}, 'STALE_FEEDBACK'),
    ({'age': -0.1}, 'STALE_FEEDBACK'),
    ({'position': math.nan}, 'INVALID_FEEDBACK'),
    ({'velocity': math.inf}, 'INVALID_FEEDBACK'),
    ({'position': -0.006}, 'OUTSIDE_ENVELOPE'),
    ({'position': math.radians(121)}, 'OUTSIDE_ENVELOPE'),
    ({'velocity': 0.01}, 'LID_ALREADY_MOVING'),
    ({'writers': 1}, 'EXTERNAL_COMMAND'),
])
def test_unsafe_press_rejected(overrides, reason):
    with pytest.raises(RuntimeError, match=reason):
        decide(**overrides)


def test_rotor_start_and_stop():
    closed = dict(ready=True, rotor_rad_s=0.0, rotor_enabled=False, can_start=True)
    assert rotor_request(closed, .1, False, None)
    assert not rotor_request(dict(closed, rotor_enabled=True), .1, False, None)
    assert not rotor_request(dict(closed, rotor_rad_s=1.0), .1, False, None)


@pytest.mark.parametrize('changes,age,busy,fault', [
    ({'can_start': False}, .1, False, None),
    ({'ready': False}, .1, False, None),
    ({}, .6, False, None),
    ({}, -.1, False, None),
    ({}, math.nan, False, None),
    ({'rotor_rad_s': math.nan}, .1, False, None),
    ({}, .1, True, None),
    ({}, .1, False, 'error'),
])
def test_rotor_start_rejected(changes, age, busy, fault):
    machine = dict(ready=True, rotor_rad_s=0.0, rotor_enabled=False, can_start=True)
    machine.update(changes)
    with pytest.raises(RuntimeError):
        rotor_request(machine, age, busy, fault)
