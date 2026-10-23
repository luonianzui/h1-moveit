import math

import pytest

from sequential_speed_cycle import SpeedPriorityCycle
from test_complete_rate_profile import button_cycle


def cycle_with_tracking_error(error):
    cycle = button_cycle(False)
    cycle.__class__ = SpeedPriorityCycle
    original_spin = cycle.spin
    def spin():
        original_spin()
        messages = cycle.publishers['right_arm'].messages
        if messages:
            message = messages[-1]
            start = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            end = message.points[-1].time_from_start.sec + message.points[-1].time_from_start.nanosec * 1e-9
            if start + .06 < cycle.sim < start + end - .01:
                cycle.positions['right_elbow_joint'] += error
    cycle.spin = spin
    return cycle


@pytest.mark.parametrize('error', [.018800003273902022, .08359999991238742, -.08359999991238742])
def test_old_observed_button_error_accepted_with_original_endpoint(error):
    cycle = cycle_with_tracking_error(error)
    cycle.move_button('right', [[0.] * 7, [.12] * 7, [.24] * 7])
    assert cycle.motion_count == 1
    assert not any(kind.endswith('_fault') for kind, payload in cycle.events)
    assert any(kind == 'fast_button_tracking_sample' and payload['current_limit_rad'] == .15
               for kind, payload in cycle.events)
    assert all(abs(cycle.positions[name] - .24) < .005 for name in cycle.buttons['arms']['right']['joints'])


@pytest.mark.parametrize('error', [.151, -.151, math.nan, math.inf])
def test_new_button_limit_is_bounded_and_failure_is_recorded(error):
    cycle = cycle_with_tracking_error(error)
    with pytest.raises(ValueError, match='relaxed simulation limit'):
        cycle.move_button('right', [[0.] * 7, [.12] * 7, [.24] * 7])
    assert any(kind == 'speed_button_tracking_fault' for kind, payload in cycle.events)
