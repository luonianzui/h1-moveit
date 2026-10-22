import math

import pytest

from stop_phase import schedule_nominal_phase


@pytest.mark.parametrize('phase', [-3., -1., 0., 1., 3.])
def test_physical_stop_schedule_waits_at_least_thirty_seconds_and_no_more_than_one_extra_turn(phase):
    result = schedule_nominal_phase(10., 11., phase, math.pi, .9)
    assert 30.05 <= result['estimated_steady_duration_sim_s'] <= 32.05
    trigger = result['estimated_physical_stop_trigger_sim']
    angle = phase + math.pi * (trigger - 11.)
    assert abs(math.remainder(angle, 2 * math.pi)) < 1e-12
    assert trigger - result['stroke_start_sim'] == pytest.approx(.9)
    assert result['estimate_not_actual_encoder_stop_certificate'] is True


@pytest.mark.parametrize('speed,offset,minimum', [(2., .9, 30.), (math.pi, 0., 30.),
                                                (math.pi, .9, 29.9), (float('nan'), .9, 30.)])
def test_unsteady_speed_unknown_contact_offset_or_shortened_spin_is_rejected(speed, offset, minimum):
    with pytest.raises(ValueError):
        schedule_nominal_phase(10., 11., .1, speed, offset, minimum)
