import math

import pytest

from fast_profile import calibrated_rate, validate_tracking


@pytest.mark.parametrize('phase', ['left_park', 'right_approach', 'left_retreat', 'right_return', 'machine_return_neutral_left'])
def test_empty_rate_uses_actual_calibrated_limit(phase):
    assert calibrated_rate(phase, None, None) == .8


@pytest.mark.parametrize('holder,payload', [('left', None), ('right', None), ('left', 'right'), ('right', 'left')])
def test_loading_and_retrieval_loaded_rates_are_below_empty_calibration(holder, payload):
    assert calibrated_rate(holder + '_transfer', holder, payload) == .25
    assert calibrated_rate(holder + '_retrieval_transfer', holder, payload) == .25
    assert calibrated_rate(holder + '_retrieval_pilot_lift', holder, payload) == .03
    assert calibrated_rate(holder + '_pilot_lift', holder, payload) == .06
    assert calibrated_rate(holder + '_retrieval_setdown', holder, payload) == .20
    assert calibrated_rate(holder + '_release_alignment_correction', holder, payload) == .05
    assert calibrated_rate('right_lower', 'right', None) == .065
    assert calibrated_rate('left_lower', 'left', None) == .13


@pytest.mark.parametrize('error', [.015001, math.nan, math.inf, -math.inf])
def test_tracking_threshold_is_not_relaxed(error):
    with pytest.raises(ValueError):
        validate_tracking({'joint':error})


def test_guard_keeps_exact_original_limit():
    validate_tracking({'first':.015, 'second':-.015})
    with pytest.raises(ValueError):
        calibrated_rate('transfer', 'unknown', 'left')


@pytest.mark.parametrize('phase,holder,payload', [('right_lower', None, None),
    ('right_lower', 'left', None), ('left_lower', 'unknown', None), ('left_lower', 'left', 'right')])
def test_loading_descent_cannot_retime_wrong_or_unknown_holder(phase, holder, payload):
    with pytest.raises(ValueError, match='matching holder'):
        calibrated_rate(phase, holder, payload)


@pytest.mark.parametrize('holder', ['left', 'right'])
def test_loading_alignment_keeps_explicit_original_contact_cap(holder):
    assert calibrated_rate(holder + '_align', holder, None) == .10
    assert calibrated_rate(holder + '_transfer', holder, None) == .25


@pytest.mark.parametrize('phase,holder,payload', [('right_align', None, None),
    ('right_align', 'left', None), ('left_align', 'unknown', None), ('left_align', 'left', 'right')])
def test_loading_alignment_cannot_accept_wrong_or_unknown_actual_holder(phase, holder, payload):
    with pytest.raises(ValueError, match='matching holder'):
        calibrated_rate(phase, holder, payload)
