import pytest

from audit_dual_speed import measured_duration


def test_exact_three_minute_complete_measured_cycle_is_accepted():
    assert measured_duration(23., 203., True, True)['status'] == 'PASS'


def test_faster_partial_task_is_not_complete():
    assert measured_duration(23., 120., False, True)['status'] == 'FAIL'
    assert measured_duration(23., 120., True, False)['status'] == 'FAIL'


def test_full_task_over_three_minutes_is_still_not_target_pass():
    result = measured_duration(23., 204., True, True)
    assert result['complete'] is True
    assert result['status'] == 'FAIL'
    assert result['under_three_minutes_sim'] is False


@pytest.mark.parametrize('first, final', [(None, 180.), (0., None), (23., 23.), (23., 22.)])
def test_missing_or_invalid_actual_timestamps_cannot_pass(first, final):
    assert measured_duration(first, final, True, True)['status'] == 'FAIL'
