import pytest

from startup_window import validate_startup_window


@pytest.mark.parametrize('sim', [0, 14.4, 24.9995, 25, 44.9995, 45, 49.1335, 90., 299.9995, 300.])
def test_fresh_measured_clock_is_accepted_without_relaxing_physical_limits(sim):
    result = validate_startup_window(sim)
    assert result['status'] == 'PASS'
    assert result['measured_sim_s'] == sim
    assert result['original_leg_targets_unchanged']
    assert result['early_start_hypothesis_not_physical_success']


@pytest.mark.parametrize('sim', [None, True, False, -0.0005, 300.0005, 500., float('nan'), float('inf')])
def test_missing_nonfinite_or_aged_start_is_rejected(sim):
    with pytest.raises(ValueError):
        validate_startup_window(sim)
