import json

import pytest

from leg_endpoint import LegEndpointGate
from leg_initialization_contract import LEG_JOINTS
from leg_initialization_runtime import verify_initialization
from test_acceleration_integration import initialization_fixture


def test_observed_full_scene_residual_is_valid_only_inside_new_stationary_limit(tmp_path):
    initial, report = initialization_fixture(tmp_path)
    actual = {name: value + .0023 for name, value in initial.items()}
    report.update(after=actual, hold_peak_error_rad=.0023)
    (tmp_path / 'run/leg_initialization.json').write_text(json.dumps(report))
    assert verify_initialization(tmp_path, initial, actual) == report


@pytest.mark.parametrize('offset', [.005, .006, float('inf')])
def test_report_endpoint_is_independently_checked_even_if_peak_claim_is_small(tmp_path, offset):
    initial, report = initialization_fixture(tmp_path)
    report['after'] = {name: value + offset for name, value in initial.items()}
    (tmp_path / 'run/leg_initialization.json').write_text(json.dumps(report))
    with pytest.raises(ValueError):
        verify_initialization(tmp_path, initial, initial)


def test_stationary_five_mrad_does_not_change_original_endpoint_default():
    actual = dict.fromkeys(LEG_JOINTS, .0023)
    assert not LegEndpointGate([0.] * 12, 1., 1.).observe(1.1, 1.1, actual)
    assert LegEndpointGate([0.] * 12, 1., 1., .005).observe(1.1, 1.1, actual)
    assert not LegEndpointGate([0.] * 12, 1., 1., .005).observe(1.1, 1.1, dict.fromkeys(LEG_JOINTS, .005))
