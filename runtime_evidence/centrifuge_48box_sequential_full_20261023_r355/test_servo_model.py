import copy
import math

import pytest

from servo_model import predicted_ramp_error, verify_rate_fork


def test_expected_ramp_lag_reduced_without_changing_guard():
    assert predicted_ramp_error(.8, 100., .2) == pytest.approx(.04)
    assert predicted_ramp_error(.8, 500., .2) == pytest.approx(.008)
    assert predicted_ramp_error(.8, 500., .2) < .015


@pytest.mark.parametrize('parameters', [(.8, 0., .2), (.8, 500., 0.), (.8, 500., 2.),
                                       (.8, 500., -1.), (math.nan, 500., .2),
                                       (.8, math.inf, .2), (-.8, 500., .2)])
def test_invalid_or_unstable_model_refused(parameters):
    with pytest.raises(ValueError):
        predicted_ramp_error(*parameters)


def test_only_manager_frequency_change_is_certified():
    original = dict(controller_manager=dict(ros__parameters=dict(update_rate=100)),
                    left_gripper_controller=dict(gains=dict(force=16)))
    candidate = copy.deepcopy(original)
    candidate['controller_manager']['ros__parameters']['update_rate'] = 500
    assert verify_rate_fork(original, candidate)
    candidate['left_gripper_controller']['gains']['force'] = 20
    with pytest.raises(ValueError):
        verify_rate_fork(original, candidate)
from servo_model import verify_manager_rate_output


@pytest.mark.parametrize('output,code', [('Integer value is: 100', 0), ('Integer value is: 500', 1),
    ('String value is: 500', 0), ('Integer value is: 500\nInteger value is: 100', 0), ('', 0)])
def test_actual_manager_rate_must_match_exactly(output, code):
    with pytest.raises(ValueError):
        verify_manager_rate_output(output, code)


def test_actual_manager_500_parameter_passes():
    assert verify_manager_rate_output('Integer value is: 500\n', 0) == 500

