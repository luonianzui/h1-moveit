import math
from pathlib import Path

import pytest

from leg_initialization_contract import LEG_JOINTS, targets_from_original, endpoint_error


def test_original_targets_not_measured_rebase():
    original = dict.fromkeys(LEG_JOINTS, .09)
    measured = dict.fromkeys(LEG_JOINTS, .01)
    assert targets_from_original(original, measured) == ([.09] * 12, [.01] * 12)
    assert endpoint_error(original, measured) == .08


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -.151, .151])
def test_invalid_initialization_envelope_rejected(invalid):
    original = dict.fromkeys(LEG_JOINTS, 0.)
    measured = dict(original, left_knee_joint=invalid)
    with pytest.raises(ValueError):
        targets_from_original(original, measured)


def test_missing_feedback_rejected():
    with pytest.raises(ValueError):
        targets_from_original(dict.fromkeys(LEG_JOINTS, 0.), {})


def test_only_leg_action_and_original_endpoint():
    source = Path(__file__).with_name('initialize_stationary_legs.py').read_text()
    assert "goal.trajectory.joint_names = list(LEG_JOINTS)" in source
    assert 'position=.005, velocity=.03' in source
    assert 'targets_from_original(initial, actual)' in source
    assert "goal.goal_time_tolerance.sec = 10" in source
    assert 'hold_peak_error_rad=peak' in source
    assert 'set_entity_pose' not in source and 'WorldControl' not in source
