"""Tests for the V12 cold-start motion arming state."""

from h1_2_navigation.motion_arming import MotionArming
import numpy as np
import pytest


def test_exact_zero_stays_in_standby_until_motion_starts():
    """Exact zero holds until movement and cannot re-enter standby."""
    arming = MotionArming()

    assert arming.standby((0.0, 0.0, 0.0)) is True
    assert arming.motion_started is False

    assert arming.standby((0.01, 0.0, 0.0)) is False
    assert arming.motion_started is True

    assert arming.standby((0.0, 0.0, 0.0)) is False


def test_threshold_arming_holds_ramp_then_latches_motion():
    arming = MotionArming((0.03, 0.0225, 0.03))

    assert arming.standby((0.02, 0.0, 0.0)) is True
    assert arming.standby((0.03, 0.0, 0.0)) is False
    assert arming.standby((0.0, 0.0, 0.0)) is False


def test_threshold_arming_accepts_combined_diagonal_onset():
    arming = MotionArming((0.03, 0.0225, 0.03))

    assert arming.standby((0.0213, 0.0213, 0.0)) is False


def test_reaches_onset_can_gate_repeat_start_without_resetting_latch():
    arming = MotionArming((0.03, 0.0225, 0.03))

    assert arming.standby((0.03, 0.0, 0.0)) is False
    assert arming.reaches_onset((0.0006, 0.0, 0.0)) is False
    assert arming.reaches_onset((0.03, 0.0, 0.0)) is True
    assert arming.standby((0.0, 0.0, 0.0)) is False


@pytest.mark.parametrize(
    'command',
    [
        (0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0),
        (np.nan, 0.0, 0.0),
    ],
)
def test_invalid_commands_fail_closed(command):
    """Malformed or non-finite commands are rejected."""
    with pytest.raises(ValueError, match='three finite'):
        MotionArming().standby(command)
