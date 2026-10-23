import math

import pytest

from coherent_time import JointTimeline, newest_pair


def timeline():
    history = JointTimeline(['first', 'second'])
    history.add(1., {'first':0., 'second':2.})
    history.add(1.01, {'first':.005, 'second':2.01})
    return history


def test_actual_bracketing_interpolation_and_exact_source_stamp():
    history = timeline()
    result = history.at(1.005)
    assert result['positions'] == pytest.approx({'first':.0025, 'second':2.005})
    assert result['lower_stamp'] == 1 and result['upper_stamp'] == 1.01
    assert result['interpolation'] and result['fraction'] == pytest.approx(.5)
    assert not history.at(1.01)['interpolation']


@pytest.mark.parametrize('stamp', [.999, 1.011, 100.])
def test_no_extrapolation_even_when_command_trajectory_is_known(stamp):
    assert timeline().at(stamp) is None


def test_missing_frame_gap_is_not_filled_from_commands_or_old_state():
    history = JointTimeline(['first'])
    history.add(1., {'first':0.})
    history.add(1.03, {'first':.015})
    assert history.at(1.01) is None


@pytest.mark.parametrize('stamp,positions', [(math.nan, {'first':0., 'second':0.}),
    (1.02, {'first':math.nan, 'second':0.}), (1.02, {'first':0.}),
    (.99, {'first':0., 'second':0.}), (1.01, {'first':.1, 'second':2.01})])
def test_bad_or_conflicting_actual_frames_fail_closed(stamp, positions):
    with pytest.raises(ValueError):
        timeline().add(stamp, positions)


def test_identical_duplicate_is_idempotent_not_a_new_measurement():
    history = timeline()
    history.add(1.01, {'first':.005, 'second':2.01})
    assert len(history.rows) == 2


def test_newest_coherent_pair_cannot_hide_old_native_state():
    history = timeline()
    frames = [(1.003, 50., {'name':'old'}), (1.007, 51., {'name':'recent'}),
              (1.011, 52., {'name':'future_of_joints'})]
    result = newest_pair(frames, history, 1.011)
    assert result['poses']['name'] == 'recent' and result['pose_stamp'] == 1.007
    assert newest_pair(frames, history, 1.03) is None


@pytest.mark.parametrize('gap', [0., .02, math.inf])
def test_bracket_bounds_cannot_be_relaxed(gap):
    with pytest.raises(ValueError):
        JointTimeline(['joint'], maximum_gap_s=gap)
