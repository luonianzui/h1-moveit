from types import SimpleNamespace

import pytest

from coherent_time import JointTimeline, newest_pair
from speed_coherent import select_coherent_pair
from speed_policy import PROFILE
from speed_source import restore_speed_source
from pathlib import Path
import inspect

from execute_coherent import CoherentCycle
from execute_dual import DualCycle
from sequential_speed_cycle import SpeedPriorityCycle
from speed_feedback_cycle import SpeedFeedbackMixin


def test_actual_r324_17ms_pair_is_supported_without_extrapolation():
    timeline = JointTimeline(['joint'])
    timeline.add(97.92, {'joint': 1.})
    timeline.add(97.93, {'joint': 2.})
    frames = [(stamp, 10., {}) for stamp in (97.926, 97.9305, 97.935, 97.939, 97.943)]
    assert newest_pair(frames, timeline, 97.943) is None
    pair = select_coherent_pair(frames, timeline, 97.943)
    assert pair['pose_stamp'] == 97.926
    assert pair['native_lag_s'] == pytest.approx(.017)
    assert pair['joints']['lower_stamp'] == 97.92
    assert pair['joints']['upper_stamp'] == 97.93
    assert pair['joints']['positions']['joint'] == pytest.approx(1.6)


@pytest.mark.parametrize('latest', [None, float('nan'), float('inf'), -1.])
def test_invalid_latest_rejected(latest):
    with pytest.raises((TypeError, ValueError)):
        select_coherent_pair([], JointTimeline(['joint']), latest)


def test_missing_bracket_and_older_than_50ms_are_rejected():
    timeline = JointTimeline(['joint'])
    timeline.add(1., {'joint': 1.})
    timeline.add(1.01, {'joint': 2.})
    assert select_coherent_pair([(1.02, 5., {})], timeline, 1.02) is None
    assert select_coherent_pair([(1., 5., {})], timeline, 1.05001) is None
    assert select_coherent_pair([(1., 5., {})], timeline, 1.05) is not None


def test_original_interpolation_width_not_relaxed():
    timeline = JointTimeline(['joint'])
    timeline.add(1., {'joint': 1.})
    timeline.add(1.02, {'joint': 2.})
    assert select_coherent_pair([(1.01, 5., {})], timeline, 1.01) is None
    with pytest.raises(ValueError):
        select_coherent_pair([], SimpleNamespace(maximum_gap_s=.05), 1.)
    assert PROFILE['coherent_joint_bracket_sim_s'] == .012


@pytest.mark.parametrize('name', ['execute_coherent.py', 'execute_dual.py', 'trace_io.py'])
def test_only_declared_callback_and_compression_deltas_from_r324(name):
    root = Path(__file__).resolve().parent
    previous = root.parent / 'centrifuge_48box_sequential_full_20261020_r324'
    expected = (previous / name).read_text().replace(str(previous), str(root)).replace(previous.name, root.name)
    assert restore_speed_source(name, (root / name).read_text()) == expected


def test_mixin_only_changes_pair_selector_not_latest_native_guard():
    for name in ('spin', 'measured_pair'):
        actual = inspect.getsource(getattr(SpeedFeedbackMixin, name)).replace('select_coherent_pair(', 'newest_pair(')
        assert actual == inspect.getsource(getattr(DualCycle, name))
    actual = inspect.getsource(SpeedFeedbackMixin._check_speed_coherent_grip)
    actual = actual.replace('_check_speed_coherent_grip(', 'check_coherent_grip(').replace('select_coherent_pair(', 'newest_pair(')
    assert actual == inspect.getsource(CoherentCycle.check_coherent_grip)
    assert SpeedPriorityCycle.spin is SpeedFeedbackMixin.spin
    assert SpeedPriorityCycle.check_coherent_grip is SpeedFeedbackMixin.check_coherent_grip
    assert SpeedPriorityCycle.latest_native_guard is DualCycle.latest_native_guard
