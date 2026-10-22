import math

import pytest

from coherent_time import JointTimeline
from indexed_timeline import IndexedJointTimeline, indexed_copy
from speed_coherent import select_coherent_pair


@pytest.mark.parametrize('capacity', [2, 17, 256])
def test_original_queries_are_exactly_equal_during_rollover(capacity):
    original = JointTimeline(('shoulder', 'finger'), capacity=capacity)
    candidate = IndexedJointTimeline(('shoulder', 'finger'), capacity=capacity)
    for number in range(600):
        stamp = number * .002
        positions = dict(shoulder=math.sin(stamp), finger=.02 * math.cos(stamp))
        for timeline in (original, candidate):
            timeline.add(stamp, positions)
        for offset in (-.500001, -.011, -.002, -1e-11, 0, 1e-11, .001, .02):
            assert candidate.at(stamp + offset) == original.at(stamp + offset)
        assert tuple(candidate.rows) == tuple(original.rows)


@pytest.mark.parametrize('query', [-1., 0., .01, .02, .03, .04, 1.])
def test_missing_brackets_and_large_gaps_preserved(query):
    original = JointTimeline(('joint',))
    candidate = IndexedJointTimeline(('joint',))
    for timeline in (original, candidate):
        timeline.add(0., {'joint': 1.})
        timeline.add(.03, {'joint': 2.})
    assert candidate.at(query) == original.at(query)


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_queries_fail(stamp):
    with pytest.raises(ValueError):
        IndexedJointTimeline(('joint',)).at(stamp)


def test_returns_defensive_copy_and_preserves_add_validation():
    timeline = IndexedJointTimeline(('joint',))
    timeline.add(1., {'joint': 3.})
    timeline.add(1., {'joint': 3.})
    result = timeline.at(1.)
    result['positions']['joint'] = 42.
    assert timeline.at(1.)['positions']['joint'] == 3.
    with pytest.raises(ValueError):
        timeline.add(1., {'joint': 4.})
    with pytest.raises(ValueError):
        timeline.add(.9, {'joint': 3.})
    with pytest.raises(ValueError):
        timeline.add(2., {'joint': math.nan})


def test_50ms_native_lag_and_12ms_interpolation_contract_unchanged():
    original = JointTimeline(('joint',))
    candidate = IndexedJointTimeline(('joint',))
    for timeline in (original, candidate):
        timeline.add(97.92, {'joint': 1.})
        timeline.add(97.93, {'joint': 2.})
    frames = [(stamp, 10., {}) for stamp in (97.926, 97.9305, 97.935, 97.939, 97.943)]
    assert select_coherent_pair(frames, candidate, 97.943) == select_coherent_pair(frames, original, 97.943)
    assert candidate.maximum_gap_s == .012


def test_conversion_retains_actual_history_and_capacity_without_aliasing():
    original = JointTimeline(('joint',), maximum_gap_s=.005, capacity=17)
    original.add(1., {'joint': 3.})
    original.add(1.004, {'joint': 4.})
    candidate = indexed_copy(original)
    assert candidate.rows == original.rows
    assert candidate.at(1.002) == original.at(1.002)
    assert candidate.rows.maxlen == 17
    assert candidate.maximum_gap_s == .005
    original.rows[0][1]['joint'] = 99.
    assert candidate.at(1.)['positions']['joint'] == 3.
