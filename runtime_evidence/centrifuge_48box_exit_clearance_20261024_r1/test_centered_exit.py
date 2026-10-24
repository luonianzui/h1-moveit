from types import SimpleNamespace

import numpy as np
import pytest

from axial_transport import SpeedRetrievalPlanner
from centered_exit import CenteredExitPlanner


def fixture():
    planner = CenteredExitPlanner.__new__(CenteredExitPlanner)
    planner.payload = 'left'
    pose = dict(position=dict(x=0., y=0., z=0.), orientation=dict(x=0., y=0., z=0., w=1.))
    planner.request = dict(poses=dict(centrifuge=pose, bucket_left=pose))
    first = np.eye(4)
    first[0, 3] = .0009
    last = first.copy()
    last[2, 3] = .05
    return planner, first, last


def test_original_success_is_returned_without_any_correction(monkeypatch):
    sentinel = object()
    monkeypatch.setattr(SpeedRetrievalPlanner, 'guided_stage', lambda *args, **kwargs: sentinel)
    planner, first, last = fixture()
    assert planner.guided_stage(first, last, {}, 'retrieve_axis_exit', []) is sentinel


@pytest.mark.parametrize('phase,error', [('retrieve_lift', 'Single-holder dense full-scene collision: test'),
                                      ('retrieve_axis_exit', 'Original shrunken joint bound exceeded'),
                                      ('retrieve_axis_exit', 'Single-arm proposal jumps between IK branches')])
def test_other_failures_remain_failures(monkeypatch, phase, error):
    def reject(*args, **kwargs):
        raise RuntimeError(error)
    monkeypatch.setattr(SpeedRetrievalPlanner, 'guided_stage', reject)
    planner, first, last = fixture()
    with pytest.raises(RuntimeError, match=error):
        planner.guided_stage(first, last, {}, phase, [])


@pytest.mark.parametrize('offset', [-.00201, .00201, float('nan')])
def test_unbounded_or_nonfinite_correction_is_rejected(monkeypatch, offset):
    def reject(*args, **kwargs):
        raise RuntimeError('Single-holder dense full-scene collision: test')
    monkeypatch.setattr(SpeedRetrievalPlanner, 'guided_stage', reject)
    planner, first, last = fixture()
    first[0, 3] = offset
    with pytest.raises(RuntimeError, match='exceeds two millimeters'):
        planner.guided_stage(first, last, {}, 'retrieve_axis_exit', [])


def test_correction_and_exit_do_not_receive_support_exemption(monkeypatch):
    calls = []
    planner, first, last = fixture()
    seed = dict(torso_joint=.1)
    reference = np.eye(4)
    planner.reference = reference.copy()
    def guided(self, begin, end, start, phase, guides, table_end=False, support_start=None):
        calls.append(dict(begin=begin.copy(), end=end.copy(), support=support_start, start=start))
        if len(calls) == 1:
            raise RuntimeError('Single-holder dense full-scene collision: test')
        assert support_start is None and not table_end
        return dict(samples=[start, dict(torso_joint=.2)], last_body=end.tolist())
    def stage(begin, end, start, phase, **kwargs):
        assert kwargs == dict(count=11, refine=False)
        assert phase == 'retrieve_axis_exit'
        assert np.array_equal(begin, first) and np.array_equal(end[:3, :3], first[:3, :3])
        assert abs(end[0, 3]) < 1e-12
        return dict(samples=[start, dict(torso_joint=.15)])
    monkeypatch.setattr(SpeedRetrievalPlanner, 'guided_stage', guided)
    planner.stage = stage
    result = planner.guided_stage(first, last, seed, 'retrieve_axis_exit', [], support_start='basket')
    assert calls[0]['support'] == 'basket' and calls[1]['support'] is None
    assert np.allclose(calls[1]['end'][:3, 3], [0., 0., .05], atol=1e-12)
    assert result['samples'] == [seed, dict(torso_joint=.15), dict(torso_joint=.2)]
    assert result['first_body'] == first.tolist()
    assert np.array_equal(planner.reference, reference)
    assert result['clearance_correction']['no_basket_collision_exemption']


def test_corrected_path_still_rejects_collision(monkeypatch):
    def reject(*args, **kwargs):
        raise RuntimeError('Single-holder dense full-scene collision: retained')
    monkeypatch.setattr(SpeedRetrievalPlanner, 'guided_stage', reject)
    planner, first, last = fixture()
    planner.stage = reject
    with pytest.raises(RuntimeError, match='retained'):
        planner.guided_stage(first, last, {}, 'retrieve_axis_exit', [])
