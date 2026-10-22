from types import SimpleNamespace

import numpy as np
import pytest

import sequential_planning


@pytest.mark.parametrize('retrieval', [False])
def test_loading_pilot_keeps_world_vertical_fifteen_mm(monkeypatch, retrieval):
    body = np.eye(4)
    body[:3, 3] = [.613, .064, 1.052]
    body[:3, :3] = sequential_planning.Rotation.from_euler('xyz', [.01, -.02, .6]).as_matrix()
    observed = {}
    planner = SimpleNamespace(names=['joint'], attach=lambda *args: None)

    def stage(first, last, seed, phase, **kwargs):
        observed.update(first=first.copy(), last=last.copy(), kwargs=kwargs)
        return dict(samples=[seed, seed])

    planner.stage = stage
    monkeypatch.setattr(sequential_planning, 'SinglePlanner', lambda *args: planner)
    monkeypatch.setattr(sequential_planning, 'pose_matrix', lambda value: body.copy())
    monkeypatch.setattr(sequential_planning, 'result', lambda *args: {})
    sequential_planning.pilot(dict(hand='left', payload='left', positions=dict(joint=0.),
                                  poses=dict(box48_left={})), np.eye(4), retrieval=retrieval)
    assert np.array_equal(observed['first'][:2, 3], observed['last'][:2, 3])
    assert np.array_equal(observed['first'][:3, :3], observed['last'][:3, :3])
    assert observed['last'][2, 3] - observed['first'][2, 3] == pytest.approx(.015)
    assert observed['kwargs'] == dict(count=21, support_start='basket' if retrieval else 'table', refine=False)
