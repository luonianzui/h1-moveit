import inspect

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from setdown_braking import braking_setdown_segments


def test_split_preserves_every_audited_point_and_shared_boundary():
    samples = [dict(torso_joint=index / 1000) for index in range(121)]
    coarse, fine, split = braking_setdown_segments(samples)
    assert split == 116 and len(fine) == 5 and len(coarse) == 117
    assert coarse[-1] == fine[0] == samples[116]
    assert coarse[:-1] + fine == samples
    assert samples[-1] == fine[-1]


@pytest.mark.parametrize('length', [0, 5, 120, 122])
def test_non_original_path_length_rejected(length):
    with pytest.raises(ValueError):
        braking_setdown_segments([dict(torso_joint=0)] * length)


@pytest.mark.parametrize('hand', ['left', 'right'])
def test_only_retrieval_setdown_brakes_at_quarter_rate(monkeypatch, hand):
    captured = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, points, phase, rate: captured.append((points, phase, rate)))
    cycle = FullCycle.__new__(FullCycle)
    cycle.record = lambda *args: None
    samples = [dict(torso_joint=index / 1000) for index in range(121)]
    cycle.move(samples, hand + '_retrieval_setdown', .12)
    assert len(captured) == 2
    assert captured[0][2] == .12 and captured[1][2] == .03
    assert captured[0][0][:-1] + captured[1][0] == samples
    assert all(row[1] == hand + '_retrieval_setdown' for row in captured)


def test_original_endpoint_contact_and_grip_gates_not_relaxed():
    original = inspect.getsource(LoadingCycle.move)
    assert "else .0003" in original and "self.safety()" in inspect.getsource(LoadingCycle.spin)
    assert "phase in ('left_retrieval_setdown', 'right_retrieval_setdown')" in inspect.getsource(FullCycle.move)

