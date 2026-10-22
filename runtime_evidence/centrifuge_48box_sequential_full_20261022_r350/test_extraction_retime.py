import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle


@pytest.mark.parametrize('phase,rate,expected', [
    ('left_retrieval_pilot_lift', .12, .03), ('right_retrieval_pilot_lift', .12, .03),
    ('left_retrieval_pilot_lift', .005, .005), ('right_retrieval_pilot_lift', .005, .005),
    ('left_pilot_lift', 2.6, 2.6), ('right_pilot_lift', 2.6, 2.6),
    ('left_retrieval_lift', .20, .20), ('right_retrieval_lift', .20, .20)])
def test_only_retrieval_pilot_caps_rate_and_keeps_exact_geometry(monkeypatch, phase, rate, expected):
    cycle = FullCycle.__new__(FullCycle)
    samples = [dict(torso_joint=0.0), dict(torso_joint=.1)]
    events = []
    cycle.record = lambda kind, data: events.append((kind, data))
    calls = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, path, stage, speed: calls.append((path, stage, speed)))
    cycle.move(samples, phase, rate)
    assert calls == [(samples, phase, expected)]
    assert calls[0][0] is samples
    if rate != expected:
        assert events[0][0] == 'retrieval_pilot_retime'
        assert events[0][1]['unchanged_path']

