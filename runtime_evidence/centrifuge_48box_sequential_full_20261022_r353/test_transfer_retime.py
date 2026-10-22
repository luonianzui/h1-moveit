import json
from pathlib import Path

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle, trajectory_rate


@pytest.mark.parametrize('side,phase,expected', [
    ('left', 'transfer', .065), ('right', 'lower', .065),
    ('right', 'transfer', .13), ('left', 'lower', .26),
    ('left', 'lift', .312), ('right', 'lift', .312),
    ('left', 'reorient', .13), ('right', 'reorient', .26)])
def test_actual_loading_rates_are_retimed_not_scale_multiplier(monkeypatch, side, phase, expected):
    actual_rate = trajectory_rate(side, phase)
    cycle = FullCycle.__new__(FullCycle)
    samples = [dict(torso_joint=0.0), dict(torso_joint=.1)]
    events = []
    cycle.record = lambda kind, data: events.append((kind, data))
    calls = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, path, stage, speed: calls.append((path, stage, speed)))
    cycle.move(samples, side + '_' + phase, actual_rate)
    assert calls == [(samples, side + '_' + phase, expected)]
    assert calls[0][0] is samples
    if side + '_' + phase in ('left_transfer', 'right_lower'):
        assert actual_rate == pytest.approx(.13)
        assert min(actual_rate, 1.3) == actual_rate
        assert expected == actual_rate / 2
        assert events[0][1]['effective_rate_rad_s'] == expected
        assert events[0][1]['original_rate_rad_s'] == actual_rate
    else:
        assert not events


@pytest.mark.parametrize('side,phase', [('left', 'transfer'), ('right', 'lower')])
def test_canonical_plan_duration_actually_increases_with_new_cap(side, phase):
    plan = json.loads(Path(__file__).with_name('inputs').joinpath('plan.json').read_text())
    stage = next(stage for stage in plan['stages'] if stage['side'] == side and stage['phase'] == phase)
    points = [{name: value for name, value in sample.items() if 'dex1_finger' not in name}
              for sample in stage['samples']]

    def duration(rate):
        previous = points[0]
        elapsed = .1
        for point in [points[0]] + points:
            elapsed += max(.020, max(abs(point[name] - previous[name]) / rate for name in point))
            previous = point
        return elapsed

    original = trajectory_rate(side, phase)
    assert original == pytest.approx(.13)
    assert duration(min(original, 1.3)) == duration(original)
    assert duration(.065) > duration(original)
    assert duration(.065) <= 2 * duration(original)

