import pytest

import execute_dual
from execute_dual import DualCycle


def mock_timed_cycle(monkeypatch, sim_step=.02, wall_step=.6):
    cycle = DualCycle.__new__(DualCycle)
    cycle.sim = 50.
    cycle.safety = lambda: None
    cycle.record = lambda kind, data: None
    wall = [0.]
    monkeypatch.setattr(execute_dual.time, 'monotonic', lambda: wall[0])

    def spin():
        wall[0] += wall_step
        cycle.sim += sim_step

    cycle.spin = spin
    return cycle, wall


def test_both_ten_sim_second_holds_cannot_expire_only_on_old_twenty_wall_factor(monkeypatch):
    cycle, wall = mock_timed_cycle(monkeypatch)
    cycle.dwell(10.)
    assert cycle.sim >= 60.
    assert wall[0] > 200.


def test_thirty_sim_second_stop_schedule_wait_is_not_fixed_six_hundred_wall_seconds(monkeypatch):
    cycle, wall = mock_timed_cycle(monkeypatch, wall_step=.5)
    cycle.wait_until_sim(80.)
    assert cycle.sim >= 80.
    assert 600. < wall[0] <= 900.


def test_simulation_wait_still_rejects_clock_stagnation(monkeypatch):
    cycle, wall = mock_timed_cycle(monkeypatch, sim_step=0.)
    with pytest.raises(TimeoutError, match='stopped advancing'):
        cycle.dwell(10.)


@pytest.mark.parametrize('target', [float('nan'), float('inf'), 49.])
def test_invalid_or_past_target_cannot_silently_complete_certificate(monkeypatch, target):
    cycle, wall = mock_timed_cycle(monkeypatch)
    with pytest.raises(ValueError):
        cycle.wait_until_sim(target)


def test_nonpositive_hold_is_not_a_valid_certificate(monkeypatch):
    cycle, wall = mock_timed_cycle(monkeypatch)
    with pytest.raises(ValueError):
        cycle.dwell(0.)
