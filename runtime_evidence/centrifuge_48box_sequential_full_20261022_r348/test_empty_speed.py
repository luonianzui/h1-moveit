import inspect

import pytest

from empty_speed import EMPTY_PHASES, motion_rate, minimum_interval, button_travel_rate
from execute_cycle import FullCycle
from execute_loading import LoadingCycle


ARM_PHASES = [side + suffix for side in ('left', 'right')
              for suffix in ('_park', '_approach', '_retreat', '_return', '_retrieval_approach',
                             '_retrieval_retreat', '_retrieval_return', '_descend', '_lift',
                             '_pilot_lift', '_transfer', '_lower', '_align', '_retrieval_descend',
                             '_retrieval_lift', '_retrieval_pilot_lift', '_retrieval_transfer',
                             '_retrieval_setdown')]


@pytest.mark.parametrize('phase', ARM_PHASES)
def test_all_arm_profiles_and_dense_point_period_remain_original(phase):
    assert not EMPTY_PHASES
    assert motion_rate(phase, .36, None, None) == .36
    assert minimum_interval(phase, None, None) == .020


@pytest.mark.parametrize('holding,held_payload', [('left', None), (None, 'right'), ('right', 'left')])
def test_live_held_identity_always_rejects_button_travel(holding, held_payload):
    assert motion_rate('right_approach', .36, holding, held_payload) == .36
    assert minimum_interval('right_approach', holding, held_payload) == .020
    with pytest.raises(ValueError):
        button_travel_rate(holding, held_payload)


@pytest.mark.parametrize('requested', [0, -.1, float('nan'), float('inf')])
def test_invalid_rate_is_rejected(requested):
    with pytest.raises(ValueError):
        motion_rate('left_approach', requested, None, None)


def test_full_cycle_adapter_keeps_original_arm_geometry_rate_and_no_retime_event(monkeypatch):
    cycle = FullCycle.__new__(FullCycle)
    cycle.holding = cycle.held_payload = None
    events, captured = [], []
    cycle.record = lambda kind, payload: events.append((kind, payload))
    monkeypatch.setattr(LoadingCycle, 'move',
                        lambda self, samples, phase, rate: captured.append((samples, phase, rate)))
    samples = [dict(torso_joint=.1), dict(torso_joint=.2)]
    cycle.move(samples, 'left_approach', .36)
    assert captured == [(samples, 'left_approach', .36)]
    assert captured[0][0] is samples and not events


def test_only_empty_button_travel_is_doubled_not_contact_intervals():
    source = inspect.getsource(FullCycle.press)
    assert 'fixed_interval=.1' in source and 'fixed_interval=.07' in source
    assert button_travel_rate(None, None) == .44
    source = inspect.getsource(FullCycle.move_button)
    assert 'else max(.02,' in source and '/ travel_rate)' in source


def test_real_arm_trajectory_times_keep_original_twenty_ms_floor():
    source = inspect.getsource(LoadingCycle.move)
    assert 'max(self.minimum_waypoint_interval(phase),' in source
    cycle = FullCycle.__new__(FullCycle)
    cycle.holding = cycle.held_payload = None
    for phase in ARM_PHASES:
        assert cycle.minimum_waypoint_interval(phase) == .020
