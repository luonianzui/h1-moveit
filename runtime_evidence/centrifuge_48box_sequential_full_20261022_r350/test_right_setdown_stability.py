import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from right_setdown_cycle import SpeedPriorityCycle
from sequential_speed_cycle import SpeedPriorityCycle as ParentCycle
from speed_policy import common_timing


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r338'


def fixture(hand='right'):
    number = 20 if hand == 'right' else 16
    source = PARENT / f'run/{number}_retrieval_transport_{hand}_result.json'
    plan = json.loads(source.read_text())
    samples = next(stage['samples'] for stage in plan['stages'] if stage['phase'] == 'retrieve_setdown')
    coarse = samples[:max(1, len(samples) - 8) + 1]
    owners = copy.deepcopy(plan['original_runtime_holder_references'])
    cycle = object.__new__(SpeedPriorityCycle)
    cycle.holders = SimpleNamespace(rows=owners, snapshot=lambda: copy.deepcopy(owners))
    cycle.order = SimpleNamespace(phase='retrieval', holder=dict(hand=hand, payload=hand),
                                 locations={hand: 'held'}, retrieval_assignment={hand: hand})
    cycle.sequential_setdown_context = dict(hand=hand, payload=hand, original_holders=copy.deepcopy(owners),
        coarse_samples=copy.deepcopy(coarse), source_snapshot_sha256=plan['source_snapshot_sha256'])
    cycle.phase = 'paired_retrieval_setdown'
    cycle.events = []
    cycle.record = lambda kind, data: cycle.events.append((kind, data))
    return cycle, coarse, source


def test_right_rate_only():
    cycle, samples, source = fixture()
    assert cycle.motion_limits(cycle.phase, .06) == (.2, .04)


def test_left_timing_unchanged():
    cycle, samples, source = fixture('left')
    assert cycle.motion_limits(cycle.phase, .13) == ParentCycle.motion_limits(cycle, cycle.phase, .13)
    assert cycle.common_timing(samples[0], samples, .4) == common_timing(samples[0], samples, .4, ramp_s=.3)


def test_fine_and_transfer_rates_unchanged():
    cycle, samples, source = fixture()
    for phase in ('paired_retrieval_setdown_fine', 'seq_retrieval_retrieve_transfer', 'seq_load_load_lower'):
        assert cycle.motion_limits(phase, .5) == ParentCycle.motion_limits(cycle, phase, .5)


def test_right_identical_points_groups_and_slower_ramp():
    cycle, samples, source = fixture()
    old = common_timing(samples[0], samples, .4, ramp_s=.3)
    current = cycle.common_timing(samples[0], samples, .2)
    assert current['points'] == old['points'] and current['groups'] == old['groups']
    assert current['duration_sim_s'] > old['duration_sim_s']
    assert current['acceleration_ramp_sim_s'] == .6
    assert cycle.events[-1][1]['actual_rate_rad_s'] == .2
    assert cycle.events[-1][1]['original_runtime_holder_references'] == cycle.holders.snapshot()


def test_changed_holder_rejected():
    cycle, samples, source = fixture()
    cycle.holders.rows['right']['payload'] = 'left'
    with pytest.raises(ValueError):
        cycle.motion_limits(cycle.phase, .06)


def test_changed_path_rejected():
    cycle, samples, source = fixture()
    with pytest.raises(ValueError):
        cycle.common_timing(samples[0], samples[1:], .2)


def test_old_right_rate_rejected():
    cycle, samples, source = fixture()
    with pytest.raises(ValueError):
        cycle.common_timing(samples[0], samples, .4)


def test_runtime_entry_uses_new_subclass():
    contents = (ROOT / 'execute_sequential.py').read_text()
    assert 'from table_alignment_cycle import SpeedPriorityCycle' in contents
    assert 'from right_setdown_cycle import SpeedPriorityCycle as ParentSpeedPriorityCycle' in (ROOT / 'table_alignment_cycle.py').read_text()
    from speed_source import NEW_ENTRY
    assert NEW_ENTRY in contents


def test_gentle_retrieval_pilot_both_hands():
    for hand in ('left', 'right'):
        cycle, samples, source = fixture(hand)
        assert cycle.motion_limits('seq_retrieval_single_pilot', .25) == (.05, .04)


def test_gentle_pilot_rejects_nonretrieval_context():
    cycle, samples, source = fixture()
    cycle.order.phase = 'loading'
    with pytest.raises(ValueError):
        cycle.motion_limits('seq_retrieval_single_pilot', .25)


def test_pilot_independent_timing_rejects_prior_rate():
    from audit_speed_paths import verify_axis_exit_timing
    cycle, samples, source = fixture('left')
    timing = common_timing(samples[0], samples, .05)
    event = dict(phase='seq_retrieval_single_pilot', payload=dict(
        holders=cycle.holders.snapshot(), initial=samples[0], samples=samples,
        actual_rate_rad_s=.05, times=timing['times'], groups=timing['groups']))
    verify_axis_exit_timing(event)
    event['payload']['actual_rate_rad_s'] = .1
    with pytest.raises(ValueError):
        verify_axis_exit_timing(event)


def test_independent_timing_audit_accepts_only_current_right_profile():
    from audit_speed_paths import verify_axis_exit_timing
    cycle, samples, source = fixture()
    timing = cycle.common_timing(samples[0], samples, .2)
    event = dict(phase=cycle.phase, payload=dict(
        holders=cycle.holders.snapshot(), initial=samples[0], samples=samples,
        actual_rate_rad_s=.2, times=timing['times'], groups=timing['groups']))
    verify_axis_exit_timing(event)
    event['payload']['actual_rate_rad_s'] = .4
    with pytest.raises(ValueError):
        verify_axis_exit_timing(event)
