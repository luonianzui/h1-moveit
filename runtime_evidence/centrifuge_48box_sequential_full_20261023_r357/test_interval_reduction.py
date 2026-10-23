import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from dual_motion import TASK_JOINTS, validate_common_start
from preclock_audit import validate_preclock_events
from speed_policy import PROFILE
from table_alignment_cycle import SpeedPriorityCycle
from empty_gap_policy import eligible, FREE_EMPTY_PHASES


@pytest.mark.parametrize('stamp', [0., .001, 1., 29.389, 100., 1000.])
def test_reduced_schedule_keeps_original_minimum(stamp):
    state = SimpleNamespace(phase='seq_load_empty_park', holders=SimpleNamespace(rows={}))
    lead = SpeedPriorityCycle.future_start_offset(state)
    assert lead == .16
    assert lead < .3
    validate_common_start(stamp, stamp + lead)


def test_overlap_does_not_change_precision_or_required_observation():
    state = SimpleNamespace(phase='seq_load_empty_park', holders=SimpleNamespace(rows={}))
    assert SpeedPriorityCycle.paired_feedback_overlap_enabled(state)
    target = dict.fromkeys(TASK_JOINTS, 0.)
    state = SimpleNamespace(joint_time=4., positions=dict(target))
    assert SpeedPriorityCycle.paired_endpoint_stamp(state, target, 4.01) is None
    assert SpeedPriorityCycle.paired_endpoint_stamp(state, target, 4.) == 4.
    state.positions['left_elbow_joint'] = .0021
    assert SpeedPriorityCycle.paired_endpoint_stamp(state, target, 4.) is None
    assert PROFILE['endpoint_hold_sim_s'] == .05
    assert PROFILE['steady_spin_sim_s'] == 30.
    assert PROFILE['static_grip_hold_sim_s'] == 1.
    assert PROFILE['release_hold_sim_s'] == .5


def run_motion(monkeypatch, overlap, bad_endpoint_until=0.):
    import sequential_speed_cycle as runtime
    from speed_policy import common_timing
    initial = dict.fromkeys(sorted(TASK_JOINTS), 0.)
    samples = [dict(initial, left_elbow_joint=.002 * index) for index in range(20)]
    timing = common_timing(initial, samples, .4)
    scheduled = 1. + .16
    last_time = scheduled + timing['duration_sim_s']
    messages = []
    cycle = SimpleNamespace(sim=1., joint_time=1., positions=dict(initial),
        holders=SimpleNamespace(rows={}, snapshot=lambda: {}), motion_count=0,
        safety=lambda: None, record=lambda *args: None,
        motion_limits=lambda *args: (.4, .04), future_start_offset=lambda: .16,
        common_timing=lambda *args: copy.deepcopy(timing),
        publishers={name: SimpleNamespace(publish=messages.append) for name in timing['groups']},
        paired_feedback_overlap_enabled=lambda: overlap)
    cycle.paired_endpoint_stamp = lambda target, not_before: SpeedPriorityCycle.paired_endpoint_stamp(cycle, target, not_before)
    def spin():
        cycle.sim += .005
        cycle.joint_time = cycle.sim
        relative = cycle.sim - scheduled
        cycle.positions = {name: float(runtime.np.interp(relative, timing['times'],
            [point[name] for point in timing['points']])) for name in initial}
        if last_time <= cycle.sim < last_time + bad_endpoint_until:
            cycle.positions['left_elbow_joint'] += .003
    cycle.spin = spin
    runtime.SpeedPriorityCycle.move(cycle, samples, 'seq_load_empty_transfer', .4)
    return cycle.sim, messages, last_time


def test_actual_move_removes_duplicate_hold_without_changing_trajectory(monkeypatch):
    old_end, old_messages, last_time = run_motion(monkeypatch, False)
    new_end, new_messages, unused = run_motion(monkeypatch, True)
    assert .035 <= old_end - new_end <= .065
    assert new_end - last_time > .05
    assert old_messages == new_messages


@pytest.mark.parametrize('bad_until', [.04, .08, .15])
def test_endpoint_deviation_still_requires_fresh_full_hold(monkeypatch, bad_until):
    finished, unused, last_time = run_motion(monkeypatch, True, bad_until)
    assert finished >= last_time + bad_until + .05


def row(kind, sim=None, phase='machine_preparation', clock=None):
    return dict(kind=kind, sim=sim, phase=phase, payload=dict(clock_sim=clock, kind='pose'))


@pytest.mark.parametrize('count', [0, 1, 9, 15])
def test_initial_preclock_count_not_hardcoded(count):
    rows = [row('localization_delivery') for unused in range(count)]
    rows += [row('clock', 1.), row('fresh_startup_window_pass', 1.), row('command', 1.)]
    assert len(validate_preclock_events(rows)) == count


@pytest.mark.parametrize('prefix', [
    [row('clock', 1.)], [row('fresh_startup_window_pass')],
    [row('clock', 1.), row('fresh_startup_window_pass', 1.), row('command', 1.)],
])
def test_late_missing_clock_is_not_ignored(prefix):
    with pytest.raises(AssertionError):
        validate_preclock_events(prefix + [row('localization_delivery')])


def test_no_ai_relay_added_to_runtime():
    root = Path(__file__).resolve().parent
    for name in ('table_alignment_cycle.py', 'planning_pipeline.py', 'planner_client.py'):
        source = (root / name).read_text()
        assert all(token not in source for token in ('import openai', 'chat.completions', 'responses.create', 'request_user_input'))


def test_observation_inverse_is_exact_not_a_wildcard():
    from interval_source import ENDPOINT_OBSERVATION, restore_interval_observation
    assert restore_interval_observation('before\n' + ENDPOINT_OBSERVATION + 'after\n') == 'before\nafter\n'
    with pytest.raises(ValueError):
        restore_interval_observation(ENDPOINT_OBSERVATION.replace('measured_hold_sim_s', 'different_field'))


@pytest.mark.parametrize('phase', list(FREE_EMPTY_PHASES))
def test_only_free_empty_phases_shortened(phase):
    state = SimpleNamespace(phase=phase, holders=SimpleNamespace(rows={}))
    assert eligible(phase, {})
    assert SpeedPriorityCycle.future_start_offset(state) == .16
    state.holders.rows = {'right': {'payload': 'right'}}
    assert SpeedPriorityCycle.future_start_offset(state) == .3
    assert not SpeedPriorityCycle.paired_feedback_overlap_enabled(state)


@pytest.mark.parametrize('phase', [
    'seq_retrieval_single_pilot', 'seq_load_single_pilot', 'paired_retrieval_setdown',
    'seq_load_empty_descend', 'seq_retrieval_empty_descend',
    'machine_stop_press', 'machine_open_withdraw', 'right_retrieval_grasp',
    'seq_load_load_lift_and_load_transfer', 'unknown_future_phase',
])
def test_contact_loaded_and_unknown_boundaries_keep_original_wait(phase):
    state = SimpleNamespace(phase=phase, holders=SimpleNamespace(rows={}))
    assert SpeedPriorityCycle.future_start_offset(state) == .3
    assert not SpeedPriorityCycle.paired_feedback_overlap_enabled(state)

