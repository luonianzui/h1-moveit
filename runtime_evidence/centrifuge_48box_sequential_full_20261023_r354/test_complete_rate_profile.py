import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from complete_rate_profile import EMPTY_CAPS, LOADED_RATES, restore_profile_source
from inherited_source_identity import restore_inherited_source
from compression_cycle import CompressionCycle
from dual_motion import ARMS, TASK_JOINTS, common_arm_timing
from execute_fast import BUTTON_CONTACT_INTERVAL
from efficient_gripper_cycle import EfficientGripperCycle
from handover_policy import FREE_PHASES
from test_dual_executor import Publisher, mocked_cycle
from test_loaded_dispatch import loaded_cycle


EVIDENCE = Path(__file__).resolve().parent


def actual_publisher_cycle(loaded=True):
    cycle = loaded_cycle() if loaded else mocked_cycle()
    cycle.__class__ = CompressionCycle
    cycle.current_damping_empty_approach_completed = True
    cycle.events = []
    cycle.record = lambda kind, value: cycle.events.append((kind, value))

    def spin():
        cycle.sim += .01
        cycle.joint_time = cycle.sim
        for publisher in cycle.publishers.values():
            if not publisher.messages:
                continue
            message = publisher.messages[-1]
            start = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                     for point in message.points]
            for index, name in enumerate(message.joint_names):
                cycle.positions[name] = float(np.interp(cycle.joint_time - start, times,
                    [point.positions[index] for point in message.points]))

    cycle.spin = spin
    return cycle


def waypoints(cycle):
    first, last = dict(cycle.positions), dict(cycle.positions)
    for name, value in [('left_elbow_joint', .12), ('right_elbow_joint', -.10), ('torso_joint', .01)]:
        first[name] = value
        last[name] = value * 2
    return [first, last]


def assert_real_three_group_messages(cycle, samples, rate, initial):
    expected = common_arm_timing(initial, samples, rate)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert cycle.motion_count == len(messages) == 3
    assert {name for message in messages for name in message.joint_names} == TASK_JOINTS
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 300000000)}
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .02 - 1e-9
        for point, source in zip(message.points, expected['points']):
            assert list(point.positions) == [source[name] for name in message.joint_names]
    command = next(value for kind, value in cycle.events if kind == 'command')
    assert command['actual_rate_rad_s'] == rate and command['samples'] == samples
    assert command['holders'] == cycle.holders.snapshot()


@pytest.mark.parametrize('phase', sorted(FREE_PHASES))
def test_free_profile_runs_actual_inherited_three_publisher_dispatch(phase):
    cycle = actual_publisher_cycle()
    initial, samples = dict(cycle.positions), waypoints(cycle)
    cycle.move(samples, phase, .25)
    assert_real_three_group_messages(cycle, samples, .6, initial)


@pytest.mark.parametrize('phase', sorted(FREE_PHASES))
@pytest.mark.parametrize('requested', [.4, .6, .8, math.nan])
def test_free_profile_requires_original_call_contract_before_any_publisher(phase, requested):
    cycle = actual_publisher_cycle()
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), phase, requested)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase,rates', sorted(LOADED_RATES.items()))
@pytest.mark.parametrize('candidate_requested', [False, True])
def test_declared_loaded_phase_retimes_real_messages_not_only_metadata(phase, rates, candidate_requested):
    cycle = actual_publisher_cycle()
    initial, samples = dict(cycle.positions), waypoints(cycle)
    cycle.move(samples, phase, rates[int(candidate_requested)])
    assert_real_three_group_messages(cycle, samples, rates[1], initial)
    assert cycle.maximum_loaded_rate(phase) == rates[1]


@pytest.mark.parametrize('phase,cap', sorted(EMPTY_CAPS.items()))
def test_empty_profile_retains_original_strict_tracking_on_actual_dispatch(phase, cap):
    cycle = actual_publisher_cycle(loaded=False)
    initial, samples = dict(cycle.positions), waypoints(cycle)
    assert cycle.motion_limits(phase, .8) == (cap, .002)
    cycle.move(samples, phase, .8)
    assert_real_three_group_messages(cycle, samples, cap, initial)


@pytest.mark.parametrize('phase', sorted(FREE_PHASES | LOADED_RATES.keys() | {'paired_retrieval_setdown_fine'}))
@pytest.mark.parametrize('invalid_pair', ['unqualified', 'single', 'duplicate'])
def test_loaded_profile_cannot_publish_before_actual_pair_and_current_qualification(phase, invalid_pair):
    cycle = actual_publisher_cycle()
    if invalid_pair == 'unqualified':
        cycle.current_damping_empty_approach_completed = False
    elif invalid_pair == 'single':
        cycle.holders.release('right')
    else:
        cycle.holders.rows['right']['payload'] = 'left'
    requested = .25 if phase in FREE_PHASES else LOADED_RATES.get(phase, (.09, .09))[0]
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), phase, requested)
    assert cycle.motion_count == 0
    assert all(not publisher.messages for publisher in cycle.publishers.values())


@pytest.mark.parametrize('phase', sorted(EMPTY_CAPS))
@pytest.mark.parametrize('invalid_state', ['unqualified', 'loaded'])
def test_empty_rate_never_bypasses_qualification_or_holder_state(phase, invalid_state):
    cycle = actual_publisher_cycle(loaded=invalid_state == 'loaded')
    cycle.current_damping_empty_approach_completed = invalid_state != 'unqualified'
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), phase, .8)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase', sorted(LOADED_RATES))
@pytest.mark.parametrize('requested', [.02, .79, math.nan, math.inf, 0., -.1])
def test_loaded_profile_rejects_undeclared_rates_before_publish(phase, requested):
    cycle = actual_publisher_cycle()
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), phase, requested)
    assert cycle.motion_count == 0


def test_direct_fine_dispatch_has_same_qualified_getter_as_metadata():
    cycle = actual_publisher_cycle()
    initial, samples = dict(cycle.positions), waypoints(cycle)
    assert cycle.coarse_setdown_rate() == .35 and cycle.fine_setdown_rate() == .09
    with pytest.raises(ValueError):
        cycle.move(samples, 'paired_retrieval_setdown_fine', .06)
    cycle.move(samples, 'paired_retrieval_setdown_fine', .09)
    assert_real_three_group_messages(cycle, samples, .09, initial)


def test_unknown_loaded_phase_keeps_original_cap():
    cycle = actual_publisher_cycle()
    with pytest.raises(ValueError, match='declared phase cap'):
        cycle.move(waypoints(cycle), 'undeclared_loaded_phase', .4)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase,error,expected', [
    ('paired_retrieval_approach', .003, 'stricter .002rad'),
    ('paired_retrieval_transfer', .02, 'original .015rad')])
def test_retimed_actual_dispatch_still_rejects_original_tracking_faults(phase, error, expected):
    cycle = actual_publisher_cycle(loaded=phase in FREE_PHASES)
    original_spin = cycle.spin

    def faulty_spin():
        original_spin()
        if cycle.sim >= 10.4:
            cycle.positions['left_elbow_joint'] += error

    cycle.spin = faulty_spin
    with pytest.raises(RuntimeError, match=expected):
        cycle.move(waypoints(cycle), phase, .25 if phase in FREE_PHASES else .8)


def test_full_cycle_profile_delegates_entire_inherited_program(monkeypatch):
    observed = []
    monkeypatch.setattr(EfficientGripperCycle, 'run_complete', lambda cycle: observed.append('complete_parent_cycle'))
    cycle = actual_publisher_cycle()
    cycle.run_complete()
    assert observed == ['complete_parent_cycle']
    kind, declaration = cycle.events[0]
    assert kind == 'complete_rate_profile_declared'
    assert declaration['original_complete_cycle_and_3_10_30_second_certificates_preserved']
    assert declaration['actual_physics_and_180_second_completion_not_preclaimed']
    assert declaration['loaded_phase_rates_rad_s'] == {phase: rates[1] for phase, rates in LOADED_RATES.items()}


def button_cycle(qualified=True):
    cycle = actual_publisher_cycle(loaded=False)
    cycle.current_damping_empty_approach_completed = qualified
    cycle.holding = cycle.held_payload = None
    cycle.buttons = {'arms': {'right': {'joints': ARMS['right']}}}
    cycle.publishers = {'right_arm': Publisher()}

    def wait(predicate):
        for attempt in range(500):
            cycle.spin()
            if predicate():
                return
        raise TimeoutError('Actual button endpoint callback did not converge')

    cycle.wait = wait
    return cycle


@pytest.mark.parametrize('qualified,rate', [(False, .8), (True, 1.2)])
@pytest.mark.parametrize('fixed_interval', [None, .44])
def test_actual_button_publisher_preserves_geometry_and_fixed_contact_interval(qualified, rate, fixed_interval):
    cycle = button_cycle(qualified)
    points = [[0.] * 7, [.12] * 7, [.24] * 7]
    cycle.move_button('right', points, fixed_interval=fixed_interval)
    message = cycle.publishers['right_arm'].messages[0]
    assert [list(point.positions) for point in message.points] == points
    times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
    increment = .12 / rate if fixed_interval is None else BUTTON_CONTACT_INTERVAL
    assert np.allclose(times, [.05, .05 + increment, .05 + 2 * increment], atol=1e-9, rtol=0)
    command = next(value for kind, value in cycle.events if kind == 'button_arm_command')
    assert command['actual_travel_rate_rad_s'] == (rate if fixed_interval is None else .8)
    assert command['actual_fixed_contact_interval_sim_s'] == BUTTON_CONTACT_INTERVAL
    assert command['start'] == 10.3 and cycle.motion_count == 1


def test_noncontact_button_profile_refuses_any_live_holder_before_publish():
    cycle = button_cycle()
    certificate = dict(source='gazebo_physics_contact', midband_verified=True,
                       normal_loads_N={'1': 10., '2': 10.})
    cycle.holders.begin('left', 'left', np.eye(4), np.eye(4), 10., certificate)
    with pytest.raises(ValueError, match='Loaded physical fingers'):
        cycle.move_button('right', [[0.] * 7])
    assert cycle.motion_count == 0 and not cycle.publishers['right_arm'].messages


@pytest.mark.parametrize('name', ['execute_dual.py', 'stop_handover_cycle.py', 'execute_fast.py'])
def test_profile_source_hooks_are_exactly_reversible_against_verified_parent_clone(name):
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    restored = restore_inherited_source(name, (EVIDENCE / name).read_text())
    assert hashlib.sha256(restored.encode()).hexdigest() == records[name]['derived_sha256']
