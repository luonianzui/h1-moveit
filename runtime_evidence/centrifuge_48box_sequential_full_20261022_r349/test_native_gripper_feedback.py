import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import native_gripper_feedback
from execute_dual import DualCycle
from inherited_source_identity import restore_inherited_source
from native_gripper_cycle import NativeGripperCycle
from native_gripper_feedback import wait_original_gripper_endpoint
from scheduled_gripper_cycle import ScheduledGripperCycle
from test_zero_effort_timing import mocked_pair
from zero_effort_policy import GRIPPER_JOINTS


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_schedule_gripper_20261014_r268'


@pytest.mark.parametrize('name', ['execute_dual.py', 'execute_cycle.py', 'execute_fast.py', 'efficient_gripper_cycle.py'])
def test_only_native_feedback_seam_changes_qualified_parent_body(name):
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    restored = restore_inherited_source(name, (EVIDENCE / name).read_text())
    assert hashlib.sha256(restored.encode()).hexdigest() == records[name]['derived_sha256']


def endpoint_fixture(error=.0003, fresh=True, converges=True):
    events = []
    cycle = SimpleNamespace(sim=10., joint_time=9.99, positions=dict.fromkeys(GRIPPER_JOINTS, -.0199+error))
    cycle.record = lambda kind, value: events.append((kind, value))
    cycle.safety = lambda: None

    def spin():
        cycle.sim += .01
        if fresh:
            cycle.joint_time = cycle.sim
        if converges:
            cycle.positions.update(dict.fromkeys(GRIPPER_JOINTS, -.0199+.0001))

    cycle.spin = spin
    return cycle, events


@pytest.mark.parametrize('hand', ['left', 'right'])
def test_stale_successful_action_feedback_waits_for_real_newer_frame_at_original_precision(hand):
    cycle, events = endpoint_fixture()
    wait_original_gripper_endpoint(cycle, hand, -.0199, 10.)
    assert cycle.joint_time > 10. and cycle.sim == pytest.approx(10.01)
    assert [kind for kind, value in events] == ['original_gripper_endpoint_fresh_feedback_begin',
                                              'original_gripper_endpoint_fresh_feedback_pass']
    assert events[-1][1]['actual_maximum_error_m'] < .00025


@pytest.mark.parametrize('fresh,converges', [(False, True), (True, False), (False, False)])
def test_stale_or_persistently_out_of_original_precision_feedback_fails_within_one_sim_second(fresh, converges):
    cycle, events = endpoint_fixture(fresh=fresh, converges=converges)
    with pytest.raises(RuntimeError, match='original .00025m'):
        wait_original_gripper_endpoint(cycle, 'left', -.0199, 10.)
    assert cycle.sim-10. < 1.02
    assert events[-1][0] == 'original_gripper_endpoint_fresh_feedback_fault'


def test_native_velocity_safety_fault_is_not_swallowed_while_waiting():
    cycle, events = endpoint_fixture()

    def fault():
        raise RuntimeError('Original native velocity fault')

    cycle.safety = fault
    with pytest.raises(RuntimeError, match='Original native velocity'):
        wait_original_gripper_endpoint(cycle, 'left', -.0199, 10.)
    assert cycle.sim == 10.


def test_clock_stall_keeps_original_wall_bound(monkeypatch):
    cycle, events = endpoint_fixture(fresh=False, converges=False)
    clock = iter((0., 31.))
    monkeypatch.setattr(native_gripper_feedback.time, 'monotonic', lambda: next(clock))
    with pytest.raises(RuntimeError):
        wait_original_gripper_endpoint(cycle, 'left', -.0199, 10.)
    assert cycle.sim == 10.


@pytest.mark.parametrize('maximum_sim,maximum_wall', [(2., 30.), (1., 31.)])
def test_feedback_wait_cannot_extend_its_declared_bounds(maximum_sim, maximum_wall):
    cycle, events = endpoint_fixture()
    with pytest.raises(ValueError):
        wait_original_gripper_endpoint(cycle, 'left', -.0199, 10., maximum_sim, maximum_wall)


def test_actual_inherited_bilateral_action_preserves_goals_and_waits_once_for_shared_result_epoch():
    cycle, goals = mocked_pair(endpoint_error=.0003)
    cycle.__class__ = NativeGripperCycle
    cycle.current_damping_empty_approach_completed = True
    cycle.joint_time = 9.99
    events = []
    cycle.record = lambda kind, value: events.append((kind, value))

    def spin():
        cycle.sim += .01
        cycle.joint_time = cycle.sim
        cycle.positions.update(dict.fromkeys(GRIPPER_JOINTS, -.0199+.0001))

    cycle.spin = spin
    DualCycle.paired_grippers(cycle, -.0199, 0, 'paired_load_release_above_bucket')
    assert cycle.sim == pytest.approx(10.01) and len(goals) == 2
    passed = [value for kind, value in events if kind == 'original_gripper_endpoint_fresh_feedback_pass']
    assert len(passed) == 2 and {value['successful_result_clock_sim'] for value in passed} == {10.}
    for goal in goals.values():
        assert goal.trajectory.header.stamp.nanosec == 160000000
        assert goal.trajectory.points[-1].time_from_start.nanosec == 250000000
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
    assert not cycle.pair_goal_handles and not cycle.pair_goal_futures


def test_original_unqualified_endpoint_gate_is_still_immediate_and_not_relaxed():
    cycle, goals = mocked_pair(endpoint_error=.0003)
    cycle.__class__ = NativeGripperCycle
    cycle.current_damping_empty_approach_completed = False
    with pytest.raises(AssertionError):
        DualCycle.paired_grippers(cycle, -.0199, 0, 'paired_load_release_above_bucket')


def test_recorded_previous_failure_really_contains_success_result_and_older_bad_native_frame():
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines()))
    source = PARENT / 'run/quarter_stroke_failure_window.json'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[str(source.relative_to(PARENT))]
    data = json.loads(source.read_text())
    result = next(row for row in data['events'] if row['kind']=='gripper_result'
                  and row['phase']=='paired_load_release_above_bucket' and row['payload']['side']=='left')
    frame = data['frames_before_failure'][-1]
    assert result['payload']['error_code'] == 0 and result['payload']['status'] == 4
    assert frame['source_sim'] == 55.72 and result['sim'] == 55.7295
    assert abs(frame['positions']['left_dex1_finger_2_joint']+.0199) > .00025


def test_final_home_planning_rates_and_short_stroke_policy_are_not_changed_again():
    assert NativeGripperCycle.press is ScheduledGripperCycle.press
    assert NativeGripperCycle.actual_plan is ScheduledGripperCycle.actual_plan
    assert NativeGripperCycle.future_start_offset is ScheduledGripperCycle.future_start_offset
    assert NativeGripperCycle.qualified_short_stroke_last_time is ScheduledGripperCycle.qualified_short_stroke_last_time
