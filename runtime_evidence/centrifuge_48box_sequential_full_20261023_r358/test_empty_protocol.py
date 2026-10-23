from endpoint_overlap_source import restore_endpoint_overlap_source
from pathlib import Path

import pytest

from coarse_ramp_cycle import CoarseRampCycle
from empty_protocol_cycle import EMPTY_IDLE_PHASES, EmptyProtocolCycle
from empty_protocol_source import restore_empty_protocol_source
from execute_dual import DualCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints
from test_retrieval_grip import candidate
from zero_effort_policy import GRIPPER_JOINTS


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_coarse_ramp_full_20261014_r279'


def fixture(**options):
    cycle, goals = candidate(**options)
    cycle.__class__ = EmptyProtocolCycle
    cycle.current_damping_empty_approach_completed = False
    cycle.waits = []
    cycle.dwell = cycle.waits.append
    return cycle, goals


def test_default_preserving_hooks_reverse_to_exact_parent_source():
    assert restore_empty_protocol_source(restore_endpoint_overlap_source((ROOT / 'execute_dual.py').read_text())) == (PARENT / 'execute_dual.py').read_text()
    for name in ('capture_pair', 'retrieve', 'safety', 'latest_native_guard', 'run_complete', 'hold', 'common_timing'):
        assert getattr(EmptyProtocolCycle, name) is getattr(CoarseRampCycle, name)


def test_original_bilateral_action_and_fresh_native_result_before_loading_plan():
    cycle, goals = fixture()
    cycle.joint_time = 9.99
    def spin():
        cycle.sim += .01
        cycle.joint_time = cycle.sim
    cycle.spin = spin
    cycle.prepare_loading_grippers()
    assert cycle.motion_count == 2 and set(goals) == {'left', 'right'}
    assert cycle.accepted_gripper_targets == dict(left=-.0199, right=-.0199)
    assert cycle.accepted_gripper_efforts == dict(left=0., right=0.)
    assert cycle.waits == [.1]
    passed = [value for kind, value in cycle.events if kind == 'original_gripper_endpoint_fresh_feedback_pass']
    assert len(passed) == 2 and {row['successful_result_clock_sim'] for row in passed} == {10.}
    assert all(row['source_sim'] > 10. and row['actual_maximum_error_m'] < .00025 for row in passed)
    for goal in goals.values():
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
        assert goal.goal_time_tolerance.sec == 3
        assert list(goal.trajectory.points[-1].positions) == [-.0199, -.0199]
        assert list(goal.trajectory.points[-1].effort) == [0., 0.]
        assert (goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec) == (10, 300000000)


def test_actual_initial_empty_accepted_registry_does_not_require_prior_dummy_action():
    cycle, goals = fixture()
    cycle.accepted_gripper_efforts = {}
    cycle.accepted_gripper_targets = {}
    cycle.prepare_loading_grippers()
    assert cycle.motion_count == 2 and cycle.waits == [.1]


def test_full_loading_entry_runs_fresh_open_before_any_actual_pickup_planning():
    cycle, goals = fixture()
    class Planned(Exception):
        pass
    def plan(kind, assignment):
        assert kind == 'load_pickup' and assignment == dict(left='left', right='right')
        assert cycle.motion_count == 2 and cycle.waits == [.1]
        assert len([row for row in cycle.events if row[0] == 'original_gripper_endpoint_fresh_feedback_pass']) == 2
        raise Planned()
    cycle.actual_plan = plan
    with pytest.raises(Planned):
        cycle.load_pair()
    assert not cycle.holders.rows


@pytest.mark.parametrize('options', [dict(result_error=1), dict(safety_fault=True), dict(endpoint_error=.00026)])
def test_original_failures_prevent_idle_and_planning(options):
    cycle, goals = fixture(**options)
    cycle.spin = lambda: setattr(cycle, 'sim', cycle.sim + .5)
    with pytest.raises((AssertionError, RuntimeError)):
        cycle.prepare_loading_grippers()
    assert not cycle.waits


@pytest.mark.parametrize('invalid', ['holder', 'preload'])
def test_nonempty_initial_state_rejected_before_any_goal(invalid):
    cycle, goals = fixture()
    if invalid == 'holder':
        cycle.holders.rows['left'] = dict(payload='left')
    else:
        cycle.accepted_gripper_efforts['right'] = 16.
    with pytest.raises(ValueError):
        cycle.prepare_loading_grippers()
    assert not goals


@pytest.mark.parametrize('phase', sorted(EMPTY_IDLE_PHASES))
def test_empty_idle_keeps_fresh_precision_and_original_native_velocity_guard(phase):
    cycle, goals = fixture()
    cycle.phase = phase
    cycle.current_damping_empty_approach_completed = True
    cycle.paired_grippers(-.019 if phase == 'machine_paired_empty_grippers' else -.0199, 0., phase)
    assert cycle.waits == [.1]
    assert len([row for row in cycle.events if row[0] == 'original_gripper_endpoint_fresh_feedback_pass']) == 2


@pytest.mark.parametrize('phase', ['paired_load_grasp', 'paired_load_release_above_bucket',
                                  'paired_retrieval_release_on_table', 'unknown'])
def test_loaded_release_and_unknown_idle_not_shortened(phase):
    cycle, goals = fixture()
    cycle.phase = phase
    assert cycle.paired_gripper_post_action_dwell() == .3


@pytest.mark.parametrize('invalid', ['holder', 'preload'])
def test_idle_admission_cannot_skip_loaded_registry(invalid):
    cycle, goals = fixture()
    cycle.phase = 'paired_load_open_empty'
    if invalid == 'holder':
        cycle.holders.rows['left'] = dict(payload='left')
    else:
        cycle.accepted_gripper_efforts['right'] = 16.
    with pytest.raises(ValueError):
        cycle.paired_gripper_post_action_dwell()


def test_point_four_empty_approach_three_actual_ros_messages_keep_original_geometry_and_floor():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EmptyProtocolCycle
    cycle.move(waypoints(cycle), 'paired_retrieval_approach', .8)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert len(messages) == 3
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 160000000)}
    command = next(value for kind, value in cycle.events if kind == 'command')
    assert command['actual_rate_rad_s'] == .4
    assert cycle.motion_limits('paired_retrieval_approach', .8) == (.4, .002)
    assert min(last-first for first,last in zip(command['times'], command['times'][1:])) >= .020-1e-9


@pytest.mark.parametrize('invalid', ['unqualified', 'holder', 'rate'])
def test_point_four_still_requires_original_empty_qualification(invalid):
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EmptyProtocolCycle
    rate = .8
    if invalid == 'unqualified':
        cycle.current_damping_empty_approach_completed = False
    elif invalid == 'holder':
        cycle.holders.rows['left'] = dict(payload='left')
    else:
        rate = .9
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_retrieval_approach', rate)
    assert cycle.motion_count == 0
