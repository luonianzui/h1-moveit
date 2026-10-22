import hashlib
import json
from pathlib import Path

import pytest

from execute_fast import RETRIEVAL_SOFT_GRASP_SECONDS
from execute_loading import GROUP_JOINTS
from initial_approach_cycle import InitialApproachCycle
from inherited_source_identity import restore_inherited_source
from retrieval_grip_cycle import RetrievalGripCycle
from test_zero_effort_timing import mocked_pair


def candidate(**options):
    cycle, goals = mocked_pair(**options)
    cycle.__class__ = RetrievalGripCycle
    cycle.current_damping_empty_approach_completed = True
    cycle.accepted_gripper_efforts = dict(left=0., right=0.)
    cycle.positions = dict.fromkeys(cycle.positions, -.0199)
    cycle.joint_time = 10.1
    cycle.events = []
    cycle.record = lambda kind, value: cycle.events.append((kind, value))
    return cycle, goals


def test_actual_bilateral_ros_goals_keep_parent_geometry_clock_times_and_gates():
    cycle, goals = candidate()
    cycle.paired_grippers(-.01901, 16., 'paired_retrieval_grasp', retrieval=True)
    assert set(goals) == {'left', 'right'}
    assert cycle.accepted_gripper_efforts == dict(left=18., right=18.)
    assert {(goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec)
            for goal in goals.values()} == {(10, 160000000)}
    for hand, goal in goals.items():
        parent, unused = candidate()
        parent.__class__ = InitialApproachCycle
        parent.retrieval_closing = True
        parent.phase = 'paired_retrieval_grasp'
        original = parent.gripper_trajectory_points(hand, -.01901, hand + '_grasp', 16.)
        assert len(goal.trajectory.points) == len(original) == 61
        assert list(goal.trajectory.joint_names) == GROUP_JOINTS[hand + '_gripper']
        for point, previous in zip(goal.trajectory.points, original):
            assert point.positions == previous.positions and point.velocities == previous.velocities
            assert point.time_from_start == previous.time_from_start
            assert list(point.effort) == [value * 18. / 16. for value in previous.effort]
        last = goal.trajectory.points[-1]
        assert last.time_from_start.sec + last.time_from_start.nanosec * 1e-9 == RETRIEVAL_SOFT_GRASP_SECONDS + .05
        assert list(last.effort) == [18., 18.]
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
        assert goal.goal_time_tolerance.sec == 6
    assert cycle.motion_count == 2 and not cycle.retrieval_closing
    assert not cycle.pair_goal_handles and not cycle.pair_goal_futures
    assert len([row for row in cycle.events if row[0] == 'original_gripper_endpoint_fresh_feedback_pass']) == 2


@pytest.mark.parametrize('invalid', ['unqualified', 'holder', 'preload', 'value', 'effort', 'retrieval'])
def test_wrong_context_rejected_before_any_actual_action(invalid):
    cycle, goals = candidate()
    value, effort, retrieval = -.01901, 16., True
    if invalid == 'unqualified':
        cycle.current_damping_empty_approach_completed = False
    elif invalid == 'holder':
        cycle.holders.rows['left'] = dict(payload='left')
    elif invalid == 'preload':
        cycle.accepted_gripper_efforts['right'] = 16.
    elif invalid == 'value':
        value = -.019
    elif invalid == 'effort':
        effort = 20.
    else:
        retrieval = False
    with pytest.raises(ValueError):
        cycle.paired_grippers(value, effort, 'paired_retrieval_grasp', retrieval=retrieval)
    assert not goals and cycle.motion_count == 0


@pytest.mark.parametrize('options', [dict(result_error=1), dict(safety_fault=True)])
def test_original_actual_action_result_and_safety_faults_still_reject(options):
    cycle, goals = candidate(**options)
    with pytest.raises((AssertionError, RuntimeError)):
        cycle.paired_grippers(-.01901, 16., 'paired_retrieval_grasp', retrieval=True)


def test_original_native_endpoint_failure_not_relaxed(monkeypatch):
    cycle, goals = candidate(endpoint_error=.00026)
    cycle.spin = lambda: setattr(cycle, 'sim', cycle.sim + .5)
    with pytest.raises(RuntimeError, match='original .00025m'):
        cycle.paired_grippers(-.01901, 16., 'paired_retrieval_grasp', retrieval=True)


def test_loading_and_release_remain_parent_actions():
    for phase, value, effort in [('paired_load_grasp', -.01901, 16.),
                                 ('paired_retrieval_open_empty', -.0199, 0.)]:
        cycle, goals = candidate()
        parent, previous_goals = candidate()
        parent.__class__ = InitialApproachCycle
        cycle.paired_grippers(value, effort, phase)
        parent.paired_grippers(value, effort, phase)
        assert goals == previous_goals
        assert cycle.accepted_gripper_efforts == parent.accepted_gripper_efforts


def test_full_lifecycle_and_physical_guards_are_identical_parent_methods():
    for name in ('run_complete', 'retrieve', 'capture_pair', 'latest_native_guard', 'safety',
                 'require_gripper_endpoint', 'motion_limits', 'hold', 'coarse_setdown_rate'):
        assert getattr(RetrievalGripCycle, name) is getattr(InitialApproachCycle, name)
    root = Path(__file__).resolve().parent
    records = {row['relative']: row for row in json.loads((root / 'derivation.json').read_text())['records']}
    for name in ('execute_cycle.py', 'execute_dual.py', 'execute_fast.py', 'execute_loading.py', 'soft_grasp.py'):
        restored = restore_inherited_source(name, (root / name).read_text())
        assert hashlib.sha256(restored.encode()).hexdigest() == records[name]['derived_sha256']
