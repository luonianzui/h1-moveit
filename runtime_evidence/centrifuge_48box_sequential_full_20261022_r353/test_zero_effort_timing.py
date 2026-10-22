import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from dual_motion import HoldRegistry
from efficient_gripper_cycle import EfficientGripperCycle
from execute_dual import DualCycle
from execute_loading import GROUP_JOINTS
from gripper_damping_cycle import GripperDampingCycle
from test_loaded_dispatch import loaded_cycle
from inherited_source_identity import restore_inherited_source
from zero_effort_policy import GRIPPER_JOINTS, SHORT_STROKE_PHASES, restore_fine_setdown_source, short_stroke_last_time


EVIDENCE = Path(__file__).resolve().parent


@pytest.mark.parametrize('phase', sorted(SHORT_STROKE_PHASES))
def test_only_declared_small_actual_four_finger_strokes_are_shortened(phase):
    actual = dict.fromkeys(GRIPPER_JOINTS, -.01901)
    assert short_stroke_last_time(phase, -.0199, 0, actual, (), False) == .5
    actual[GRIPPER_JOINTS[-1]] = -.018
    assert short_stroke_last_time(phase, -.0199, 0, actual, (), False) is None


@pytest.mark.parametrize('effort', [1, 16, 18])
def test_preload_and_grasp_duration_cannot_be_shortened(effort):
    assert short_stroke_last_time('paired_load_open_empty', -.0199, effort,
                                  dict.fromkeys(GRIPPER_JOINTS, -.01901), (), False) is None


def test_unknown_phase_retrieval_grasp_and_live_holder_never_get_fast_release():
    actual = dict.fromkeys(GRIPPER_JOINTS, -.01901)
    assert short_stroke_last_time('unrecognized', -.0199, 0, actual, (), False) is None
    assert short_stroke_last_time('paired_retrieval_open_empty', -.0199, 0, actual, (), True) is None
    with pytest.raises(ValueError, match='qualified normal release'):
        short_stroke_last_time('paired_load_release_above_bucket', -.0199, 0, actual, ('left',), False)


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf])
def test_nonfinite_actual_finger_or_target_fails_closed(invalid):
    actual = dict.fromkeys(GRIPPER_JOINTS, -.01901)
    with pytest.raises(ValueError):
        short_stroke_last_time('paired_load_open_empty', invalid, 0, actual, (), False)
    actual[GRIPPER_JOINTS[-1]] = invalid
    with pytest.raises(ValueError):
        short_stroke_last_time('paired_load_open_empty', -.0199, 0, actual, (), False)


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


def mocked_pair(endpoint_error=0., result_error=0, safety_fault=False):
    cycle = EfficientGripperCycle.__new__(EfficientGripperCycle)
    cycle.sim = 10.
    cycle.positions = dict.fromkeys(GRIPPER_JOINTS, -.01901)
    cycle.holders = HoldRegistry()
    cycle.accepted_gripper_targets = dict(left=-.01901, right=-.01901)
    cycle.accepted_gripper_efforts = dict(left=16., right=16.)
    cycle.pair_goal_handles = {}
    cycle.pair_goal_futures = {}
    cycle.motion_count = 0
    cycle.retrieval_closing = False
    cycle.record = lambda kind, payload: None
    cycle.dwell = lambda duration: None
    cycle.spin = lambda: None
    goals = {}

    def safety():
        if safety_fault:
            raise RuntimeError('Original velocity fault')

    cycle.safety = safety

    def send(hand, goal):
        goals[hand] = goal

        def result():
            for name, value in zip(goal.trajectory.joint_names, goal.trajectory.points[-1].positions):
                cycle.positions[name] = value + endpoint_error
            return ImmediateFuture(SimpleNamespace(status=4, result=SimpleNamespace(
                error_code=result_error, error_string='rejected' if result_error else '')))

        return ImmediateFuture(SimpleNamespace(accepted=True, get_result_async=result))

    cycle.gripper_actions = {
        hand: SimpleNamespace(server_is_ready=lambda: True,
                              send_goal_async=lambda goal, hand=hand: send(hand, goal))
        for hand in ('left', 'right')
    }
    return cycle, goals


def test_real_base_paired_action_publishes_two_common_clock_ros_goals():
    cycle, goals = mocked_pair()
    DualCycle.paired_grippers(cycle, -.0199, 0, 'paired_load_release_above_bucket')
    assert set(goals) == {'left', 'right'}
    starts = {(goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec)
              for goal in goals.values()}
    assert starts == {(10, 300000000)}
    for hand, goal in goals.items():
        assert list(goal.trajectory.joint_names) == GROUP_JOINTS[hand + '_gripper']
        first, last = goal.trajectory.points
        assert list(first.positions) == [-.01901, -.01901]
        assert list(first.effort) == [16., 16.]
        assert first.time_from_start.nanosec == 50000000
        assert (last.time_from_start.sec, last.time_from_start.nanosec) == (0, 500000000)
        assert list(last.positions) == [-.0199, -.0199]
        assert list(last.velocities) == list(first.velocities) == [0., 0.]
        assert list(last.effort) == [0., 0.]
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
        assert goal.goal_time_tolerance.sec == 3
    assert cycle.motion_count == 2
    assert not cycle.pair_goal_handles and not cycle.pair_goal_futures


@pytest.mark.parametrize('options', [dict(endpoint_error=.00026), dict(result_error=1), dict(safety_fault=True)])
def test_original_actual_action_safety_result_and_endpoint_guards_cannot_be_bypassed(options):
    cycle, goals = mocked_pair(**options)
    with pytest.raises((AssertionError, RuntimeError)):
        DualCycle.paired_grippers(cycle, -.0199, 0, 'paired_load_release_above_bucket')
    if options.get('safety_fault'):
        assert not goals


def test_fine_rate_only_with_current_qualification_and_two_distinct_actual_holders():
    cycle = EfficientGripperCycle.__new__(EfficientGripperCycle)
    cycle.current_damping_empty_approach_completed = True
    cycle.holders = SimpleNamespace(rows=dict(left=dict(payload='right'), right=dict(payload='left')))
    assert cycle.fine_setdown_rate() == .06
    cycle.current_damping_empty_approach_completed = False
    with pytest.raises(ValueError):
        cycle.fine_setdown_rate()
    cycle.current_damping_empty_approach_completed = True
    cycle.holders.rows.pop('right')
    with pytest.raises(ValueError):
        cycle.fine_setdown_rate()


def test_base_fine_source_only_declared_default_hook_and_full_cycle_inherited():
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    source = restore_inherited_source('execute_dual.py', (EVIDENCE / 'execute_dual.py').read_text()).encode()
    assert hashlib.sha256(source).hexdigest() == records['execute_dual.py']['derived_sha256']
    assert 'fine_rate_rad_s=.03' in restore_fine_setdown_source(source.decode())
    assert EfficientGripperCycle.run_complete is GripperDampingCycle.run_complete
    assert EfficientGripperCycle.paired_grippers is DualCycle.paired_grippers
    assert EfficientGripperCycle.safety is GripperDampingCycle.safety


def test_real_base_fine_dispatch_still_publishes_three_common_clock_groups():
    cycle = loaded_cycle()
    cycle.__class__ = EfficientGripperCycle
    cycle.current_damping_empty_approach_completed = True
    cycle.move([dict(cycle.positions), dict(cycle.positions)],
               'paired_retrieval_setdown_fine', cycle.fine_setdown_rate())
    messages = [publisher.messages[0] for publisher in cycle.publishers.values()]
    assert len(messages) == cycle.motion_count == 3
    assert len({(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages}) == 1
    assert len({tuple((point.time_from_start.sec, point.time_from_start.nanosec)
                     for point in message.points) for message in messages}) == 1


@pytest.mark.parametrize('qualification,second_holder,rate', [(False, True, .06), (True, False, .06),
                                                             (True, True, .07)])
def test_direct_fine_dispatch_cannot_bypass_qualification_holder_or_rate(qualification, second_holder, rate):
    cycle = loaded_cycle()
    cycle.__class__ = EfficientGripperCycle
    cycle.current_damping_empty_approach_completed = qualification
    if not second_holder:
        cycle.holders.release('right')
    with pytest.raises(ValueError):
        cycle.move([dict(cycle.positions)], 'paired_retrieval_setdown_fine', rate)
    assert all(not publisher.messages for publisher in cycle.publishers.values())
