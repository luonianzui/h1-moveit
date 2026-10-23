import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from execute_dual import DualCycle
from ramped_button_cycle import RampedButtonCycle
from scheduled_gripper_cycle import ScheduledGripperCycle
from inherited_source_identity import restore_inherited_source
from test_complete_rate_profile import actual_publisher_cycle, button_cycle, waypoints
from test_zero_effort_timing import mocked_pair
from zero_effort_policy import GRIPPER_JOINTS, SHORT_STROKE_PHASES


EVIDENCE = Path(__file__).resolve().parent


@pytest.mark.parametrize('name', ['execute_fast.py', 'execute_dual.py', 'execute_cycle.py', 'efficient_gripper_cycle.py'])
def test_only_new_scheduling_hooks_reverse_exactly_to_qualified_parent_source(name):
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    restored = restore_inherited_source(name, (EVIDENCE / name).read_text())
    assert hashlib.sha256(restored.encode()).hexdigest() == records[name]['derived_sha256']


@pytest.mark.parametrize('qualified,offset', [(False, .3), (True, .16)])
def test_three_actual_physical_arm_publishers_keep_common_headers_points_and_rates(qualified, offset):
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = ScheduledGripperCycle
    cycle.current_damping_empty_approach_completed = qualified
    phase = 'paired_retrieval_approach' if qualified else 'paired_load_approach'
    cycle.move(waypoints(cycle), phase, .8)
    starts = {(message.header.stamp.sec, message.header.stamp.nanosec)
              for publisher in cycle.publishers.values() for message in publisher.messages}
    assert starts == {(10, round(offset * 1e9))}
    assert cycle.motion_count == 3


@pytest.mark.parametrize('qualified,offset', [(False, .3), (True, .16)])
def test_actual_button_fixed_contact_and_explicit_scheduled_stop_are_preserved(qualified, offset):
    cycle = button_cycle(qualified)
    cycle.__class__ = ScheduledGripperCycle
    points = [[0.] * 7, [.12] * 7, [.24] * 7]
    assert cycle.move_button('right', points, fixed_interval=.1) == 10 + offset
    message = cycle.publishers['right_arm'].messages[-1]
    assert [list(point.positions) for point in message.points] == points
    assert np.allclose([point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                        for point in message.points], [.05, .07, .09], atol=1e-9, rtol=0)
    cycle.sim = cycle.joint_time = 10.
    assert cycle.move_button('right', points, fixed_interval=.1, scheduled=10.7) == 10.7


@pytest.mark.parametrize('phase', sorted(SHORT_STROKE_PHASES))
@pytest.mark.parametrize('qualified,last_time,offset', [(False, .5, .3), (True, .25, .16)])
def test_real_base_bilateral_action_only_retimes_original_qualified_short_zero_effort_strokes(phase, qualified, last_time, offset):
    cycle, goals = mocked_pair()
    cycle.__class__ = ScheduledGripperCycle
    cycle.current_damping_empty_approach_completed = qualified
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    DualCycle.paired_grippers(cycle, -.0199, 0, phase)
    assert set(goals) == {'left', 'right'} and cycle.motion_count == 2
    assert {(goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec)
            for goal in goals.values()} == {(10, round(offset * 1e9))}
    for goal in goals.values():
        first, last = goal.trajectory.points
        assert first.time_from_start.sec == 0 and first.time_from_start.nanosec == 50000000
        assert list(first.positions) == [-.01901, -.01901] and list(last.positions) == [-.0199, -.0199]
        assert list(first.effort) == [16., 16.] and list(last.effort) == [0., 0.]
        assert list(first.velocities) == list(last.velocities) == [0., 0.]
        assert last.time_from_start.sec + last.time_from_start.nanosec * 1e-9 == last_time
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
        assert goal.goal_time_tolerance.sec == 3
    event = next(payload for kind, payload in records if kind == 'zero_final_effort_short_stroke_retimed')
    assert event['actual_last_time_sim_s'] == last_time
    assert not cycle.pair_goal_handles and not cycle.pair_goal_futures


@pytest.mark.parametrize('invalid', ['long', 'grasp', 'holder', 'unknown_phase'])
def test_original_short_stroke_admission_and_original_grasp_timing_are_not_relaxed(invalid):
    cycle, goals = mocked_pair()
    cycle.__class__ = ScheduledGripperCycle
    cycle.current_damping_empty_approach_completed = True
    cycle.phase = 'paired_load_release_above_bucket'
    effort = 0
    if invalid == 'long':
        cycle.positions[GRIPPER_JOINTS[-1]] = -.018
    elif invalid == 'grasp':
        effort = 16
    elif invalid == 'holder':
        cycle.holders.rows['left'] = {'payload': 'left'}
    else:
        cycle.phase = 'unknown'
    if invalid == 'holder':
        with pytest.raises(ValueError):
            cycle.gripper_trajectory_points('left', -.0199, 'left_paired_empty', effort)
        return
    points = cycle.gripper_trajectory_points('left', -.0199, 'left_paired_empty', effort)
    assert points[-1].time_from_start.sec == 1 and points[-1].time_from_start.nanosec == 0


@pytest.mark.parametrize('fault', ['native_velocity', 'action_result', 'endpoint'])
def test_real_bilateral_action_keeps_original_native_result_and_endpoint_rejections(fault):
    cycle, goals = mocked_pair(endpoint_error=.0003 if fault == 'endpoint' else 0.,
                               result_error=1 if fault == 'action_result' else 0,
                               safety_fault=fault == 'native_velocity')
    cycle.__class__ = ScheduledGripperCycle
    cycle.current_damping_empty_approach_completed = True
    with pytest.raises((AssertionError, RuntimeError)):
        DualCycle.paired_grippers(cycle, -.0199, 0, 'paired_load_release_above_bucket')


def test_original_final_open_home_and_actual_retrieval_planner_are_inherited_without_failed_skip():
    assert ScheduledGripperCycle.press is RampedButtonCycle.press
    assert ScheduledGripperCycle.actual_plan is RampedButtonCycle.actual_plan
    assert not (EVIDENCE / 'direct_retrieval_cycle.py').exists()
    assert not (EVIDENCE / 'probe_direct_retrieval.py').exists()


def test_short_stroke_commanded_cubic_bound_is_small_but_not_actual_physical_certificate():
    assert 1.5 * .001 / (.25 - .05) == pytest.approx(.0075)
    assert .0075 < .2
