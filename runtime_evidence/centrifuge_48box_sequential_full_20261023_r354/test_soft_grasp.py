import inspect
import math

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from soft_grasp import soft_grasp_profile


def test_measured_initial_and_exact_final_targets_preserved():
    profile = soft_grasp_profile([-.0199, -.0198], -.01901, 0, 16)
    assert profile[0] == dict(time_s=.05, positions=[-.0199, -.0198], velocities=[0, 0], effort=[0, 0])
    assert profile[-1]['time_s'] == 6.05
    assert profile[-1]['positions'] == [-.01901] * 2
    assert profile[-1]['velocities'] == [0, 0] and profile[-1]['effort'] == [16] * 2
    assert len(profile) == 61


def test_symmetric_monotonic_effort_and_position_ramps():
    profile = soft_grasp_profile([-.0199, -.0199], -.01901, 0, 16)
    for previous, current in zip(profile, profile[1:]):
        assert current['time_s'] > previous['time_s']
        assert current['positions'][0] == current['positions'][1]
        assert current['effort'][0] == current['effort'][1]
        assert previous['effort'][0] <= current['effort'][0] <= 16
        assert previous['positions'][0] <= current['positions'][0] <= -.01901
        assert 0 <= current['velocities'][0] <= .000279
        assert (current['effort'][0] - previous['effort'][0]) / (current['time_s'] - previous['time_s']) <= 5.001


@pytest.mark.parametrize('arguments', [([0], 0, 0, 16), ([math.nan, 0], 0, 0, 16),
    ([0, 0], math.inf, 0, 16), ([0, 0], 0, -1, 16), ([0, 0], 0, 0, 19), ([0, 0], 0, 17, 16)])
def test_invalid_profiles_fail_closed(arguments):
    with pytest.raises(ValueError):
        soft_grasp_profile(*arguments)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_default_loading_trajectory_is_identical_and_short(side):
    cycle = FullCycle.__new__(FullCycle)
    names = [side + '_dex1_finger_' + str(number) + '_joint' for number in (1, 2)]
    cycle.positions = dict(zip(names, [-.0199, -.0198]))
    cycle.accepted_gripper_efforts = {side: 0}
    points = cycle.gripper_trajectory_points(side, -.01901, side + '_grasp', 16)
    assert len(points) == 2 and points[0].time_from_start.nanosec == 50000000
    assert points[1].time_from_start.sec == 1
    assert list(points[1].positions) == [-.01901] * 2 and list(points[1].effort) == [16] * 2
    assert cycle.gripper_action_timeout_s(side, side + '_grasp') == 90


@pytest.mark.parametrize('side', ['left', 'right'])
def test_retrieval_hook_emits_actual_ros_profile_only_during_grasp(side):
    cycle = FullCycle.__new__(FullCycle)
    names = [side + '_dex1_finger_' + str(number) + '_joint' for number in (1, 2)]
    cycle.positions = dict(zip(names, [-.0199, -.0198]))
    cycle.accepted_gripper_efforts = {side: 0}
    cycle.retrieval_closing = True
    events = []
    cycle.record = lambda kind, data: events.append((kind, data))
    points = cycle.gripper_trajectory_points(side, -.01901, side + '_grasp', 16)
    assert len(points) == 61 and points[-1].time_from_start.sec == 6
    assert points[-1].time_from_start.nanosec == 50000000
    assert list(points[-1].effort) == [16] * 2 and events[0][0] == 'retrieval_soft_grasp_profile'
    assert cycle.gripper_action_timeout_s(side, side + '_grasp') == 240
    with pytest.raises(AssertionError):
        cycle.gripper_trajectory_points(side, -.0199, side + '_release_on_table', 0)


def test_action_still_has_original_result_endpoint_and_hold_contract():
    source = inspect.getsource(LoadingCycle.gripper)
    assert 'self.gripper_trajectory_points' in source and 'self.gripper_action_timeout_s' in source
    assert "response.result.error_code == 0" in source and '.00025' in source
    assert 'self.accepted_gripper_efforts[side] = effort' in source
    source = inspect.getsource(FullCycle.grip_retrieval)
    assert 'finally:' in source and 'self.retrieval_closing = False' in source

