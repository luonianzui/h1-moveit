import inspect
from types import SimpleNamespace

import pytest

from alignment_scene import make_alignment_scene, ObservedPlanner
from dual_motion import HoldRegistry, TASK_JOINTS
from execute_dual import DualCycle
from loading_contact_guard import contact_violation


def test_actual_model_path_is_not_silently_discarded_by_observed_scene():
    assert inspect.signature(make_alignment_scene).parameters['robot_path'].default is None
    assert inspect.signature(ObservedPlanner).parameters['robot_path'].default is None


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def mocked_cycle(tracking_fault=False, endpoint_fault=False):
    cycle = DualCycle.__new__(DualCycle)
    cycle.phase = 'test'
    cycle.holders = HoldRegistry()
    cycle.sim = cycle.joint_time = 10.
    cycle.positions = dict.fromkeys(TASK_JOINTS, 0.)
    cycle.motion_count = 0
    cycle.publishers = {name: Publisher() for name in ('left_arm', 'right_arm', 'torso')}
    cycle.safety = lambda: None
    cycle.record = lambda kind, value: None

    def spin():
        cycle.sim += .025
        cycle.joint_time = cycle.sim
        cycle.positions['left_elbow_joint'] = .02 if tracking_fault else 0.
        if endpoint_fault and cycle.sim > 10.6:
            cycle.positions['left_elbow_joint'] = .001

    cycle.spin = spin
    return cycle


def test_three_physical_groups_receive_the_same_start_and_waypoint_times():
    cycle = mocked_cycle()
    samples = [dict(cycle.positions), dict(cycle.positions)]
    cycle.move(samples, 'paired_empty_test', .8)
    messages = [publisher.messages[0] for publisher in cycle.publishers.values()]
    assert cycle.motion_count == 3
    starts = {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages}
    assert starts == {(10, 300000000)}
    times = {tuple((point.time_from_start.sec, point.time_from_start.nanosec) for point in message.points) for message in messages}
    assert len(times) == 1
    names = {name for message in messages for name in message.joint_names}
    assert names == TASK_JOINTS


def test_original_tracking_limit_cannot_be_bypassed_by_paired_dispatch():
    cycle = mocked_cycle(tracking_fault=True)
    with pytest.raises(RuntimeError, match='original .015rad'):
        cycle.move([dict(cycle.positions), dict(cycle.positions)], 'paired_empty_test', .8)


def test_original_endpoint_precision_cannot_be_bypassed_by_paired_dispatch():
    cycle = mocked_cycle(endpoint_fault=True)
    with pytest.raises(RuntimeError, match='precision gate'):
        cycle.move([dict(cycle.positions), dict(cycle.positions)], 'paired_empty_test', .8)


@pytest.mark.parametrize('phase', ['paired_load_release_above_bucket', 'paired_load_drop_settling'])
def test_dual_normal_drop_uses_original_contact_window_not_a_global_relaxation(phase):
    pair = dict(first='box48_left::body', second='centrifuge::bucket_left::collision', depth=.0012, force=16.)
    assert contact_violation(pair, 'left', phase, 4., 5.) is None
    assert contact_violation(pair, 'left', phase, 6., 5.) == 'depth_exceeds_0.001'
    pair['second'] = 'centrifuge::bucket_right::collision'
    assert contact_violation(pair, 'left', phase, 4., 5.) == 'depth_exceeds_0.001'


def test_unrelated_or_finger_contact_never_gets_passive_drop_relaxation():
    pair = dict(first='box48_left::body', second='h1_2::left_dex1_finger_1_link', depth=.0012, force=16.)
    assert contact_violation(pair, 'left', 'paired_load_drop_settling', 4., 5.) == 'depth_exceeds_0.001'
