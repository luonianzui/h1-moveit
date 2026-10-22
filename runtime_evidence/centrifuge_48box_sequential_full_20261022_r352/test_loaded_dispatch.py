import numpy as np
import pytest

from dual_motion import TASK_JOINTS
from execute_dual import DualCycle
from handover_policy import FREE_PHASES
from stop_handover_cycle import StopHandoverCycle
from test_dual_executor import mocked_cycle


def loaded_cycle(subclass=True):
    cycle = mocked_cycle()
    if subclass:
        cycle.__class__ = StopHandoverCycle
    certificate = dict(source='gazebo_physics_contact', midband_verified=True,
                       normal_loads_N={'1': 10., '2': 10.})
    for hand in ('left', 'right'):
        cycle.holders.begin(hand, hand, np.eye(4), np.eye(4), 10., certificate)
    return cycle


@pytest.mark.parametrize('phase', sorted(FREE_PHASES))
def test_actual_base_dispatch_publishes_all_three_groups_for_authorized_profile(phase):
    cycle = loaded_cycle()
    samples = [dict(cycle.positions), dict(cycle.positions)]
    cycle.move(samples, phase, .25)
    assert cycle.motion_count == 3
    messages = [publisher.messages[0] for publisher in cycle.publishers.values()]
    assert len(messages) == 3
    assert {name for message in messages for name in message.joint_names} == TASK_JOINTS
    assert len({(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages}) == 1
    assert len({tuple((point.time_from_start.sec, point.time_from_start.nanosec)
                     for point in message.points) for message in messages}) == 1


def test_original_base_loaded_quarter_rate_is_still_enforced():
    cycle = loaded_cycle(subclass=False)
    with pytest.raises(ValueError, match='declared phase cap'):
        cycle.move([dict(cycle.positions)], 'paired_retrieval_transfer', .4)
    assert all(not publisher.messages for publisher in cycle.publishers.values())


def test_nonfree_loaded_phase_keeps_original_dispatch_cap():
    cycle = loaded_cycle()
    with pytest.raises(ValueError, match='declared phase cap'):
        cycle.move([dict(cycle.positions)], 'paired_unapproved_transport', .4)
    assert all(not publisher.messages for publisher in cycle.publishers.values())


def test_missing_actual_second_holder_cannot_authorize_speed_fork():
    cycle = loaded_cycle()
    cycle.holders.release('right')
    assert cycle.maximum_loaded_rate('paired_retrieval_transfer') == .25
    with pytest.raises(ValueError):
        cycle.move([dict(cycle.positions)], 'paired_retrieval_transfer', .25)
    assert all(not publisher.messages for publisher in cycle.publishers.values())


def test_declared_loaded_profile_cannot_be_exceeded():
    cycle = loaded_cycle()
    with pytest.raises(ValueError):
        cycle.move([dict(cycle.positions)], 'paired_retrieval_transfer', .8)
    assert all(not publisher.messages for publisher in cycle.publishers.values())
