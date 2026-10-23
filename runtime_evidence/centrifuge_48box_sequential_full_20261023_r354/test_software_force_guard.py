from types import SimpleNamespace

import pytest

from execute_loading import LoadingCycle
from pickup_approach_cycle import PickupApproachCycle


def feedback(effort):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
                           reference=SimpleNamespace(positions=[-.01901]*2, effort=[effort]*2),
                           feedback=SimpleNamespace(positions=[-.01901]*2),
                           output=SimpleNamespace(positions=[], effort=[effort]*2))


@pytest.mark.parametrize('effort', [16., 18., 18.01])
def test_original_actual_output_callback_accepts_only_original_bounded_force(effort):
    cycle = PickupApproachCycle.__new__(PickupApproachCycle)
    cycle.last_gripper_record = dict(left=0.)
    cycle.record = lambda kind, payload: None
    cycle.receive_gripper_controller('left', feedback(effort))
    assert PickupApproachCycle.receive_gripper_controller is LoadingCycle.receive_gripper_controller


@pytest.mark.parametrize('effort', [18.011, 20., float('inf'), float('nan')])
def test_original_actual_output_callback_rejects_twenty_N_without_relaxation(effort):
    cycle = PickupApproachCycle.__new__(PickupApproachCycle)
    cycle.last_gripper_record = dict(left=0.)
    cycle.record = lambda kind, payload: None
    with pytest.raises(RuntimeError, match='exceeds 18 N'):
        cycle.receive_gripper_controller('left', feedback(effort))
