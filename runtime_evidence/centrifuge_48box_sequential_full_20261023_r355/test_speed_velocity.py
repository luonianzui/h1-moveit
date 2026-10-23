import math
from types import SimpleNamespace

import pytest

from execute_loading import LoadingCycle, velocity_violation
from sequential_speed_cycle import SpeedPriorityCycle
from speed_policy import gripper_velocity_violation


@pytest.mark.parametrize('value', [0., .2, -.2, .23181895512792153, -.59, .6])
def test_explicit_new_velocity_limit(value):
    assert gripper_velocity_violation(value) is None


@pytest.mark.parametrize('value', [.600001, -.600001, math.nan, math.inf, -math.inf])
def test_still_rejects_excess_or_nonfinite(value):
    assert gripper_velocity_violation(value) is not None


def callback_cycle(override):
    records = []
    cycle = SimpleNamespace(positions={}, gripper_velocities={}, latest={}, velocity_fault=None,
                            sim=None, last_record=-1., record=lambda kind, data: records.append((kind, data)))
    if override:
        cycle.gripper_velocity_violation = lambda value: SpeedPriorityCycle.gripper_velocity_violation(cycle, value)
    return cycle, records


def message(value):
    return SimpleNamespace(name=['right_dex1_finger_2_joint'], position=[-.0199], velocity=[value],
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)))


def test_callback_uses_new_monitor_without_modifying_actual_feedback_or_old_default():
    cycle, records = callback_cycle(True)
    LoadingCycle.receive_joints(cycle, message(.23181895512792153))
    assert cycle.velocity_fault is None
    assert records[0][1]['velocities']['right_dex1_finger_2_joint'] == .23181895512792153
    original, unused = callback_cycle(False)
    LoadingCycle.receive_joints(original, message(.23181895512792153))
    assert original.velocity_fault['reason'] == 'exceeds_model_0.2_m_s'
    assert velocity_violation(.23181895512792153) == 'exceeds_model_0.2_m_s'
    LoadingCycle.receive_joints(cycle, message(.61))
    assert cycle.velocity_fault['reason'] == 'exceeds_relaxed_simulation_0.6_m_s'
    assert any(kind == 'velocity_fault_sample' for kind, data in records)
