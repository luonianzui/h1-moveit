import inspect
import math

import pytest

from efficient_gripper_cycle import EfficientGripperCycle
from execute_cycle import FullCycle
from leg_endpoint import LegEndpointGate
from leg_posture import LEG_JOINTS
from zero_effort_policy import GRIPPER_JOINTS


def test_exact_r260_boundary_needs_newer_native_feedback_without_relaxing_precision():
    gate = LegEndpointGate([0.] * 12, 8.92, 8.93)
    actual = dict.fromkeys(LEG_JOINTS, 0.)
    actual['left_ankle_pitch_joint'] = -.0010152535721841272
    assert not gate.observe(8.92, 8.93, actual)
    actual['left_ankle_pitch_joint'] = -.0009994157242448481
    assert not gate.observe(8.92, 8.94, actual)
    assert gate.observe(8.93, 8.94, actual)
    actual['left_ankle_pitch_joint'] = -.001
    assert not gate.observe(8.94, 8.95, actual)


def test_unsettled_feedback_retains_three_sim_second_timeout():
    gate = LegEndpointGate([0.] * 12, 8.92, 8.93)
    with pytest.raises(TimeoutError):
        gate.observe(12., 11.931, dict.fromkeys(LEG_JOINTS, 0.))


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf])
def test_nonfinite_feedback_and_position_cannot_qualify(invalid):
    gate = LegEndpointGate([0.] * 12, 8.92, 8.93)
    actual = dict.fromkeys(LEG_JOINTS, 0.)
    with pytest.raises(ValueError):
        gate.observe(invalid, 8.94, actual)
    actual[LEG_JOINTS[0]] = invalid
    with pytest.raises(ValueError):
        gate.observe(8.93, 8.94, actual)


def test_runtime_declares_five_mrad_stationary_precision_and_original_one_second_hold():
    method = inspect.getsource(FullCycle.initialize_leg_posture)
    assert 'response.error_code == 0' in method
    assert 'endpoint.observe(self.joint_time, self.sim, self.positions), timeout_s=30' in method
    assert 'max(abs(self.positions[name] - target)' in method and '< .005' in method
    assert 'self.dwell(1)' in method and 'last.time_from_start.sec = 1' in method


def test_explicit_two_mrad_stationary_gate_does_not_change_legacy_default():
    actual = dict.fromkeys(LEG_JOINTS, .0015)
    legacy = LegEndpointGate([0.] * 12, 1., 1.)
    candidate = LegEndpointGate([0.] * 12, 1., 1., position_limit_rad=.002)
    assert not legacy.observe(1.1, 1.1, actual)
    assert candidate.observe(1.1, 1.1, actual)
    with pytest.raises(ValueError):
        LegEndpointGate([0.] * 12, 1., 1., position_limit_rad=.003)


def test_inherited_gripper_interface_defaults_undefined_retrieval_flag_without_retiming():
    cycle = EfficientGripperCycle.__new__(EfficientGripperCycle)
    cycle.phase = 'machine_preparation'
    cycle.positions = dict.fromkeys(GRIPPER_JOINTS, -.019)
    cycle.holders = type('EmptyHolders', (), {'rows': {}})()
    cycle.accepted_gripper_efforts = dict(left=0., right=0.)
    points = cycle.gripper_trajectory_points('left', -.0199, 'left_paired_empty', 0)
    assert len(points) == 2 and points[-1].time_from_start.sec == 1
