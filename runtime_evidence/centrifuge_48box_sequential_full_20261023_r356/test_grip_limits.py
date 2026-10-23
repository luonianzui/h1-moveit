import inspect
import math

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from grip_limits import CARRY_ANGLE_LIMIT_DEG, CARRY_SLIP_LIMIT_M, grip_exceeds_limits


@pytest.mark.parametrize('angle', [0, .618, 2.0097, 4.999, -4.999])
def test_authorized_five_degree_carry_accepts_measured_failure_without_slip_relaxation(angle):
    assert not grip_exceeds_limits(.000268, angle)


@pytest.mark.parametrize('slip,angle', [(.002001, 0), (.001, 5), (.001, -5),
                                     (math.nan, 0), (0, math.inf), (-.001, 0)])
def test_loss_of_grip_and_nonfinite_still_fail_closed(slip, angle):
    assert grip_exceeds_limits(slip, angle)


def test_loading_and_retrieval_use_same_policy_not_different_hidden_thresholds():
    assert CARRY_ANGLE_LIMIT_DEG == 5 and CARRY_SLIP_LIMIT_M == .002
    loading = inspect.getsource(LoadingCycle.safety)
    retrieval = inspect.getsource(FullCycle.safety)
    assert 'grip_exceeds_limits(float(slip), math.degrees(angle))' in loading
    assert 'grip_exceeds_limits(native_slip, math.degrees(native_angle))' in loading
    assert 'grip_exceeds_limits(max(slip, native_slip), max(angle, native_angle))' in retrieval


def test_permission_records_actual_limit_but_final_orientation_is_not_relaxed():
    source = inspect.getsource(FullCycle.run_complete)
    assert 'carry_angle_limit_deg=CARRY_ANGLE_LIMIT_DEG' in source
    source = inspect.getsource(FullCycle.plan_retrieval)
    assert 'final_orientation_limit_deg=2' in source
