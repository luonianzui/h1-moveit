import inspect

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from retrieval_worker import RetrievalPlanner


@pytest.mark.parametrize('hand', ['left', 'right'])
@pytest.mark.parametrize('rate,expected', [(.20, .10), (.08, .08)])
def test_actual_retrieval_transfer_rate_is_halved_without_changing_path(monkeypatch, hand, rate, expected):
    cycle = FullCycle.__new__(FullCycle)
    samples = [dict(torso_joint=0.0), dict(torso_joint=.1)]
    cycle.record = lambda kind, payload: None
    calls = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, path, stage, speed: calls.append((path, stage, speed)))
    phase = hand + '_retrieval_transfer'
    cycle.move(samples, phase, rate)
    assert calls == [(samples, phase, expected)]
    assert calls[0][0] is samples


def test_carry_does_not_use_unconstrained_rrt():
    source = inspect.getsource(RetrievalPlanner.transport)
    assert 'transfer = free_path' not in source
    assert 'transfer = self.cartesian' in source
    assert 'self.checked_path(transfer, carry_reference=reference)' in source
    assert 'self.checked_path(setdown, table_endpoint=True, carry_reference=reference)' in source


def test_carry_guard_checks_body_orientation_on_dense_fcl_samples():
    source = inspect.getsource(RetrievalPlanner.checked_path)
    assert '/ .002' in source
    assert 'expected_body[2, 2] <= math.cos(math.radians(2))' in source
    assert 'Commanded carry payload tilt exceeds 2 degrees' in source


def test_held_payload_uses_user_authorized_five_degree_policy_with_two_mm_slip():
    source = inspect.getsource(FullCycle.safety)
    assert 'grip_exceeds_limits(max(slip, native_slip), max(angle, native_angle))' in source
    from grip_limits import CARRY_ANGLE_LIMIT_DEG, CARRY_SLIP_LIMIT_M
    assert CARRY_ANGLE_LIMIT_DEG == 5 and CARRY_SLIP_LIMIT_M == .002


def test_delivery_goal_is_declared_before_motion_and_release():
    source = inspect.getsource(FullCycle.plan_retrieval)
    assert "self.record('return_table_target'" in source
    assert "result['return_table_yaw_rad'] - yaw" in source
    assert "grip_angle_limit_deg=CARRY_ANGLE_LIMIT_DEG" in source

