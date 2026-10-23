import numpy as np
import pytest

from execute_fast import FastCycle
from execute_loading import LoadingCycle


def cycle(holder=None, payload=None):
    instance = object.__new__(FastCycle)
    instance.holding, instance.held_payload = holder, payload
    instance.record = lambda kind, payload: None
    return instance


@pytest.mark.parametrize('phase,holder,payload,expected', [
    ('right_approach', None, None, .8), ('left_lift', 'left', None, .25),
    ('right_transfer', 'right', None, .25), ('left_pilot_lift', 'left', None, .06),
    ('right_descend', None, None, .36), ('left_retrieval_descend', None, None, .25),
    ('right_retrieval_transfer', 'right', 'left', .25),
    ('left_retrieval_pilot_lift', 'left', 'right', .03)])
def test_actual_dispatch_retimes_without_changing_points_or_base_guard(monkeypatch, phase, holder, payload, expected):
    observed = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, samples, phase, rate:
                        observed.append((samples, phase, rate)))
    samples = [{'joint':0.}, {'joint':.12}]
    cycle(holder, payload).move(samples, phase, .12)
    assert observed == [(samples, phase, expected)]


def test_final_setdown_keeps_original_slow_contact_segment(monkeypatch):
    import execute_fast
    samples = [{'joint':0.}, {'joint':.12}, {'joint':.15}]
    monkeypatch.setattr(execute_fast, 'braking_setdown_segments', lambda values:(values[:2], values[1:], 1))
    observed = []
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, points, phase, rate:
                        observed.append((points, phase, rate)))
    cycle('left', 'right').move(samples, 'left_retrieval_setdown', .12)
    assert observed[0][2] == .20 and observed[1][2] == .03
    assert observed[0][0] == samples[:2] and observed[1][0] == samples[1:]


def test_two_second_soft_closure_keeps_force_velocity_and_minimum_interval():
    instance = cycle('right', 'left')
    instance.retrieval_closing = True
    instance.positions = {'right_dex1_finger_1_joint':-.0199, 'right_dex1_finger_2_joint':-.0199}
    instance.accepted_gripper_efforts = {'right':0}
    points = instance.gripper_trajectory_points('right', -.01901, 'right_grasp', 16)
    assert len(points) == 61
    times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in points]
    assert abs(times[-1] - 2.05) < 1e-9 and min(np.diff(times)) >= .02
    assert list(points[-1].positions) == [-.01901, -.01901]
    assert list(points[-1].effort) == [16., 16.]
    assert all(0 <= value <= 18 for point in points for value in point.effort)
    assert all(abs(value) < .2 for point in points for value in point.velocities)
