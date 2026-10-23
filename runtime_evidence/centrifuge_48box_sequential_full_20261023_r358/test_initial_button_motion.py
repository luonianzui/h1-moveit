import math
from types import SimpleNamespace

import numpy as np
import pytest
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

from initial_button_motion import InitialButtonMotionMixin, controller_sample, initial_withdraw_points


class Parent:
    def button_points_before_publish(self, points, fixed_interval):
        self.parent_calls.append(fixed_interval)
        return points


class Candidate(InitialButtonMotionMixin, Parent):
    pass


def candidate():
    instance = Candidate.__new__(Candidate)
    instance.phase = 'machine_preparation_open_withdraw'
    instance.current_damping_empty_approach_completed = False
    instance.holders = SimpleNamespace(rows={})
    instance.holding = instance.held_payload = None
    instance.parent_calls = []
    instance.events = []
    instance.button_controller_samples = 0
    instance.record = lambda kind, payload: instance.events.append((kind, payload))
    instance.button_noncontact_rate = lambda: .6
    return instance


def points():
    return [JointTrajectoryPoint(positions=[value] * 7) for value in (0., .3, -.2)]


def test_initial_withdraw_has_bounded_acceleration_and_exact_vertices():
    original = points()
    output, profile = initial_withdraw_points(original, .6)
    assert profile['maximum_acceleration_rad_s2'] == 2.
    assert profile['ramp_time_sim_s'] == .3
    for index, point in zip(profile['original_vertex_indices'], original):
        assert output[index].positions == point.positions
        assert list(output[index].velocities) == [0.] * 7
    times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in output]
    velocities = np.array([point.velocities for point in output])
    assert np.min(np.diff(times)) >= .02 - 1e-9
    assert np.max(np.abs(velocities)) <= .6 + 1e-12
    assert np.max(np.abs(np.diff(velocities, axis=0) / np.diff(times)[:, None])) <= 2. + 1e-6
    assert all(-.2 <= value <= .3 for point in output for value in point.positions)


def test_first_empty_withdraw_uses_profile_without_setting_grip_qualification():
    instance = candidate()
    output = instance.button_points_before_publish(points(), None)
    assert len(output) > 3 and not instance.parent_calls
    assert not instance.current_damping_empty_approach_completed
    assert instance.events[-1][1]['original_tracking_limit_rad'] == .15
    assert instance.events[-1][1]['original_terminal_limit_rad'] == .005


@pytest.mark.parametrize('change', ['contact', 'qualified', 'other_phase'])
def test_other_motion_is_delegated_unchanged(change):
    instance = candidate()
    interval = .02 if change == 'contact' else None
    if change == 'qualified':
        instance.current_damping_empty_approach_completed = True
    if change == 'other_phase':
        instance.phase = 'machine_close_withdraw'
    original = points()
    assert instance.button_points_before_publish(original, interval) is original
    assert instance.parent_calls == [interval] and not instance.events


@pytest.mark.parametrize('field,value', [('holders', SimpleNamespace(rows={'left': 'box48_left'})),
                                        ('holding', 'left'), ('held_payload', 'right')])
def test_any_holder_rejects_initial_profile(field, value):
    instance = candidate()
    setattr(instance, field, value)
    with pytest.raises(ValueError, match='empty'):
        instance.button_points_before_publish(points(), None)


@pytest.mark.parametrize('rate', [math.nan, math.inf, 0., -.1, .61])
def test_invalid_rate_rejected(rate):
    instance = candidate()
    instance.button_noncontact_rate = lambda: rate
    with pytest.raises(ValueError, match='bounded'):
        instance.button_points_before_publish(points(), None)


def test_reference_observer_preserves_native_fields_and_is_bounded():
    message = JointTrajectoryControllerState()
    message.header.stamp.sec = 42
    message.header.stamp.nanosec = 123456789
    message.joint_names = ['left_elbow_joint']
    message.reference.positions = [.2]
    message.feedback.positions = [.05]
    message.error.positions = [.15]
    sample = controller_sample(message)
    assert sample['source_ns'] == 42123456789
    assert sample['reference']['positions'] == [.2]
    assert sample['feedback']['positions'] == [.05]
    instance = candidate()
    instance.button_controller_samples = 9999
    instance.observe_button_controller('left', message)
    instance.observe_button_controller('left', message)
    assert len(instance.events) == 1 and instance.button_controller_samples == 10000
    instance.button_controller_samples = 0
    instance.phase = 'seq_load_transport'
    instance.observe_button_controller('left', message)
    assert len(instance.events) == 1


def test_runtime_dispatches_smoothed_initial_withdraw_and_original_endpoint():
    from table_alignment_cycle import SpeedPriorityCycle
    from test_complete_rate_profile import button_cycle
    instance = button_cycle(False)
    instance.__class__ = SpeedPriorityCycle
    instance.phase = 'machine_preparation_open_withdraw'
    original = [[0.] * 7, [.12] * 7, [.24] * 7]
    instance.move_button('right', original)
    message = instance.publishers['right_arm'].messages[0]
    assert len(message.points) > len(original)
    assert list(message.points[-1].positions) == original[-1]
    assert instance.motion_count == 1
    assert any(kind == 'initial_button_withdraw_smoothing' for kind, value in instance.events)
    assert not any(kind.endswith('_fault') for kind, value in instance.events)
    assert all(abs(instance.positions[name] - .24) < .005 for name in instance.buttons['arms']['right']['joints'])
