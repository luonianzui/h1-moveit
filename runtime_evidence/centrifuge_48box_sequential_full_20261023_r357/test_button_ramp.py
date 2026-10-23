import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from button_ramp import INTERPOLATION_GATE, ramp_button_positions, require_spline_output, restore_button_ramp_source
from execute_fast import BUTTON_CONTACT_INTERVAL
from ramped_button_cycle import RampedButtonCycle
from inherited_source_identity import restore_inherited_source
from test_complete_rate_profile import button_cycle


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_complete_rate_profile_20261014_r262'


def cubic_sample(first, last, fraction):
    interval = last['time_s'] - first['time_s']
    begin, end = np.asarray(first['positions']), np.asarray(last['positions'])
    begin_speed, end_speed = np.asarray(first['velocities']), np.asarray(last['velocities'])
    return ((2 * fraction ** 3 - 3 * fraction ** 2 + 1) * begin
            + (fraction ** 3 - 2 * fraction ** 2 + fraction) * interval * begin_speed
            + (-2 * fraction ** 3 + 3 * fraction ** 2) * end
            + (fraction ** 3 - fraction ** 2) * interval * end_speed)


def actual_failed_command():
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines()))
    source = PARENT / 'run/button_velocity_failure_window.json'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[str(source.relative_to(PARENT))]
    data = json.loads(source.read_text())
    command = next(row for row in data['button_commands'] if row['kind'] == 'button_arm_command')
    return [row['positions'] for row in command['payload']['points']]


@pytest.mark.parametrize('points', [
    [[0.] * 7, [.00001] * 7, [.00002] * 7],
    [[0.] * 7, [.12] * 7, [.24] * 7],
    [[0.] * 7, [.125] * 7, [0.] * 7],
    [[0.] * 7, [0.] * 7, [0.] * 7]])
def test_ramp_floor_original_vertices_and_controller_cubic_stay_on_original_polyline(points):
    profile = ramp_button_positions(points)
    records = profile['points']
    assert records[0]['time_s'] == .05
    assert min(np.diff([row['time_s'] for row in records])) >= .02 - 1e-12
    for index, vertex in enumerate(profile['original_vertex_indices']):
        assert records[vertex]['positions'] == points[index]
        assert records[vertex]['velocities'] == [0.] * 7
    assert max(abs(value) for row in records for value in row['velocities']) <= 1.2 + 1e-12
    for segment, (begin, end) in enumerate(zip(profile['original_vertex_indices'], profile['original_vertex_indices'][1:])):
        original_begin, original_end = np.asarray(points[segment]), np.asarray(points[segment + 1])
        delta = original_end - original_begin
        for first, last in zip(records[begin:end], records[begin + 1:end + 1]):
            for fraction in np.linspace(0, 1, 21):
                actual = cubic_sample(first, last, fraction)
                if np.max(np.abs(delta)) == 0:
                    assert np.allclose(actual, original_begin, atol=1e-12, rtol=0)
                else:
                    axis = int(np.argmax(np.abs(delta)))
                    scalar = (actual[axis] - original_begin[axis]) / delta[axis]
                    assert -1e-12 <= scalar <= 1 + 1e-12
                    assert np.allclose(actual, original_begin + scalar * delta, atol=1e-12, rtol=0)
                reference = np.asarray(first['positions']) * (1 - fraction) + np.asarray(last['positions']) * fraction
                assert np.max(np.abs(actual - reference)) < .015


def test_actual_failed_path_gets_zero_velocity_at_the_problematic_corner_and_terminal():
    points = actual_failed_command()
    profile = ramp_button_positions(points)
    for original, index in zip(points, profile['original_vertex_indices']):
        assert profile['points'][index]['positions'] == original
        assert profile['points'][index]['velocities'] == [0.] * 7
    assert profile['maximum_acceleration_rad_s2'] == pytest.approx(12., abs=1e-12)
    assert 1.057972167399443 < profile['points'][-1]['time_s'] < 1.5619582510991643
    assert profile['original_joint_polyline_preserved']


@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf, 0., -1.])
def test_invalid_time_or_rate_parameters_fail_closed(invalid):
    with pytest.raises(ValueError):
        ramp_button_positions([[0.] * 7], maximum_rate=invalid)
    with pytest.raises(ValueError):
        ramp_button_positions([[0.] * 7], ramp_s=invalid)


@pytest.mark.parametrize('points', [[], [[0.] * 6], [[math.nan] * 7], [[math.inf] * 7]])
def test_incomplete_or_nonfinite_original_geometry_never_enters_a_command(points):
    with pytest.raises(ValueError):
        ramp_button_positions(points)


@pytest.mark.parametrize('rate,ramp,floor', [(1.21, .1, .02), (1.2, .01, .02), (1.2, .05, .02), (1.2, .1, .01)])
def test_declared_peak_acceleration_and_original_twenty_ms_floor_cannot_be_bypassed(rate, ramp, floor):
    with pytest.raises(ValueError):
        ramp_button_positions([[0.] * 7], maximum_rate=rate, ramp_s=ramp, minimum_interval=floor)


@pytest.mark.parametrize('output,code', [('String value is: none', 0), ('', 0), ('String value is: splines', 1)])
def test_unread_or_wrong_actual_controller_interpolation_is_refused(output, code):
    with pytest.raises(ValueError):
        require_spline_output(output, code)


def test_actual_spline_readback_is_required_not_assumed_from_default_yaml():
    assert require_spline_output('String value is: splines\n', 0)
    source = (EVIDENCE / 'execute_cycle.py').read_text()
    assert source.count(INTERPOLATION_GATE) == 1
    assert source.index(INTERPOLATION_GATE) < source.index("run/execution.claim")


def test_independent_controller_cubic_derivatives_preserve_declared_speed_and_acceleration_bounds():
    profile = ramp_button_positions(actual_failed_command())
    for first, last in zip(profile['points'], profile['points'][1:]):
        interval = last['time_s'] - first['time_s']
        begin, end = np.asarray(first['positions']), np.asarray(last['positions'])
        begin_speed, end_speed = np.asarray(first['velocities']), np.asarray(last['velocities'])
        for fraction in np.linspace(0, 1, 21):
            velocity = ((6 * fraction ** 2 - 6 * fraction) * begin / interval
                        + (3 * fraction ** 2 - 4 * fraction + 1) * begin_speed
                        + (-6 * fraction ** 2 + 6 * fraction) * end / interval
                        + (3 * fraction ** 2 - 2 * fraction) * end_speed)
            acceleration = ((12 * fraction - 6) * begin / interval ** 2
                            + (6 * fraction - 4) * begin_speed / interval
                            + (-12 * fraction + 6) * end / interval ** 2
                            + (6 * fraction - 2) * end_speed / interval)
            assert np.max(np.abs(velocity)) <= 1.2 + 1e-8
            assert np.max(np.abs(acceleration)) <= 12 + 1e-8


def test_real_inherited_button_publisher_transmits_ramped_velocities_and_tracks_actual_points():
    cycle = button_cycle()
    cycle.__class__ = RampedButtonCycle
    points = actual_failed_command()
    cycle.move_button('right', points)
    message = cycle.publishers['right_arm'].messages[0]
    profile = ramp_button_positions(points)
    assert len(message.points) == len(profile['points']) > len(points)
    for actual, expected in zip(message.points, profile['points']):
        assert list(actual.positions) == expected['positions']
        assert list(actual.velocities) == expected['velocities']
        stamp = actual.time_from_start.sec + actual.time_from_start.nanosec * 1e-9
        assert abs(stamp - expected['time_s']) < 1e-9
    command = next(value for kind, value in cycle.events if kind == 'button_arm_command')
    assert command['duration'] == pytest.approx(profile['points'][-1]['time_s'], abs=1e-9)
    assert [row['velocities'] for row in command['points']] == [row['velocities'] for row in profile['points']]
    assert cycle.motion_count == 1 and command['start'] == 10.3


@pytest.mark.parametrize('qualified,fixed', [(False, None), (True, .02), (False, .02)])
def test_current_qualification_and_fixed_contact_paths_never_use_the_new_ramp(qualified, fixed):
    cycle = button_cycle(qualified)
    cycle.__class__ = RampedButtonCycle
    points = [[0.] * 7, [.12] * 7, [.24] * 7]
    cycle.move_button('right', points, fixed_interval=fixed)
    message = cycle.publishers['right_arm'].messages[0]
    assert len(message.points) == 3 and all(not point.velocities for point in message.points)
    assert not any(kind == 'ramped_button_noncontact_profile' for kind, value in cycle.events)
    if fixed is not None:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, [.05, .05 + BUTTON_CONTACT_INTERVAL, .05 + 2 * BUTTON_CONTACT_INTERVAL])


def test_default_publication_hook_is_exactly_reversible_to_verified_parent_body():
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    original = restore_inherited_source('execute_fast.py', (EVIDENCE / 'execute_fast.py').read_text())
    assert hashlib.sha256(original.encode()).hexdigest() == records['execute_fast.py']['derived_sha256']
