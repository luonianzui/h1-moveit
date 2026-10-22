import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from dual_motion import common_arm_timing
from gentle_setdown_cycle import COARSE_PHASE, COARSE_RATE, GentleSetdownCycle
from inherited_source_identity import restore_inherited_source
from native_gripper_cycle import NativeGripperCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_dense_shortcut_full_20261014_r271'


def real_failure_command():
    assert hashlib.sha256((PARENT / 'SHA256SUMS').read_bytes()).hexdigest() == '78ba3a51efb0058b5ccc1fbbc9e95c5d1d4d953bead81d72b56cfb5cb1ce0daf'
    manifest = {relative.lstrip('*'): checksum for checksum, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    relative = 'run/native_setdown_failure_window.json'
    source = PARENT / relative
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[relative]
    window = json.loads(source.read_text())
    assert window['fault']['payload']['slip_m'] == .0020153168604874153
    assert window['fault']['phase'] == COARSE_PHASE
    return window['commands'][-1]['payload']


def gentle_cycle(loaded=True):
    cycle = actual_publisher_cycle(loaded=loaded)
    cycle.__class__ = GentleSetdownCycle
    return cycle


def assert_actual_messages(cycle, initial, samples):
    expected = common_arm_timing(initial, samples, COARSE_RATE)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert cycle.motion_count == len(messages) == 3
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 160000000)}
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .020 - 1e-9
        for point, source in zip(message.points, expected['points']):
            assert list(point.positions) == [source[name] for name in message.joint_names]
            assert not point.velocities and not point.accelerations and not point.effort
    command = next(value for kind, value in cycle.events if kind == 'command')
    assert command['actual_rate_rad_s'] == .30 and command['samples'] == samples
    assert command['holders'] == cycle.holders.snapshot()


def test_actual_three_publisher_dispatch_uses_point_three_only_for_coarse_setdown():
    cycle = gentle_cycle()
    initial, samples = dict(cycle.positions), waypoints(cycle)
    assert cycle.coarse_setdown_rate() == .30
    assert cycle.maximum_loaded_rate(COARSE_PHASE) == .30
    cycle.move(samples, COARSE_PHASE, cycle.coarse_setdown_rate())
    assert_actual_messages(cycle, initial, samples)


def test_original_failed_path_dispatch_preserves_all_targets_actual_initial_and_floor():
    command = real_failure_command()
    cycle = gentle_cycle()
    cycle.positions.update(command['initial'])
    initial = dict(cycle.positions)
    cycle.move(command['samples'], COARSE_PHASE, cycle.coarse_setdown_rate())
    assert_actual_messages(cycle, initial, command['samples'])
    original = common_arm_timing(initial, command['samples'], .35)
    current = common_arm_timing(initial, command['samples'], .30)
    assert original['times'] == command['times']
    assert current['points'] == original['points'] and current['groups'] == original['groups']
    assert abs(current['duration_sim_s'] - original['duration_sim_s'] - .6535041783140398) < 1e-12


@pytest.mark.parametrize('invalid', ['unqualified', 'empty', 'single', 'duplicate'])
def test_original_current_pair_qualification_cannot_be_bypassed(invalid):
    cycle = gentle_cycle()
    if invalid == 'unqualified':
        cycle.current_damping_empty_approach_completed = False
    elif invalid == 'empty':
        cycle.holders.rows.clear()
    elif invalid == 'single':
        cycle.holders.rows.pop('right')
    else:
        cycle.holders.rows['right']['payload'] = 'left'
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), COARSE_PHASE, .30)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('rate', [.20, .35, .36, 0., -1., math.nan, math.inf])
def test_wrong_rate_is_rejected_before_any_physical_publisher(rate):
    cycle = gentle_cycle()
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), COARSE_PHASE, rate)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase,rate', [('paired_retrieval_pilot_lift', .08), ('paired_retrieval_lift', .4),
                                      ('paired_retrieval_setdown_fine', .09), ('paired_load_forward_inside_basket_raise', .14)])
def test_every_other_loaded_phase_is_identical_to_the_parent_policy(phase, rate):
    cycle = gentle_cycle()
    parent = actual_publisher_cycle()
    assert cycle.motion_limits(phase, rate) == parent.motion_limits(phase, rate)
    assert cycle.maximum_loaded_rate(phase) == parent.maximum_loaded_rate(phase)


def test_original_native_guard_feedback_and_complete_lifecycle_are_not_overridden():
    for name in ('latest_native_guard', 'safety', 'capture_pair', 'paired_grippers',
                 'require_gripper_endpoint', 'retrieve', 'run_complete', 'fine_setdown_rate'):
        assert getattr(GentleSetdownCycle, name) is getattr(NativeGripperCycle, name)
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    for name in ('execute_cycle.py', 'execute_dual.py', 'dual_worker.py', 'complete_rate_profile.py'):
        current = restore_inherited_source(name, (EVIDENCE / name).read_text())
        assert hashlib.sha256(current.encode()).hexdigest() == records[name]['derived_sha256']
