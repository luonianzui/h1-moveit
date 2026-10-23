import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from dual_motion import common_arm_timing
from gentle_setdown_cycle import GentleSetdownCycle
from initial_approach_cycle import APPROACH_PHASE, APPROACH_RATE, InitialApproachCycle
from inherited_source_identity import restore_inherited_source
from test_complete_rate_profile import actual_publisher_cycle, waypoints


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_gentle_setdown_20261014_r272'


def actual_failed_approach():
    assert hashlib.sha256((PARENT / 'SHA256SUMS').read_bytes()).hexdigest() == 'ad16d061304cc7fafdd45ba7029098cb70705112a4cd94658444071720b22e83'
    manifest = {relative.lstrip('*'): checksum for checksum, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    path = PARENT / 'run/recorded_motion_commands.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path.relative_to(PARENT))]
    commands = json.loads(path.read_text())
    assert len(commands) == 1 and commands[0]['phase'] == APPROACH_PHASE
    command = commands[0]['payload']
    assert command['actual_rate_rad_s'] == .8 and command['holders'] == {}
    return command


def candidate_cycle():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = InitialApproachCycle
    cycle.current_damping_empty_approach_completed = False
    return cycle


@pytest.mark.parametrize('actual_failure', [False, True])
def test_actual_three_publisher_dispatch_keeps_geometry_common_clock_and_floor(actual_failure):
    cycle = candidate_cycle()
    if actual_failure:
        command = actual_failed_approach()
        cycle.positions.update(command['initial'])
        samples = command['samples']
    else:
        samples = waypoints(cycle)
    initial = dict(cycle.positions)
    expected = common_arm_timing(initial, samples, APPROACH_RATE)
    cycle.move(samples, APPROACH_PHASE, .8)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert cycle.motion_count == len(messages) == 3
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 300000000)}
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .020 - 1e-9
        for point, source in zip(message.points, expected['points']):
            assert list(point.positions) == [source[name] for name in message.joint_names]
    recorded = next(value for kind, value in cycle.events if kind == 'command')
    assert recorded['samples'] == samples and recorded['actual_rate_rad_s'] == .7
    assert recorded['holders'] == {} and cycle.current_damping_empty_approach_completed


@pytest.mark.parametrize('requested', [.7, 0., -1., math.inf, math.nan, .81, .6])
def test_wrong_requested_rate_rejected_before_any_publisher(requested):
    cycle = candidate_cycle()
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), APPROACH_PHASE, requested)
    assert cycle.motion_count == 0


def test_held_payload_cannot_use_empty_approach_retiming():
    cycle = actual_publisher_cycle(loaded=True)
    cycle.__class__ = InitialApproachCycle
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), APPROACH_PHASE, .8)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase,rate', [('paired_retrieval_approach', .8), ('paired_load_descend', .8),
                                      ('paired_retrieval_setdown', .30), ('paired_retrieval_setdown_fine', .09)])
def test_other_phases_identical_to_parent(phase, rate):
    loaded = phase in ('paired_retrieval_setdown', 'paired_retrieval_setdown_fine')
    cycle = actual_publisher_cycle(loaded=loaded)
    cycle.__class__ = InitialApproachCycle
    parent = actual_publisher_cycle(loaded=loaded)
    parent.__class__ = GentleSetdownCycle
    assert cycle.motion_limits(phase, rate) == parent.motion_limits(phase, rate)


def test_original_complete_lifecycle_and_native_guards_are_inherited():
    for name in ('safety', 'latest_native_guard', 'capture_pair', 'paired_grippers',
                 'require_gripper_endpoint', 'retrieve', 'run_complete', 'coarse_setdown_rate'):
        assert getattr(InitialApproachCycle, name) is getattr(GentleSetdownCycle, name)


def test_parent_tracking_limit_and_finger_guard_not_relaxed():
    cycle = candidate_cycle()
    parent = candidate_cycle()
    parent.__class__ = GentleSetdownCycle
    assert cycle.motion_limits(APPROACH_PHASE, .8)[1] == parent.motion_limits(APPROACH_PHASE, .8)[1]
    source = (EVIDENCE / 'execute_loading.py').read_text()
    assert 'Effort gripper velocity fault' in source


def test_only_latest_runtime_alias_requires_source_inverse():
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    for name in ('execute_cycle.py', 'execute_dual.py', 'dual_worker.py', 'gentle_setdown_cycle.py'):
        restored = restore_inherited_source(name, (EVIDENCE / name).read_text())
        assert hashlib.sha256(restored.encode()).hexdigest() == records[name]['derived_sha256']
