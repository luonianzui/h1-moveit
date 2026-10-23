import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

from dual_motion import common_arm_timing
from inherited_source_identity import restore_inherited_source
from pickup_approach_cycle import PickupApproachCycle, PICKUP_PHASE
from retrieval_grip_cycle import RetrievalGripCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_retrieval_grip_full_20261014_r275'


def actual_failure_command():
    assert hashlib.sha256((PARENT / 'SHA256SUMS').read_bytes()).hexdigest() == 'd649ce4ffb8b6251eba595e2079dbe8d8dcf859cbc0f0ae5c2f1435c1ec71abf'
    manifest = {relative.lstrip('*'): checksum for checksum, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    path = PARENT / 'run/exact_velocity_fault_and_command.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path.relative_to(PARENT))]
    events = json.loads(path.read_text())['events']
    row = next(row for row in events if row['kind'] == 'command' and row['phase'] == PICKUP_PHASE)
    assert row['payload']['actual_rate_rad_s'] == .4 and row['payload']['holders'] == {}
    return row['payload']


def candidate(loaded=False):
    cycle = actual_publisher_cycle(loaded=loaded)
    cycle.__class__ = PickupApproachCycle
    cycle.current_damping_empty_approach_completed = True
    return cycle


@pytest.mark.parametrize('actual', [False, True])
def test_actual_three_group_publisher_preserves_all_points_and_original_floor(actual):
    cycle = candidate()
    if actual:
        command = actual_failure_command()
        cycle.positions.update(command['initial'])
        samples = command['samples']
    else:
        samples = waypoints(cycle)
    expected = common_arm_timing(cycle.positions, samples, .35)
    cycle.move(samples, PICKUP_PHASE, .8)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert len(messages) == cycle.motion_count == 3
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 160000000)}
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .020 - 1e-9
        for point, source in zip(message.points, expected['points']):
            assert list(point.positions) == [source[name] for name in message.joint_names]
    record = next(value for kind, value in cycle.events if kind == 'command')
    assert record['actual_rate_rad_s'] == .35 and record['samples'] == samples


@pytest.mark.parametrize('requested', [.4, .35, 0., -.1, math.nan, math.inf, .81])
def test_wrong_direct_rate_rejected_before_publishing(requested):
    cycle = candidate()
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), PICKUP_PHASE, requested)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('loaded', [False, True])
def test_unqualified_or_loaded_context_rejected_before_publishing(loaded):
    cycle = candidate(loaded)
    if not loaded:
        cycle.current_damping_empty_approach_completed = False
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), PICKUP_PHASE, .8)
    assert cycle.motion_count == 0


def test_every_other_method_and_runtime_source_preserves_parent_identity():
    for name in ('run_complete', 'retrieve', 'paired_grippers', 'gripper_trajectory_points',
                 'capture_pair', 'latest_native_guard', 'safety', 'require_gripper_endpoint', 'hold'):
        assert getattr(PickupApproachCycle, name) is getattr(RetrievalGripCycle, name)
    records = {row['relative']: row for row in json.loads((ROOT / 'derivation.json').read_text())['records']}
    for name in ('execute_cycle.py', 'execute_dual.py', 'retrieval_grip_cycle.py', 'dual_worker.py'):
        source = restore_inherited_source(name, (ROOT / name).read_text())
        assert hashlib.sha256(source.encode()).hexdigest() == records[name]['derived_sha256']
