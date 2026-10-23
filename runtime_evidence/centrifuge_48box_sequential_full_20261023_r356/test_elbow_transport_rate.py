import json
from pathlib import Path

import numpy as np
import pytest

from dual_motion import TASK_JOINTS, common_arm_timing
from elbow_transport_source import restore_elbow_transport
from empty_protocol_cycle import EmptyProtocolCycle
from endpoint_overlap_cycle import EndpointOverlapCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r288'


def candidate():
    cycle = actual_publisher_cycle()
    cycle.__class__ = EndpointOverlapCycle
    cycle.elbow_retrieval_transport_cap = .3
    return cycle


def assert_real_three_group_messages(cycle, samples, rate, initial):
    expected = common_arm_timing(initial, samples, rate)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert cycle.motion_count == len(messages) == 3
    assert {name for message in messages for name in message.joint_names} == TASK_JOINTS
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 160000000)}
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .02 - 1e-9
        for point, source in zip(message.points, expected['points']):
            assert list(point.positions) == [source[name] for name in message.joint_names]
    command = next(payload for kind, payload in cycle.events if kind == 'command')
    assert command['actual_rate_rad_s'] == rate and command['samples'] == samples
    assert command['holders'] == cycle.holders.snapshot()


def test_real_constructor_keeps_parent_initialization_and_both_caps(monkeypatch):
    observed = []
    def initialize(self, *args, **kwargs):
        observed.append((args, kwargs))
    monkeypatch.setattr(EmptyProtocolCycle, '__init__', initialize)
    cycle = EndpointOverlapCycle('plan', 'buttons')
    assert observed == [(('plan', 'buttons'), {})]
    assert cycle.elbow_empty_approach_cap == .5 and cycle.elbow_retrieval_transport_cap == .3


@pytest.mark.parametrize('phase', ['outboard', 'transfer'])
@pytest.mark.parametrize('actual_failed_path', [False, True])
def test_real_three_publisher_dispatch_preserves_geometry_and_original_common_clock(phase, actual_failed_path):
    cycle = candidate()
    samples = waypoints(cycle)
    if actual_failed_path:
        request = json.loads((PARENT / 'run/9_retrieval_transport_request.json').read_text())
        result = json.loads((PARENT / 'run/9_retrieval_transport_result.json').read_text())
        index = next(index for index, stage in enumerate(result['stages']) if stage['phase'] == phase)
        cycle.positions.update(request['positions'])
        cycle.positions.update(result['stages'][index - 1]['samples'][-1])
        samples = result['stages'][index]['samples']
    initial = dict(cycle.positions)
    cycle.move(samples, 'paired_retrieval_' + phase, .25)
    assert_real_three_group_messages(cycle, samples, .3, initial)
    assert any(kind == 'elbow_retrieval_transport_effort_qualification' for kind, payload in cycle.events)


@pytest.mark.parametrize('cap', [.6, .4, float('nan'), 0])
def test_invalid_caps_reject_before_any_publish(cap):
    cycle = candidate()
    cycle.elbow_retrieval_transport_cap = cap
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_retrieval_outboard', .25)
    assert cycle.motion_count == 0 and all(not publisher.messages for publisher in cycle.publishers.values())


@pytest.mark.parametrize('state', ['unqualified', 'single', 'duplicate'])
def test_original_two_actual_holders_and_qualification_are_required(state):
    cycle = candidate()
    if state == 'unqualified':
        cycle.current_damping_empty_approach_completed = False
    elif state == 'single':
        cycle.holders.release('right')
    else:
        cycle.holders.rows['right']['payload'] = 'left'
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_retrieval_outboard', .25)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase', ['paired_load_forward_outside_housing_table_transfer', 'paired_load_forward_above_housing_outboard'])
def test_loading_transport_is_unchanged(phase):
    cycle = candidate()
    initial, samples = dict(cycle.positions), waypoints(cycle)
    cycle.move(samples, phase, .25)
    assert_real_three_group_messages(cycle, samples, .6, initial)


def test_source_exact_inverse_matches_sealed_failed_parent():
    source = (ROOT / 'endpoint_overlap_cycle.py').read_text()
    assert restore_elbow_transport('endpoint_overlap_cycle.py', source) == (PARENT / 'endpoint_overlap_cycle.py').read_text()
