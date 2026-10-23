import json
from pathlib import Path

import numpy as np
import pytest

from dual_motion import common_arm_timing
from empty_protocol_cycle import EmptyProtocolCycle
from endpoint_overlap_cycle import EndpointOverlapCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r287'


def candidate(loaded=False):
    cycle = actual_publisher_cycle(loaded=loaded)
    cycle.__class__ = EndpointOverlapCycle
    cycle.elbow_empty_approach_cap = .5
    cycle.current_damping_empty_approach_completed = False
    return cycle


def test_real_constructor_keeps_parent_initialization_and_sets_fixed_cap(monkeypatch):
    observed = []
    def initialize(self, *args, **kwargs):
        observed.append((args, kwargs))
    monkeypatch.setattr(EmptyProtocolCycle, '__init__', initialize)
    cycle = EndpointOverlapCycle('plan', 'buttons')
    assert observed == [(('plan', 'buttons'), {})] and cycle.elbow_empty_approach_cap == .5


@pytest.mark.parametrize('actual_failed_plan', [False, True])
def test_three_actual_publisher_path_retimes_without_changing_geometry(actual_failed_plan):
    cycle = candidate()
    samples = waypoints(cycle)
    if actual_failed_plan:
        request = json.loads((PARENT / 'run/1_load_pickup_request.json').read_text())
        cycle.positions.update(request['positions'])
        samples = json.loads((PARENT / 'run/1_load_pickup_result.json').read_text())['stages'][0]['samples']
    expected = common_arm_timing(dict(cycle.positions), samples, .5)
    cycle.move(samples, 'paired_load_approach', .8)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert len(messages) == 3 and cycle.motion_count == 3
    assert len({(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages}) == 1
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert np.allclose(times, expected['times'], atol=1e-9, rtol=0)
        assert min(np.diff(times)) >= .020 - 1e-9
        for point, sample in zip(message.points, expected['points']):
            assert list(point.positions) == [sample[name] for name in message.joint_names]
    command = next(payload for kind, payload in cycle.events if kind == 'command')
    assert command['samples'] == samples and command['actual_rate_rad_s'] == .5 and command['holders'] == {}


@pytest.mark.parametrize('cap', [.7, .8, float('nan'), 0])
def test_invalid_production_cap_is_rejected_before_publish(cap):
    cycle = candidate()
    cycle.elbow_empty_approach_cap = cap
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_load_approach', .8)
    assert cycle.motion_count == 0


def test_held_payload_cannot_use_empty_approach_cap():
    cycle = candidate(loaded=True)
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_load_approach', .8)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase,rate', [('paired_retrieval_approach', .8), ('paired_load_descend', .8),
                                      ('paired_retrieval_setdown', .30), ('paired_retrieval_setdown_fine', .09)])
def test_all_other_motion_limits_unchanged(phase, rate):
    cycle = candidate('setdown' in phase)
    original = candidate('setdown' in phase)
    del original.elbow_empty_approach_cap
    cycle.current_damping_empty_approach_completed = True
    original.current_damping_empty_approach_completed = True
    assert cycle.motion_limits(phase, rate) == original.motion_limits(phase, rate)
