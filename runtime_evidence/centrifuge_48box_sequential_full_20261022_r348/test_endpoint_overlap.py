import math
from pathlib import Path

import pytest

from coarse_ramp_cycle import CoarseRampCycle
from dual_motion import common_arm_timing
from empty_protocol_cycle import EmptyProtocolCycle
from endpoint_overlap_cycle import EndpointOverlapCycle
from endpoint_overlap_source import restore_endpoint_overlap_source
from test_complete_rate_profile import actual_publisher_cycle, waypoints


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_empty_protocol_full_20261015_r280'


@pytest.mark.parametrize('loaded', [False, True])
def test_actual_three_ros_publishers_same_messages_still_wait_point_two_native_stability(loaded):
    parent = actual_publisher_cycle(loaded=loaded)
    parent.__class__ = EmptyProtocolCycle
    cycle = actual_publisher_cycle(loaded=loaded)
    cycle.__class__ = EndpointOverlapCycle
    samples = waypoints(cycle)
    phase = 'paired_retrieval_transfer' if loaded else 'machine_paired_empty_return'
    requested = .25 if loaded else .8
    endpoint_observations = []
    original = cycle.paired_endpoint_stamp
    def sample(target, not_before):
        stamp = original(target, not_before)
        if stamp is not None:
            endpoint_observations.append((cycle.sim, stamp, not_before, dict(cycle.positions)))
        return stamp
    cycle.paired_endpoint_stamp = sample
    cycle.move(samples, phase, requested)
    parent.move(samples, phase, requested)
    assert {group: publisher.messages for group,publisher in cycle.publishers.items()} == {
        group: publisher.messages for group,publisher in parent.publishers.items()}
    first, last = endpoint_observations[0], endpoint_observations[-1]
    assert last[1]-first[1] > .2
    assert all(stamp >= finish for sim,stamp,finish,positions in endpoint_observations)
    assert cycle.sim >= last[2] + .15
    assert parent.sim - cycle.sim >= .13 and parent.sim - cycle.sim <= .17
    assert all(abs(last[3][name]-value) < .0003 for name,value in samples[-1].items())


def test_hooks_default_false_reverse_to_exact_parent_and_all_lifecycle_unchanged():
    assert restore_endpoint_overlap_source((ROOT/'execute_dual.py').read_text()) == (PARENT/'execute_dual.py').read_text()
    for name in ('retrieve','capture_pair','safety','latest_native_guard','hold','run_complete','common_timing',
                 'prepare_loading_grippers','paired_grippers','require_gripper_endpoint'):
        assert getattr(EndpointOverlapCycle,name) is getattr(EmptyProtocolCycle,name)


@pytest.mark.parametrize('invalid', ['before_finish','nonfinite','outside_precision'])
def test_native_endpoint_cannot_be_certified_early_stale_or_outside_original_precision(invalid):
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    target = dict(cycle.positions)
    cycle.joint_time = 10.2
    if invalid == 'before_finish':
        cycle.joint_time = 10.09
    elif invalid == 'nonfinite':
        cycle.joint_time = math.nan
    else:
        cycle.positions[next(iter(target))] += .00031
    assert cycle.paired_endpoint_stamp(target, 10.1) is None


def test_single_holder_cannot_enable_shared_endpoint_qualification():
    cycle = actual_publisher_cycle(loaded=True)
    cycle.__class__ = EndpointOverlapCycle
    cycle.holders.release('right')
    with pytest.raises(ValueError):
        cycle.paired_feedback_overlap_enabled()


def test_unqualified_initial_approach_keeps_original_serial_observation():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    cycle.current_damping_empty_approach_completed = False
    assert not cycle.paired_feedback_overlap_enabled()
    assert cycle.paired_endpoint_stamp(dict(cycle.positions), 10.1) == cycle.sim


def test_approach_restores_physically_qualified_point_three_five_not_unreached_point_four():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    assert cycle.motion_limits('paired_retrieval_approach', .8) == (.35,.002)


def test_bad_native_clock_cannot_return_success_from_actual_dispatch():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    spin = cycle.spin
    def stale():
        spin()
        cycle.joint_time = math.nan
    cycle.spin = stale
    with pytest.raises(RuntimeError, match='original precision'):
        cycle.move(waypoints(cycle), 'machine_paired_empty_return', .8)


def test_transient_endpoint_deviation_restarts_entire_original_native_stability_window():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    samples = waypoints(cycle)
    spin = cycle.spin
    deviations = []
    observations = []
    original = cycle.paired_endpoint_stamp
    def observed(target, not_before):
        stamp = original(target, not_before)
        observations.append(stamp)
        return stamp
    def transient():
        spin()
        command = next(value for kind,value in cycle.events if kind == 'command')
        finish = command['start'] + command['times'][-1]
        if finish + .10 <= cycle.joint_time < finish + .13:
            cycle.positions['left_elbow_joint'] = samples[-1]['left_elbow_joint'] + .0004
            deviations.append(cycle.joint_time)
    cycle.spin = transient
    cycle.paired_endpoint_stamp = observed
    cycle.move(samples, 'machine_paired_empty_return', .8)
    assert deviations
    first_after_reset = next(stamp for stamp in observations if stamp is not None and stamp > deviations[-1])
    assert cycle.joint_time - first_after_reset > .2


def test_clock_stall_after_partial_valid_endpoint_never_satisfies_native_point_two_certificate():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = EndpointOverlapCycle
    spin = cycle.spin
    def stalled():
        spin()
        command = next(value for kind,value in cycle.events if kind == 'command')
        finish = command['start'] + command['times'][-1]
        cycle.joint_time = min(cycle.joint_time, finish + .1)
    cycle.spin = stalled
    with pytest.raises(RuntimeError, match='original precision'):
        cycle.move(waypoints(cycle), 'machine_paired_empty_return', .8)
