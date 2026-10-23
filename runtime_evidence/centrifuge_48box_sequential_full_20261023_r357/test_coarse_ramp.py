from endpoint_overlap_source import restore_endpoint_overlap_source
from empty_protocol_source import restore_empty_protocol_source
import hashlib
import json
from pathlib import Path

import pytest

from coarse_ramp_cycle import CoarseRampCycle
from dual_motion import common_arm_timing
from execute_dual import DualCycle
from paired_ramp import ramped_common_timing, PAIR_TIMING_HOOK
from post_retreat_gripper_cycle import PostRetreatGripperCycle
from test_complete_rate_profile import actual_publisher_cycle, waypoints


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_post_retreat_gripper_20261014_r278'


def actual_command():
    manifest = {relative.lstrip('*'): checksum for checksum, relative in
                (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
    path = PARENT / 'run/exact_fault_and_commands.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path.relative_to(PARENT))]
    row = json.loads(path.read_text())['commands'][-1]
    assert row['phase'] == 'paired_retrieval_setdown' and row['payload']['actual_rate_rad_s'] == .3
    return row['payload']


@pytest.mark.parametrize('actual', [False, True])
def test_real_three_ros_publishers_keep_every_original_position_common_clock_and_floor(actual):
    cycle = actual_publisher_cycle(loaded=True)
    cycle.__class__ = CoarseRampCycle
    if actual:
        command = actual_command()
        cycle.positions.update(command['initial'])
        samples = command['samples']
    else:
        samples = waypoints(cycle)
    expected = ramped_common_timing(cycle.positions, samples, .3, .6)
    original = common_arm_timing(cycle.positions, samples, .3)
    cycle.move(samples, 'paired_retrieval_setdown', .3)
    messages = [publisher.messages[-1] for publisher in cycle.publishers.values()]
    assert len(messages) == cycle.motion_count == 3
    assert {(message.header.stamp.sec, message.header.stamp.nanosec) for message in messages} == {(10, 160000000)}
    assert expected['points'] == original['points'] and expected['groups'] == original['groups']
    for message in messages:
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in message.points]
        assert times == pytest.approx(expected['times'], abs=1e-9)
        assert min(last-first for first,last in zip(times,times[1:])) >= .020-1e-9
        for point, reference in zip(message.points, original['points']):
            assert list(point.positions) == [reference[name] for name in message.joint_names]
            assert not point.velocities and not point.accelerations


@pytest.mark.parametrize('invalid', ['no_qualification', 'single_holder', 'rate'])
def test_loaded_ramp_cannot_bypass_original_current_pair_or_rate_admission(invalid):
    cycle = actual_publisher_cycle(loaded=True)
    cycle.__class__ = CoarseRampCycle
    cycle.phase = 'paired_retrieval_setdown'
    rate = .3
    if invalid == 'no_qualification':
        cycle.current_damping_empty_approach_completed = False
    elif invalid == 'single_holder':
        cycle.holders.release('right')
    else:
        rate = .35
    with pytest.raises(ValueError):
        cycle.common_timing(cycle.positions, waypoints(cycle), rate)
    assert cycle.motion_count == 0


@pytest.mark.parametrize('phase', ['paired_retrieval_approach','paired_retrieval_transfer',
                                  'paired_retrieval_setdown_fine','paired_load_approach'])
def test_all_other_phase_timings_remain_byte_equivalent_parent_outputs(phase):
    cycle = actual_publisher_cycle(loaded=True)
    cycle.__class__ = CoarseRampCycle
    cycle.phase = phase
    samples = waypoints(cycle)
    assert cycle.common_timing(cycle.positions, samples, .3) == common_arm_timing(cycle.positions,samples,.3)


def test_only_default_timing_hook_changes_source_not_lifecycle_or_native_guards():
    source = restore_empty_protocol_source(restore_endpoint_overlap_source((ROOT / 'execute_dual.py').read_text()))
    restored = source.replace(PAIR_TIMING_HOOK, '', 1).replace(
        'self.common_timing(self.positions, samples, rate)', 'common_arm_timing(self.positions, samples, rate)')
    assert restored == (PARENT / 'execute_dual.py').read_text()
    for name in ('run_complete','retrieve','safety','latest_native_guard','capture_pair','hold'):
        assert getattr(CoarseRampCycle,name) is getattr(PostRetreatGripperCycle,name)
