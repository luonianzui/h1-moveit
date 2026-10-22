import hashlib
import json
from pathlib import Path

import pytest

from complete_rate_profile import EMPTY_CAPS
from ramped_button_cycle import RampedButtonCycle
from inherited_source_identity import restore_inherited_source
from test_complete_rate_profile import actual_publisher_cycle, assert_real_three_group_messages, waypoints


EVIDENCE = Path(__file__).resolve().parent
BASELINE = EVIDENCE.parent / 'centrifuge_48box_leg_fresh_endpoint_20261014_r261'


def test_qualified_point_four_approach_cap_is_unchanged_against_verified_parent_clone():
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    source = (EVIDENCE / 'complete_rate_profile.py').read_text()
    assert source.count("'paired_retrieval_approach': .4,") == 1
    assert hashlib.sha256(source.encode()).hexdigest() == records['complete_rate_profile.py']['derived_sha256']
    assert EMPTY_CAPS['machine_paired_empty_return'] == .6
    assert EMPTY_CAPS['paired_retrieval_descend'] == .26


def test_actual_inherited_dispatch_uses_point_identical_linear_point_four_not_failed_ramp():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = RampedButtonCycle
    initial, samples = dict(cycle.positions), waypoints(cycle)
    assert cycle.motion_limits('paired_retrieval_approach', .8) == (.4, .002)
    cycle.move(samples, 'paired_retrieval_approach', .8)
    assert_real_three_group_messages(cycle, samples, .4, initial)
    assert all(not point.velocities and not point.accelerations
               for publisher in cycle.publishers.values() for point in publisher.messages[-1].points)
    assert not any(kind == 'empty_retrieval_global_arc_ramp' for kind, value in cycle.events)
    source = restore_inherited_source('execute_cycle.py', (EVIDENCE / 'execute_cycle.py').read_text())
    assert source.count('from endpoint_overlap_cycle import EndpointOverlapCycle as DualCycle') == 1
    assert 'PairedRampCycle' not in source
    assert not (EVIDENCE / 'paired_ramp_cycle.py').exists()


def test_prior_point_four_baseline_is_real_complete_pass_not_current_certificate():
    assert hashlib.sha256((BASELINE / 'REPORT.md').read_bytes()).hexdigest() == 'ab46753b11fe9d3d9313a9ff9aabdee20a8e17acee16dbb4879ffaaf519c86d3'
    identity = json.loads((BASELINE / 'identity.json').read_text())
    assert identity['empty_retrieval_approach_rate_cap_rad_s'] == .4
    assert json.loads((BASELINE / 'run/result.json').read_text())['status'] == 'PASS'
    assert json.loads((BASELINE / 'cycle_audit.json').read_text())['status'] == 'PASS'


def test_exact_failed_192_point_path_retimes_actual_three_publishers_without_geometry_change():
    parent = EVIDENCE.parent / 'centrifuge_48box_button_ramp_20261014_r263'
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (parent / 'SHA256SUMS').read_text().splitlines()))
    relative = 'run/retrieval_velocity_failure_window.json'
    source = parent / relative
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[relative]
    events = json.loads(source.read_text())['selected']
    command = next(row for row in events if row['kind'] == 'command')['payload']
    assert command['actual_rate_rad_s'] == .6 and command['holders'] == {}
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = RampedButtonCycle
    cycle.positions.update(command['initial'])
    initial = dict(cycle.positions)
    cycle.move(command['samples'], 'paired_retrieval_approach', .8)
    assert_real_three_group_messages(cycle, command['samples'], .4, initial)
    assert all(len(publisher.messages[-1].points) == 192 for publisher in cycle.publishers.values())


@pytest.mark.parametrize('invalid', ['loaded', 'unqualified'])
def test_original_empty_qualification_remains_required(invalid):
    cycle = actual_publisher_cycle(loaded=invalid == 'loaded')
    cycle.__class__ = RampedButtonCycle
    cycle.current_damping_empty_approach_completed = invalid != 'unqualified'
    with pytest.raises(ValueError):
        cycle.move(waypoints(cycle), 'paired_retrieval_approach', .8)
    assert cycle.motion_count == 0


def test_point_four_keeps_original_two_milliradian_tracking_fault():
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = RampedButtonCycle
    original_spin = cycle.spin

    def faulty_spin():
        original_spin()
        if cycle.sim >= 10.4:
            cycle.positions['right_elbow_joint'] += .003

    cycle.spin = faulty_spin
    with pytest.raises(RuntimeError, match='stricter .002rad'):
        cycle.move(waypoints(cycle), 'paired_retrieval_approach', .8)
