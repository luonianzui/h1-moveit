import copy
import json
from pathlib import Path

import pytest

from sequential_support import actual_preload_support, actual_preload_table_continuity


def fixture():
    from test_sequential_support import fixture as original_fixture
    cycle = original_fixture()
    source = Path(__file__).parent.parent / 'centrifuge_48box_sequential_review_20261019_r4/r300_table_transition_fault.json'
    record = json.loads(source.read_text())['records'][0]
    payload = record['fault']['payload']
    cycle.sim = record['fault']['sim']
    cycle.joint_time = record['last_joint']['payload']['source_sim']
    cycle.positions = record['last_joint']['payload']['positions']
    cycle.poses = payload['poses']
    cycle.contacts['left'] = payload['contacts']
    prior = [row for row in record['recent'] if row['kind'] == 'contact' and row['payload']['side'] == 'left'
             and any('table::' in pair['first'] + pair['second'] and pair['force'] > 0
                     for pair in row['payload']['pairs'])][-1]
    cycle.table_positive = dict(left=prior['sim'])
    return cycle


def test_actual_r300_single_solver_frame_keeps_original_twenty_ms_window():
    cycle = fixture()
    before = copy.deepcopy(vars(cycle))
    assert not actual_preload_support(cycle, 'left')['valid']
    result = actual_preload_table_continuity(cycle, 'left')
    assert result['valid'] and result['table_contact_still_present']
    assert abs(result['continuity']['zero_force_gap_sim_s'] - .0005) < 1e-10
    assert result['continuity']['original_support_window_sim_s'] == .02
    assert vars(cycle) == before


@pytest.mark.parametrize('change', ['no_previous_positive', 'expired', 'future', 'missing_table', 'foreign_hand',
    'wrong_phase', 'no_accepted', 'stale_native', 'not_closed', 'existing_holder', 'bad_depth', 'bad_force'])
def test_gap_is_not_an_exemption_for_unknown_or_unsafe_state(change):
    from types import SimpleNamespace
    cycle = fixture()
    if change == 'no_previous_positive':
        cycle.table_positive = {}
    elif change == 'expired':
        cycle.table_positive['left'] = cycle.sim - .02001
    elif change == 'future':
        cycle.table_positive['left'] = cycle.sim + .001
    elif change == 'missing_table':
        cycle.contacts['left'] = [pair for pair in cycle.contacts['left'] if 'table::' not in pair['first'] + pair['second']]
    elif change == 'foreign_hand':
        cycle.contacts['left'][-1]['second'] = 'h1_2::right_dex1_finger_1_link'
    elif change == 'wrong_phase':
        cycle.phase = 'seq_load_load_transfer'
    elif change == 'no_accepted':
        cycle.preload_grasp_claim['accepted'] = False
    elif change == 'stale_native':
        cycle.joint_time = cycle.sim - .013
    elif change == 'not_closed':
        cycle.positions['left_dex1_finger_1_joint'] = -.0194
    elif change == 'existing_holder':
        cycle.holders = SimpleNamespace(rows={'right': {'payload': 'right'}})
    elif change == 'bad_depth':
        cycle.contacts['left'][0]['depth'] = .00101
    elif change == 'bad_force':
        cycle.contacts['left'][0]['force'] = float('nan')
    assert not actual_preload_table_continuity(cycle, 'left')['valid']


@pytest.mark.parametrize('change', ['recorded_frame', 'expired', 'drift', 'other_box'])
def test_remaining_monitor_preserves_birth_and_other_box_gates(monkeypatch, change):
    from test_sequential_runtime import guard_fixture
    cycle = guard_fixture(monkeypatch)
    actual = fixture()
    for name in ('phase', 'grasp_candidate', 'current_grasp_hand', 'sim', 'joint_time', 'positions',
                 'preload_grasp_claim', 'accepted_gripper_targets', 'accepted_gripper_efforts', 'table_positive'):
        setattr(cycle, name, getattr(actual, name))
    cycle.pose_time = cycle.sim
    cycle.poses.update(actual.poses)
    cycle.contacts['left'] = actual.contacts['left']
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    if change == 'expired':
        cycle.table_positive['left'] = cycle.sim - .02001
    elif change == 'drift':
        cycle.poses['box48_left']['position']['x'] += .00031
    elif change == 'other_box':
        cycle.contacts['right'] = []
    if change == 'recorded_frame':
        cycle.safety()
        assert records[-1][1]['rows']['left']['continuity']['support_source'] == 'original_table_force_continuity'
        assert not cycle.holders.rows
    else:
        with pytest.raises(RuntimeError):
            cycle.safety()
