import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from execute_dual import DualCycle
from sequential_cycle import SequentialCycle
from sequential_support import actual_preload_support


def fixture():
    path = Path(__file__).parent.parent / 'centrifuge_48box_sequential_review_20261019_r2/table_transition_fault.json'
    record = json.loads(path.read_text())['records'][0]
    payload = record['fault']['payload']
    return SimpleNamespace(current_grasp_hand='left', grasp_candidate='left', phase='left_seq_load_grasp',
        pair_goal_handles={'left': SimpleNamespace(accepted=True)},
        preload_grasp_claim=dict(accepted=True, hand='left', payload='left'),
        accepted_gripper_targets=dict(left=-.01901), accepted_gripper_efforts=dict(left=16.),
        sim=record['fault']['sim'], joint_time=record['last_joint']['payload']['source_sim'],
        positions=record['last_joint']['payload']['positions'],
        poses=payload['poses'], contacts=dict(left=payload['contacts']))


def test_recorded_actual_r298_preload_is_qualified_without_new_anchor():
    cycle = fixture()
    before = copy.deepcopy(vars(cycle))
    result = actual_preload_support(cycle, 'left')
    assert result['valid'] and result['source'] == 'gazebo_physics_contact'
    assert all(3 <= value <= 20.5 for value in result['normal_loads_N'].values())
    assert result['maximum_native_error_m'] < .00002
    assert result['no_holder_or_runtime_anchor_created_or_rebased']
    assert vars(cycle) == before


@pytest.mark.parametrize('change', ['not_accepted', 'wrong_phase', 'wrong_target', 'stale', 'not_closed',
                                  'one_finger', 'weak', 'excess_force', 'foreign_hand', 'bad_clock', 'bad_stamp'])
def test_preload_rejects_missing_or_invalid_physical_evidence(change):
    cycle = fixture()
    if change == 'not_accepted':
        cycle.preload_grasp_claim['accepted'] = False
    elif change == 'wrong_phase':
        cycle.phase = 'seq_load_empty_approach'
    elif change == 'wrong_target':
        cycle.grasp_candidate = 'right'
    elif change == 'stale':
        cycle.joint_time = cycle.sim - .013
    elif change == 'bad_clock':
        cycle.sim = float('nan')
    elif change == 'bad_stamp':
        cycle.joint_time = float('nan')
    elif change == 'not_closed':
        cycle.positions['left_dex1_finger_1_joint'] = -.0194
    elif change == 'one_finger':
        cycle.contacts['left'] = cycle.contacts['left'][:1]
    elif change in ('weak', 'excess_force'):
        factor = 0.1 if change == 'weak' else 2.
        for pair in cycle.contacts['left']:
            pair['force_vectors'] = [[value * factor for value in vector] for vector in pair['force_vectors']]
    elif change == 'foreign_hand':
        cycle.contacts['left'][0]['second'] = 'h1_2::right_dex1_finger_1_pad_link'
    assert not actual_preload_support(cycle, 'left')['valid']


def test_formal_capture_and_original_waiting_are_not_replaced():
    assert SequentialCycle.capture_pair is DualCycle.capture_pair


def test_preload_cannot_ignore_another_actual_holder():
    cycle = fixture()
    cycle.holders = SimpleNamespace(rows={'right': {'payload': 'right'}})
    assert not actual_preload_support(cycle, 'left')['valid']


@pytest.mark.parametrize('change', ['qualified', 'target_drift', 'other_box_unsupported'])
def test_support_integration_keeps_birth_pose_and_other_box_protection(monkeypatch, change):
    from test_sequential_runtime import guard_fixture
    cycle = guard_fixture(monkeypatch)
    actual = fixture()
    cycle.phase, cycle.grasp_candidate, cycle.current_grasp_hand = actual.phase, 'left', 'left'
    cycle.sim = cycle.pose_time = actual.sim
    cycle.joint_time, cycle.positions = actual.joint_time, actual.positions
    cycle.poses.update(actual.poses)
    cycle.contacts['left'] = actual.contacts['left']
    cycle.preload_grasp_claim = actual.preload_grasp_claim
    cycle.accepted_gripper_targets = actual.accepted_gripper_targets
    cycle.accepted_gripper_efforts = actual.accepted_gripper_efforts
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    if change == 'target_drift':
        cycle.poses['box48_left']['position']['x'] += .00031
    elif change == 'other_box_unsupported':
        cycle.contacts['right'] = []
    if change != 'qualified':
        with pytest.raises(RuntimeError):
            cycle.safety()
    else:
        cycle.safety()
        left = records[-1][1]['rows']['left']
        assert not left['metrics']['supported']
        assert left['metrics']['actual_preload_support']['valid']
        assert left['continuity']['support_source'] == 'actual_opposed_preload'
        assert not left['continuity']['original_table_support_valid']
        assert not cycle.holders.rows
