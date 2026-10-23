import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from preload_observation import pending_unilateral_frame, wait_for_observation
from sequential_support import actual_preload_support

ROOT = Path(__file__).resolve().parent

def actual_fixture():
    source = ROOT / 'r355_preload_fixture.json'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == 'f6269d3f4a833d2c4eda6b96ea2d68a6b1d22645fe58b1f21a24bb782fef2a0c'
    data = json.loads(source.read_text())
    fault, last = data['fault'], data['last']
    history = FloorContactHistory()
    for event in data['contacts']:
        history.observe(event['payload'])
    goal = last['gripper_goal_accepted']['payload']
    joint = last['actual_full_joint_frame']['payload']
    cycle = SimpleNamespace(remaining_guard_enabled=True, current_grasp_hand='left',
        grasp_candidate='left', phase='left_retrieval_grasp', holders=SimpleNamespace(rows={}),
        order=SimpleNamespace(locations={'left': 'basket'}), retrieval_closing=True,
        pair_goal_handles={'left': SimpleNamespace(accepted=True)},
        preload_grasp_claim=last['sequential_preload_claim']['payload'],
        accepted_gripper_targets={'left': goal['target']}, accepted_gripper_efforts={'left': goal['feedforward_N']},
        sim=fault['sim'], joint_time=joint['source_sim'], positions=joint['positions'],
        poses=fault['payload']['poses'], contacts={'left': fault['payload']['contacts']},
        closing_floor_history=history)
    return cycle, fault['payload']['metrics'], data

def test_actual_failure_only_wait_eligible_not_support_or_capture():
    cycle, metrics, _ = actual_fixture()
    before = copy.deepcopy(vars(cycle.closing_floor_history))
    assert not single_missing_floor_frame(cycle, 'left', metrics)['valid']
    assert not actual_preload_support(cycle, 'left', retrieval=True)['valid']
    result = pending_unilateral_frame(cycle)
    assert result is not None and result['pending_kind'] == 'unilateral_low_force'
    assert result['not_a_support_or_grip_certificate']
    assert result['observed_fingers'] == [1]
    assert not cycle.holders.rows and vars(cycle.closing_floor_history) == before

@pytest.mark.parametrize('change', ['older','unknown','future','force','zero','foreign',
    'no_contact','holder','not_closing','not_accepted','native_stale','above_floor','both_fingers'])
def test_unilateral_wait_fail_closed(change):
    cycle, _, _ = actual_fixture()
    if change == 'older': cycle.closing_floor_history.rows['left']['positive'] -= .0005
    elif change == 'unknown': cycle.closing_floor_history.rows['left']['positive'] = None
    elif change == 'future': cycle.closing_floor_history.rows['left']['positive'] = cycle.sim + .001
    elif change == 'force': cycle.contacts['left'][0]['force'] = 1.001
    elif change == 'zero': cycle.contacts['left'][0]['force'] = 0.
    elif change == 'foreign': cycle.contacts['left'][0]['second'] = 'h1_2::right_dex1_finger_1_link'
    elif change == 'no_contact': cycle.contacts['left'] = []
    elif change == 'holder': cycle.holders.rows['right'] = {'payload': 'right'}
    elif change == 'not_closing': cycle.retrieval_closing = False
    elif change == 'not_accepted': cycle.preload_grasp_claim['accepted'] = False
    elif change == 'native_stale': cycle.joint_time = cycle.sim - .013
    elif change == 'above_floor': cycle.poses['box48_left']['position']['z'] += .0001
    elif change == 'both_fingers':
        pair = copy.deepcopy(cycle.contacts['left'][0])
        pair['second'] = pair['second'].replace('finger_1', 'finger_2')
        cycle.contacts['left'].append(pair)
    assert pending_unilateral_frame(cycle) is None

def test_wait_exhaustion_does_not_change_original_support_decision():
    cycle, metrics, _ = actual_fixture()
    initial = pending_unilateral_frame(cycle)
    count = []
    outcome = wait_for_observation(cycle, initial, lambda: None, lambda: count.append(True),
        clock=lambda: 0., pending=pending_unilateral_frame)
    assert outcome['callbacks'] == 64 and outcome['still_missing']
    assert not single_missing_floor_frame(cycle, 'left', metrics)['valid']
    assert not cycle.holders.rows and len(count) == 65

def test_synthetic_next_supported_callback_only_exercises_wait_protocol():
    cycle, _, data = actual_fixture()
    initial = pending_unilateral_frame(cycle)
    def pump():
        contact = copy.deepcopy(data['contacts'][-2]['payload'])
        contact['stamp'] = cycle.sim + .0005
        cycle.sim = contact['stamp']
        cycle.contacts['left'] = contact['pairs']
        cycle.closing_floor_history.observe(contact)
    outcome = wait_for_observation(cycle, initial, pump, lambda: None,
        clock=lambda: 0., pending=pending_unilateral_frame)
    assert outcome['callbacks'] == 1 and not outcome['still_missing']
    assert not cycle.holders.rows

def test_simulation_budget_still_rejects_skipped_callbacks():
    cycle, _, _ = actual_fixture()
    def pump():
        cycle.sim += .0015
    with pytest.raises(RuntimeError, match='bounded simulation'):
        wait_for_observation(cycle, pending_unilateral_frame(cycle), pump, lambda: None,
            clock=lambda: 0., pending=pending_unilateral_frame)

def test_original_protection_and_actions_unmodified_from_r355():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r355'
    for name in ('sequential_cycle.py', 'preload_floor_frame.py', 'sequential_support.py',
                 'sequential_speed_cycle.py', 'sequential_force.py', 'speed_policy.py',
                 'speed_retrieval_geometry.py', 'table_alignment_cycle.py'):
        assert (ROOT / name).read_bytes() == (parent / name).read_bytes(), name


def test_only_exact_declared_observation_functions_restore_parent():
    from unilateral_source import restore_unilateral_observation
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r355'
    current = (ROOT / 'preload_observation.py').read_text()
    assert restore_unilateral_observation(current) == (parent / 'preload_observation.py').read_text()
    with pytest.raises(ValueError, match='exact unilateral'):
        restore_unilateral_observation(current.replace("force'] <= 1.", "force'] <= 10."))
    with pytest.raises(ValueError, match='exact unilateral'):
        restore_unilateral_observation(current.replace('super().safety()', 'None'))
