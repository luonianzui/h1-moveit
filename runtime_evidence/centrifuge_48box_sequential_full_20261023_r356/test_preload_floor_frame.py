import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from sequential_support import actual_preload_support


def actual_fixture():
    path = Path(__file__).resolve().parents[1] / 'centrifuge_48box_sequential_review_20261019_r13/r310_basket_transition_fault.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == '10f2bcb8953f9f356c57a6caa9923dc18b9bb18b6a35f4ebe3fb5cb3f40268ee'
    record = json.loads(path.read_text())['records'][0]
    fault, goal = record['fault'], record['last_goal']['payload']
    history = FloorContactHistory()
    for event in record['recent']:
        if event['kind'] == 'contact':
            history.observe(event['payload'])
    cycle = SimpleNamespace(current_grasp_hand='left', grasp_candidate='left', phase='left_retrieval_grasp',
        holders=SimpleNamespace(rows={}, snapshot=lambda: {}), retrieval_closing=True,
        pair_goal_handles={'left':SimpleNamespace(accepted=True)}, preload_grasp_claim=record['last_claim']['payload'],
        accepted_gripper_targets={'left':goal['target']}, accepted_gripper_efforts={'left':goal['feedforward_N']},
        sim=fault['sim'], joint_time=record['last_joint']['payload']['source_sim'],
        positions=record['last_joint']['payload']['positions'], poses=fault['payload']['poses'],
        contacts={'left':fault['payload']['contacts']}, closing_floor_history=history)
    return cycle, copy.deepcopy(fault['payload']['metrics'])


def test_actual_single_missing_contact_frame_uses_floor_history_not_grip_certificate():
    cycle, metrics = actual_fixture()
    original = copy.deepcopy(vars(cycle.closing_floor_history))
    assert not actual_preload_support(cycle, 'left', retrieval=True)['valid']
    result = single_missing_floor_frame(cycle, 'left', metrics)
    assert result['valid'] and result['not_a_grip_certificate']
    assert result['observed_fingers'] == [1]
    assert result['missing_floor_sim_s'] == pytest.approx(.0005)
    assert result['maximum_native_distance_to_final_goal_m'] > .00025
    assert not cycle.holders.rows and vars(cycle.closing_floor_history) == original


@pytest.mark.parametrize('change', ['two_frames','unknown','future_history','stale_contact','stale_native',
    'not_accepted','not_closing','wrong_phase','other_payload','holder','after_profile','before_start',
    'above_floor','side_overhang','outside','no_finger','foreign_hand','zero_force','excess_force',
    'depth','nonfinite','missing_handle','not_midband'])
def test_single_frame_transition_is_narrow_and_fail_closed(change):
    cycle, metrics = actual_fixture()
    row = cycle.closing_floor_history.rows['left']
    if change == 'two_frames': row['positive'] -= .0005
    elif change == 'unknown': row['positive'] = None
    elif change == 'future_history': row['positive'] = row['stamp'] + .0005
    elif change == 'stale_contact': cycle.sim = row['stamp'] + .0011
    elif change == 'stale_native': cycle.joint_time = cycle.sim - .013
    elif change == 'not_accepted': cycle.preload_grasp_claim['accepted'] = False
    elif change == 'not_closing': cycle.retrieval_closing = False
    elif change == 'wrong_phase': cycle.phase = 'seq_retrieval_single_pilot'
    elif change == 'other_payload': cycle.grasp_candidate = 'right'
    elif change == 'holder': cycle.holders.rows = {'left': {'payload':'left'}}
    elif change == 'after_profile': cycle.preload_grasp_claim['accepted_common_start_sim'] = cycle.sim - 2.051
    elif change == 'before_start': cycle.preload_grasp_claim['accepted_common_start_sim'] = cycle.sim + .001
    elif change == 'above_floor': metrics['corners_min'][2] = -.029994
    elif change == 'side_overhang': metrics['side_overhang_m'] = .0000051
    elif change == 'outside': metrics['vertical_inside'] = False
    elif change == 'no_finger': cycle.contacts['left'] = []
    elif change == 'foreign_hand': cycle.contacts['left'][0]['second'] = 'h1_2::right_dex1_finger_1_link'
    elif change == 'zero_force': cycle.contacts['left'][0]['force'] = 0.
    elif change == 'excess_force': cycle.contacts['left'][0]['force'] = 40.01
    elif change == 'depth': cycle.contacts['left'][0]['depth'] = .0011
    elif change == 'nonfinite': cycle.sim = float('nan')
    elif change == 'missing_handle': cycle.pair_goal_handles.clear()
    else: cycle.contacts['left'][0]['positions'][0][0] += .05
    assert not single_missing_floor_frame(cycle, 'left', metrics)['valid']


def test_finger_contact_does_not_refresh_basket_history_and_reversal_invalidates():
    cycle, metrics = actual_fixture()
    history = cycle.closing_floor_history
    positive = history.rows['left']['positive']
    stamp = history.rows['left']['stamp']
    history.observe(dict(side='left', stamp=stamp+.0005, pairs=cycle.contacts['left']))
    assert history.rows['left']['positive'] == positive
    history.observe(dict(side='left', stamp=stamp, pairs=cycle.contacts['left']))
    assert history.rows['left']['positive'] is None


@pytest.mark.parametrize('change', ['recorded', 'other_box', 'second_missing_frame'])
def test_integrated_transition_keeps_other_box_protection(monkeypatch, change):
    from support_continuity import SupportContinuity
    from test_sequential_runtime import guard_fixture
    import sequential_cycle
    cycle = guard_fixture(monkeypatch)
    actual, expected_metrics = actual_fixture()
    for name, value in vars(actual).items():
        setattr(cycle, name, value)
    cycle.pose_time = cycle.sim
    cycle.contacts['right'] = []
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {payload:SupportContinuity(.02) for payload in ('left', 'right')}
    cycle.support_continuity['left'].last_positive_sim = cycle.sim - .0005
    original = sequential_cycle.retention_metrics
    def measured(poses, pairs, payload):
        if payload == 'right':
            return dict(supported=True, contacts_valid=change != 'other_box', sidewall_certified=True, vertical_inside=True)
        return original(poses, pairs, payload)
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', measured)
    if change == 'second_missing_frame':
        cycle.closing_floor_history.rows['left']['positive'] -= .0005
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    if change == 'recorded':
        cycle.safety()
        row = records[-1][1]['rows']['left']
        assert row['continuity']['support_source'] == 'one_actual_contact_frame_floor_history'
        assert row['continuity']['not_a_strong_grip_certificate'] and not cycle.holders.rows
    else:
        with pytest.raises(RuntimeError):
            cycle.safety()
