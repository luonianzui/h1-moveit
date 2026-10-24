import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from sequential_support import actual_preload_support, original_basket_preload_gap_valid


def fixture():
    path = Path(__file__).parent.parent / 'centrifuge_48box_sequential_review_20261019_r7/r303_basket_transition_fault.json'
    record = json.loads(path.read_text())['records'][0]
    fault = record['fault']
    payload = fault['payload']['payload']
    claim = record['last_claim']['payload']
    goal = record['last_goal']['payload']
    assert claim['accepted'] and goal['feedforward_N'] == 16.5 and goal['target'] == -.01901
    cycle = SimpleNamespace(current_grasp_hand=claim['hand'], grasp_candidate=payload, phase='left_retrieval_grasp',
        preload_grasp_claim=claim, accepted_gripper_targets={claim['hand']: goal['target']},
        accepted_gripper_efforts={claim['hand']: goal['feedforward_N']}, sim=fault['sim'],
        joint_time=record['last_joint']['payload']['source_sim'], positions=record['last_joint']['payload']['positions'],
        poses=fault['payload']['poses'], contacts={payload: fault['payload']['contacts']})
    return cycle, copy.deepcopy(fault['payload']['continuity'])


def test_real_r303_weak_preload_uses_original_basket_history_not_strong_grip_proof():
    cycle, continuity = fixture()
    result = actual_preload_support(cycle, 'left', retrieval=True)
    assert not result['valid']
    assert all(0 < value < 3 for value in result['normal_loads_N'].values())
    assert original_basket_preload_gap_valid(result, continuity)


@pytest.mark.parametrize('change', ['expired', 'future', 'unknown', 'different_window', 'missing_finger',
    'zero_force', 'excess_force', 'nonfinite_force', 'stale_native', 'off_target', 'missing_native', 'not_physical'])
def test_original_window_never_bypasses_missing_or_unsafe_evidence(change):
    cycle, continuity = fixture()
    certificate = actual_preload_support(cycle, 'left', retrieval=True)
    if change == 'expired':
        continuity['zero_force_gap_sim_s'] = .020001
    elif change == 'future':
        continuity['zero_force_gap_sim_s'] = -.0005
    elif change == 'unknown':
        continuity['zero_force_gap_sim_s'] = float('inf')
    elif change == 'different_window':
        continuity['limit_sim_s'] = .03
    elif change == 'missing_finger':
        del certificate['fingers']['2']
    elif change == 'zero_force':
        certificate['normal_loads_N']['1'] = 0.
    elif change == 'excess_force':
        certificate['normal_loads_N']['1'] = 20.50001
    elif change == 'nonfinite_force':
        certificate['normal_loads_N']['1'] = float('nan')
    elif change == 'stale_native':
        certificate['native_age_sim_s'] = .013
    elif change == 'off_target':
        certificate['maximum_native_error_m'] = .000251
    elif change == 'missing_native':
        del certificate['native_age_sim_s']
    elif change == 'not_physical':
        certificate['source'] = 'commanded'
    assert not original_basket_preload_gap_valid(certificate, continuity)


@pytest.mark.parametrize('change', ['recorded', 'expired', 'no_accept', 'other_box', 'outside', 'uncertified_wall'])
def test_integrated_gap_still_requires_original_containment_and_other_box_protection(monkeypatch, change):
    from test_sequential_runtime import guard_fixture
    from support_continuity import SupportContinuity
    import sequential_cycle
    cycle = guard_fixture(monkeypatch)
    actual, continuity = fixture()
    for name, value in vars(actual).items():
        setattr(cycle, name, value)
    cycle.pose_time = cycle.sim
    cycle.contacts['right'] = []
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {payload: SupportContinuity(.02) for payload in ('left', 'right')}
    cycle.support_continuity['left'].last_positive_sim = cycle.sim - (.020001 if change == 'expired' else .0005)
    original_metrics = sequential_cycle.retention_metrics
    def metrics(poses, pairs, payload):
        if payload == 'right':
            return dict(supported=True, contacts_valid=change != 'other_box', sidewall_certified=True, vertical_inside=True)
        result = original_metrics(poses, pairs, payload)
        if change == 'outside':
            result['vertical_inside'] = False
        if change == 'uncertified_wall':
            result['sidewall_certified'] = False
        return result
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', metrics)
    if change == 'no_accept':
        cycle.preload_grasp_claim['accepted'] = False
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    if change == 'recorded':
        cycle.safety()
        row = records[-1][1]['rows']['left']
        assert row['continuity']['not_a_strong_grip_certificate']
        assert row['continuity']['support_source'] == 'original_basket_positive_history_with_actual_opposed_contacts'
        assert not row['metrics']['actual_retrieval_preload_support']['valid']
        assert not cycle.holders.rows
    else:
        with pytest.raises(RuntimeError):
            cycle.safety()
