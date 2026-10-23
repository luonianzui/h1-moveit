import copy
import json
from pathlib import Path

import pytest

from sequential_support import actual_preload_support
from sequential_force import RETRIEVAL_FORCE_N


def fixture():
    from test_sequential_support import fixture as loading_fixture
    cycle = loading_fixture()
    source = Path(__file__).parent.parent / 'centrifuge_48box_sequential_review_20261019_r5/r301_basket_transition_fault.json'
    record = json.loads(source.read_text())['records'][0]
    payload = record['fault']['payload']
    cycle.phase = 'left_retrieval_grasp'
    cycle.sim = record['fault']['sim']
    cycle.joint_time = record['last_joint']['payload']['source_sim']
    cycle.positions = record['last_joint']['payload']['positions']
    cycle.poses = payload['poses']
    cycle.contacts['left'] = payload['contacts']
    cycle.original_physical_snapshot_effort_N = 17.5
    cycle.synthetic_candidate_acceptance_for_unit_test = True
    cycle.accepted_gripper_efforts = dict(left=RETRIEVAL_FORCE_N)
    return cycle


def test_recorded_contact_geometry_with_synthetic_new_force_contract():
    cycle = fixture()
    assert cycle.synthetic_candidate_acceptance_for_unit_test
    assert cycle.original_physical_snapshot_effort_N != RETRIEVAL_FORCE_N
    before = copy.deepcopy(vars(cycle))
    result = actual_preload_support(cycle, 'left', retrieval=True)
    assert result['valid'] and result['source'] == 'gazebo_physics_contact'
    assert all(3 <= value <= 20.5 for value in result['normal_loads_N'].values())
    assert result['maximum_native_error_m'] < .0002
    assert vars(cycle) == before
    assert not actual_preload_support(cycle, 'left')['valid']


@pytest.mark.parametrize('support,expected', [('centrifuge::bucket_left::floor', True),
    ('centrifuge::bucket_right::floor', False), ('table::surface', False)])
def test_retrieval_accepts_only_its_actual_assigned_bucket(support, expected):
    cycle = fixture()
    cycle.contacts['left'].append(dict(first='box48_left::body::collision', second=support,
        force=0., depth=0., positions=[], normals=[], force_vectors=[]))
    assert actual_preload_support(cycle, 'left', retrieval=True)['valid'] is expected


@pytest.mark.parametrize('change', ['not_accepted', 'wrong_phase', 'wrong_effort', 'stale', 'one_finger',
                                  'foreign_hand', 'weak', 'excess', 'another_holder'])
def test_retrieval_preload_is_actual_physical_evidence_not_an_order(change):
    from types import SimpleNamespace
    cycle = fixture()
    if change == 'not_accepted':
        cycle.preload_grasp_claim['accepted'] = False
    elif change == 'wrong_phase':
        cycle.phase = 'seq_retrieval_single_pilot'
    elif change == 'wrong_effort':
        cycle.accepted_gripper_efforts['left'] = 18.
    elif change == 'stale':
        cycle.joint_time = cycle.sim - .013
    elif change == 'one_finger':
        cycle.contacts['left'] = cycle.contacts['left'][:1]
    elif change == 'foreign_hand':
        cycle.contacts['left'][0]['second'] = 'h1_2::right_dex1_finger_1_link'
    elif change in ('weak', 'excess'):
        for pair in cycle.contacts['left']:
            pair['force_vectors'] = [[value * (.01 if change == 'weak' else 3.) for value in force]
                                     for force in pair['force_vectors']]
    elif change == 'another_holder':
        cycle.holders = SimpleNamespace(rows={'right': {'payload': 'right'}})
    assert not actual_preload_support(cycle, 'left', retrieval=True)['valid']


@pytest.mark.parametrize('change', ['recorded', 'no_accept', 'other_box', 'outside', 'wall_uncertified'])
def test_current_box_handover_keeps_original_containment_and_other_box_monitor(monkeypatch, change):
    from test_sequential_runtime import guard_fixture
    from support_continuity import SupportContinuity
    import sequential_cycle
    cycle = guard_fixture(monkeypatch)
    actual = fixture()
    for name in ('phase', 'grasp_candidate', 'current_grasp_hand', 'sim', 'joint_time', 'positions',
                 'preload_grasp_claim', 'accepted_gripper_targets', 'accepted_gripper_efforts'):
        setattr(cycle, name, getattr(actual, name))
    cycle.pose_time = cycle.sim
    cycle.poses.update(actual.poses)
    cycle.contacts['left'] = actual.contacts['left']
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {payload: SupportContinuity(.02) for payload in ('left', 'right')}
    cycle.support_continuity['left'].last_positive_sim = cycle.sim - .0005
    original_metrics = sequential_cycle.retention_metrics
    def metrics(poses, pairs, payload):
        if payload == 'right':
            return dict(supported=True, contacts_valid=change != 'other_box', sidewall_certified=True, vertical_inside=True)
        result = original_metrics(poses, pairs, payload)
        if change == 'outside':
            result['vertical_inside'] = False
        if change == 'wall_uncertified':
            result['sidewall_certified'] = False
        return result
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', metrics)
    if change == 'no_accept':
        cycle.preload_grasp_claim['accepted'] = False
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    if change == 'recorded':
        cycle.safety()
        assert not cycle.holders.rows
        left = records[-1][1]['rows']['left']
        assert left['continuity']['support_source'] == 'actual_opposed_retrieval_preload'
        assert not left['metrics']['supported'] and left['metrics']['actual_retrieval_preload_support']['valid']
    else:
        with pytest.raises(RuntimeError):
            cycle.safety()
