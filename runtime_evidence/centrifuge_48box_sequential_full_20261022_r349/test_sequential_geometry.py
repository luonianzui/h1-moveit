import copy
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from dual_motion import TASK_JOINTS
from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner, accurate, dense, validate_request
from sequential_planning import nearest_symmetric_yaw


def request():
    return dict(sequential_contract=CONTRACT, hand='left', payload='left',
                positions=json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions'],
                original_runtime_holder_references={})


def task_state():
    return {name: 0. for name in TASK_JOINTS}


def test_dense_original_resolution_and_input_immutability():
    first = task_state()
    last = dict(first, torso_joint=.507)
    baseline = copy.deepcopy((first, last))
    rows = list(dense(first, last))
    assert rows[0][1] == first and rows[-1][1] == last
    assert all(max(abs(end[name] - start[name]) for name in first) <= .002 + 1e-12
               for (_, start), (_, end) in zip(rows, rows[1:]))
    assert (first, last) == baseline


@pytest.mark.parametrize('step', [.002001, 0., -1., float('nan')])
def test_cannot_coarsen_original_collision_resolution(step):
    with pytest.raises(ValueError):
        list(dense(task_state(), task_state(), step))


@pytest.mark.parametrize('change', [dict(sequential_contract='paired'), dict(hand='wrong'),
                                  dict(payload='wrong'), dict(positions={}),
                                  dict(original_runtime_holder_references=[]),
                                  dict(original_runtime_holder_references={'right': dict(payload='right')}),
                                  dict(original_runtime_holder_references={'left': dict(payload='right')}),
                                  dict(original_runtime_holder_references={'left': dict(payload='left'), 'right': dict(payload='right')})])
def test_explicit_single_holder_schema(change):
    with pytest.raises(ValueError):
        validate_request(dict(request(), **change))


def test_nonfinite_snapshot_rejected():
    current = request()
    current['positions']['torso_joint'] = float('nan')
    with pytest.raises(ValueError):
        validate_request(current)


def test_full_actual_passive_and_finger_snapshot_required():
    validate_request(request())
    for name in ('left_hip_yaw_joint', 'left_dex1_finger_1_joint'):
        current = request()
        current['positions'].pop(name)
        with pytest.raises(ValueError):
            validate_request(current)


@pytest.mark.parametrize('position_m,angle_deg,valid', [(0., 0., True), (.000019, .009, True),
                                                     (.000021, 0., False), (0., .011, False)])
def test_original_ik_precision_preserved(position_m, angle_deg, valid):
    assert accurate([20 * position_m, 0., 0., math.radians(angle_deg), 0., 0.]) is valid


@pytest.mark.parametrize('values', [[0.] * 5, [float('nan')] * 6, [float('inf')] * 6])
def test_ik_error_requires_finite_six_vector(values):
    assert not accurate(values)


@pytest.mark.parametrize('bucket_yaw', [1.8790664457948965, -1.2625262077948966])
def test_opposite_baskets_choose_same_minimal_yaw(bucket_yaw):
    target = nearest_symmetric_yaw(bucket_yaw + math.pi / 2, 0.)
    assert target == pytest.approx(.308270119, abs=.002)
    assert abs(target) < math.pi / 2


@pytest.mark.parametrize('target,current', [(float('nan'), 0.), (0., float('inf'))])
def test_nonfinite_yaw_rejected(target, current):
    with pytest.raises(ValueError):
        nearest_symmetric_yaw(target, current)


def test_geometric_hints_never_replace_inactive_arm_measurement():
    first = task_state()
    active = [name for name in first if name == 'torso_joint' or name.startswith('left_')]
    guides = [dict(first, **{name: .01 * index if name in active else 2. for name in first}) for index in range(3)]
    calls = []
    planner = SimpleNamespace(active=active)
    planner.solve = lambda target, seed, refine: dict(seed)
    planner.check_edge = lambda start, end, *args, **kwargs: calls.append((start, end, kwargs))
    result = SinglePlanner.guided_stage(planner, np.eye(4), np.eye(4), first, 'test_only', guides, support_start='table')
    assert result['hints_not_measured_state_or_frozen_runtime_reference']
    assert all(all(end[name] == first[name] for name in first if name not in active) for start, end, kwargs in calls)
    assert calls[0][2]['initial_support'] == 'table'
    assert calls[1][2]['initial_support'] == 'table_start_only'
    assert calls[2][2]['initial_support'] is None


def test_geometric_branch_jump_rejected_before_edge():
    first = task_state()
    planner = SimpleNamespace(active=['torso_joint'])
    planner.solve = lambda target, seed, refine: dict(seed, torso_joint=1.)
    planner.check_edge = lambda *args, **kwargs: pytest.fail('Rejected jump cannot reach collision dispatcher')
    with pytest.raises(RuntimeError, match='branches'):
        SinglePlanner.guided_stage(planner, np.eye(4), np.eye(4), first, 'test_only', [first] * 3)


def test_loaded_hint_is_blended_from_current_branch_without_first_point_jump():
    first = task_state()
    calls = []
    planner = SimpleNamespace(active=['torso_joint'])
    planner.solve = lambda target, seed, refine: dict(seed)
    planner.check_edge = lambda start, end, *args, **kwargs: calls.append((start, end))
    guide = dict(first, torso_joint=.2)
    planned = SinglePlanner.guided_stage(planner, np.eye(4), np.eye(4), first, 'test_only', [guide] * 5)
    assert calls[0][1] == first
    assert planned['samples'][-1]['torso_joint'] == pytest.approx(.2)
    assert all(abs(end['torso_joint'] - start['torso_joint']) <= .05 + 1e-12 for start, end in calls)


def test_rejected_hint_jump_retries_current_branch_without_relaxing_gate():
    first = task_state()
    calls = []
    planner = SimpleNamespace(active=['torso_joint'])

    def solve(target, seed, refine):
        calls.append(dict(seed))
        return dict(seed, torso_joint=1. if seed['torso_joint'] > .01 else .001)

    planner.solve = solve
    planner.check_edge = lambda *args, **kwargs: None
    guides = [dict(first, torso_joint=.2)] * 5
    result = SinglePlanner.guided_stage(planner, np.eye(4), np.eye(4), first, 'test_only', guides)
    assert result['original_step_and_precision_current_branch_fallback_indices']
    assert all(abs(row['torso_joint']) <= .12 for row in result['samples'])


def test_hints_are_proposals_not_execution_authority():
    hints = json.loads((EVIDENCE / 'inputs/sequential_hints.json').read_text())
    assert hints['scope'] == 'IK_BRANCH_PROPOSALS_NOT_ACTUAL_MEASUREMENTS_OR_EXECUTION_AUTHORITY'
    assert set(hints['loading']) == set(hints['retrieval']) == {'left', 'right'}
