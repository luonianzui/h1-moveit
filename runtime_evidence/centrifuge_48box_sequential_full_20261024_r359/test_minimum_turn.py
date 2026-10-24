from acceleration_source import restore_acceleration, restore_acceleration_bytes
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from dual_motion import TASK_JOINTS
from minimum_turn import CONTRACT, branch_seed, choose_pickup, direct_empty_connector, motion_metrics, seed_candidates
from minimum_turn_source import restore_minimum_turn


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_front_camera_full_20261015_r284'


def test_worker_change_exactly_reverses_to_direct_frozen_parent():
    expected = (PARENT / 'dual_worker.py').read_text().replace(str(PARENT), str(ROOT))
    expected = expected.replace('install_lab_scene_48box_front_camera_full_20261015_r284',
                                'install_lab_scene_48box_sequential_full_20261024_r359').replace(
        'centrifuge_48box_front_camera_full_r284', 'centrifuge_48box_sequential_full_r349')
    assert restore_minimum_turn('dual_worker.py', (ROOT / 'dual_worker.py').read_text()) == expected


@pytest.mark.parametrize('name', ['execute_dual.py', 'execute_cycle.py', 'dual_motion.py', 'soft_grasp.py',
                                 'inputs/robot_dual.urdf', 'inputs/controllers_servo.yaml',
                                 'inputs/centrifuge_brake.sdf', 'inputs/48box_friction.sdf', 'inputs/lab_fast_world.sdf'])
def test_all_physical_guard_lifecycle_geometry_and_timing_bytes_preserved(name):
    expected = (PARENT / name).read_bytes().replace(str(PARENT).encode(), str(ROOT).encode()).replace(
        b'install_lab_scene_48box_front_camera_full_20261015_r284',
        b'install_lab_scene_48box_sequential_full_20261024_r359').replace(
        b'centrifuge_48box_front_camera_full_r284', b'centrifuge_48box_sequential_full_r349')
    assert restore_acceleration_bytes(name, (ROOT / name).read_bytes()) == expected


@pytest.mark.parametrize('mode', ['load_pickup', 'retrieval_pickup', 'pilot', 'load_transport',
                                 'load_alignment', 'retrieval_transport'])
def test_only_declared_candidate_modes_prefer_continuous_actual_seed(mode):
    current, guide = {'torso_joint': .1}, {'torso_joint': -.2}
    assert branch_seed(dict(mode=mode, minimum_turn_contract=CONTRACT), 'stage', current, [guide], 0) is current
    assert branch_seed(dict(mode=mode), 'stage', current, [guide], 0) is guide


def test_empty_retreat_and_unmarked_history_still_use_original_guides():
    current, guide = {}, {'marker': True}
    assert branch_seed(dict(mode='empty_return', minimum_turn_contract=CONTRACT), 'inside', current, [guide], 100) is guide
    assert branch_seed({}, 'inside', current, None, 0) is current
    with pytest.raises(ValueError):
        branch_seed(dict(mode='pilot', minimum_turn_contract='unknown'), 'lift', current, [guide], 0)


def test_cost_penalizes_real_unwrapped_joint_travel_not_flip_labels():
    first = {'left_wrist_roll_joint': 0., 'torso_joint': 0.}
    small = motion_metrics(first, [dict(first, left_wrist_roll_joint=.1)], first)
    turn = motion_metrics(first, [dict(first, left_wrist_roll_joint=2 * math.pi)], first)
    zigzag = motion_metrics(first, [dict(first, left_wrist_roll_joint=-.2),
                                    dict(first, left_wrist_roll_joint=.1)], first)
    assert small['score'] < zigzag['score'] < turn['score']
    assert turn['axial_rad'] == 2 * math.pi


@pytest.mark.parametrize('points', [[], [{'torso_joint': float('nan')}], [{'wrong': 0.}]])
def test_incomplete_or_nonfinite_scoring_rejected(points):
    with pytest.raises(ValueError):
        motion_metrics({'torso_joint': 0.}, points, ['torso_joint'])


def test_seed_search_is_deterministic_and_stays_in_original_ranges():
    names = ['torso_joint', 'left_wrist_roll_joint']
    planner = SimpleNamespace(names=names, lower=np.array([-.5, -1.]), upper=np.array([.5, 1.]))
    home = dict(zip(names, [0., 0.]))
    guide = dict(zip(names, [.1, .2]))
    first = seed_candidates(planner, home, guide, home)
    assert first == seed_candidates(planner, home, guide, home)
    assert len(first) == 12
    assert all(np.all(np.array([seed[name] for name in names]) >= planner.lower) and
               np.all(np.array([seed[name] for name in names]) <= planner.upper) for seed in first)


def test_direct_connector_checks_dense_real_geometry_and_rejects_collision():
    observed = []
    def check(joints):
        observed.append(joints['torso_joint'])
        return {'collision': abs(joints['torso_joint'] - .01) < .001}
    planner = SimpleNamespace(names=['torso_joint'], lower=np.array([-.5]), upper=np.array([.5]),
                              planner=SimpleNamespace(check=check), checked=0)
    with pytest.raises(ValueError, match='collision'):
        direct_empty_connector(planner, {'torso_joint': 0.}, {'torso_joint': .02})
    assert any(abs(value - .01) < .001 for value in observed)
    assert max(np.diff(observed)) <= .0020000001


def test_failed_alternatives_keep_original_qualified_pickup_not_a_fake_improvement():
    first = {name: 0. for name in TASK_JOINTS}
    last = {name: .1 for name in TASK_JOINTS}
    baseline = dict(status='PASS', flips_deg=[0, 0], checked_samples=0, stages=[dict(phase='approach', samples=[first, last]),
                                         dict(phase='descend', samples=[last])])
    class FailedPlanner:
        names = sorted(TASK_JOINTS)
        lower = np.full(len(names), -3.)
        upper = np.full(len(names), 3.)
        def __init__(self, request, references):
            pass
        def solve(self, targets, seed):
            raise RuntimeError('No valid original-precision IK')
        planner = SimpleNamespace(check=lambda joints: {'collision': True})
        checked = 0
    worker = SimpleNamespace(legacy_plan_pickup=lambda request: baseline, TASK_JOINTS=TASK_JOINTS,
        EVIDENCE=ROOT, json=json, PairedPlanner=FailedPlanner, pose_matrix=lambda pose: np.eye(4),
        nominal_references=lambda: {'left': np.eye(4), 'right': np.eye(4)})
    request = dict(minimum_turn_contract=CONTRACT, mode='load_pickup', retrieval=False,
                   original_runtime_holder_references={}, positions=first,
                   assignment={'left': 'left', 'right': 'right'}, poses={'box48_left': {}, 'box48_right': {}})
    result = choose_pickup(request, worker)
    assert result['stages'] == baseline['stages']
    assert result['minimum_turn_selection']['improved'] is False
    request['original_runtime_holder_references'] = {'left': {'payload': 'left'}}
    with pytest.raises(ValueError, match='held reference'):
        choose_pickup(request, worker)


def test_new_first_frame_helper_fixes_decimal_only():
    assert (ROOT / 'review_first_frame.py').read_text() == (PARENT / 'review_first_frame.py').read_text().replace(
        "'-ss', '.5'", "'-ss', '0.5'", 1)


def test_bounded_alternate_search_keeps_every_original_dense_geometry_check():
    parent = (PARENT / 'dense_tree.py').read_text()
    parent = parent[:parent.index('\n\ndef select_free_path')]
    candidate = (ROOT / 'bounded_connector.py').read_text()
    assert candidate.replace('def bounded_free_path(planner, first, last):',
                             'def free_path(planner, first, last):', 1).replace(
        'deadline = time.monotonic() + 6.', 'deadline = time.monotonic() + 120', 1) == parent
