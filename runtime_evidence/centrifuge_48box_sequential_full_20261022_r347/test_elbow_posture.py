import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from elbow_posture import ACTIVE_MODES, ELBOW_CONTRACT, accurate, active, choose_elbow_pickup, refine
from elbow_source import restore_elbow


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_minimum_turn_full_20261015_r285'


@pytest.mark.parametrize('mode', sorted(ACTIVE_MODES))
def test_only_explicit_current_contract_changes_grasp_or_loaded_ik(mode):
    assert active(dict(mode=mode, elbow_posture_contract=ELBOW_CONTRACT))
    assert not active(dict(mode=mode))


@pytest.mark.parametrize('mode', ['empty_return', 'empty_gripper_after_retreat', 'empty_return_after_gripper'])
def test_empty_retreat_protocol_stays_original(mode):
    assert not active(dict(mode=mode, elbow_posture_contract=ELBOW_CONTRACT))


def test_unknown_contract_is_not_silently_accepted():
    with pytest.raises(ValueError):
        active(dict(mode='pilot', elbow_posture_contract='unknown'))


@pytest.mark.parametrize('offset', [0, 6])
def test_both_wrist_position_and_orientation_limits_are_original(offset):
    errors = np.zeros(12)
    assert accurate(errors)
    errors[offset] = .000021 * 20
    assert not accurate(errors)
    errors[:] = 0
    errors[offset + 3] = np.deg2rad(.011)
    assert not accurate(errors)


@pytest.mark.parametrize('values', [np.full(12, np.nan), np.full(12, np.inf), np.zeros(6)])
def test_nonfinite_or_incomplete_ik_rejected(values):
    assert not accurate(values)


def test_unmarked_history_delegates_to_identical_original_solver():
    measured = {'old': True}
    observed = []
    planner = SimpleNamespace(request={})
    def original(bodies, seed):
        observed.append((bodies, seed))
        return measured
    assert refine(planner, {'bodies': True}, {'seed': True}, original) is measured
    assert observed == [({'bodies': True}, {'seed': True})]


@pytest.mark.parametrize('name', ['dual_worker.py', 'minimum_turn.py'])
def test_elbow_hooks_exactly_reverse_to_direct_frozen_parent(name):
    expected = (PARENT / name).read_text().replace(str(PARENT), str(ROOT))
    expected = expected.replace('install_lab_scene_48box_minimum_turn_full_20261015_r285',
                                'install_lab_scene_48box_sequential_full_20261022_r347')
    assert restore_elbow(name, (ROOT / name).read_text()) == expected


def test_bound_hints_are_not_an_actual_current_path_certificate():
    data = json.loads((ROOT / 'inputs/elbow_seed_hints.json').read_text())
    assert data['scope'] == 'PLANNING_HINTS_ONLY_REQUIRE_CURRENT_FULL_DENSE_FCL'
    assert data['no_current_scene_or_physical_certificate_inherited']
    assert set(data['hints']) == {'load_pickup', 'retrieval_pickup'}
    assert all(row['source'].startswith(str(PARENT)) and len(row['source_sha256']) == 64
               for row in data['hints'].values())


def test_held_reference_cannot_choose_new_symmetric_grasp():
    request = dict(mode='load_pickup', elbow_posture_contract=ELBOW_CONTRACT,
                   original_runtime_holder_references={'left': [[1]]})
    with pytest.raises(ValueError, match='actually holding'):
        choose_elbow_pickup(request, SimpleNamespace())
