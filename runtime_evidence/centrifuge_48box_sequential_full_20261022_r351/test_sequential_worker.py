import copy
import math
from types import SimpleNamespace

import numpy as np
import pytest

import sequential_worker as worker
from test_sequential_geometry import request


def loaded_request(monkeypatch):
    current = request()
    identity = np.eye(4).tolist()
    current.update(mode='load_pilot', runtime_reference_not_rebased=True, planning_reference=identity,
                   original_runtime_holder_references={'left': dict(payload='left', source_sim=1.,
                       native_reference=identity, fk_reference=identity)},
                   poses={'box48_left': dict(position={}, orientation=dict(w=1.)),
                          'left_wrist_yaw_link': dict(position={}, orientation=dict(w=1.))})
    monkeypatch.setattr(worker, 'SinglePlanner', lambda source: SimpleNamespace(names=list(source['positions']), wrist=lambda joints: np.eye(4)))
    return current


def test_current_planning_fk_keeps_original_anchors(monkeypatch):
    current = loaded_request(monkeypatch)
    original = copy.deepcopy(current)
    assert np.array_equal(worker.loaded_reference(current), np.eye(4))
    assert current == original


@pytest.mark.parametrize('mode', sorted(worker.LOADED_MODES))
def test_loaded_dispatch_requires_actual_holder(mode):
    current = request()
    current['mode'] = mode
    with pytest.raises(ValueError, match='original actual holder'):
        worker.plan(current)


@pytest.mark.parametrize('mode', sorted(worker.EMPTY_MODES))
def test_empty_dispatch_cannot_ignore_holder(mode):
    current = request()
    current.update(mode=mode, original_runtime_holder_references={'left': dict(payload='left')})
    with pytest.raises(ValueError, match='empty-phase'):
        worker.plan(current)


def test_planning_transform_cannot_be_historical(monkeypatch):
    current = loaded_request(monkeypatch)
    current['planning_reference'] = np.eye(4).tolist()
    current['planning_reference'][0][3] = 1e-6
    with pytest.raises(ValueError, match='current actual FK'):
        worker.loaded_reference(current)


@pytest.mark.parametrize('reference', ['native_reference', 'fk_reference'])
def test_original_two_mm_slip_not_relaxed(monkeypatch, reference):
    current = loaded_request(monkeypatch)
    current['original_runtime_holder_references']['left'][reference] = np.eye(4).tolist()
    current['original_runtime_holder_references']['left'][reference][0][3] = .00201
    with pytest.raises(ValueError, match='two-mm'):
        worker.loaded_reference(current)


@pytest.mark.parametrize('reference', ['native_reference', 'fk_reference'])
def test_original_five_degree_angle_not_relaxed(monkeypatch, reference):
    current = loaded_request(monkeypatch)
    matrix = np.eye(4)
    angle = math.radians(5.01)
    matrix[:2, :2] = [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
    current['original_runtime_holder_references']['left'][reference] = matrix.tolist()
    with pytest.raises(ValueError, match='five-degree'):
        worker.loaded_reference(current)


@pytest.mark.parametrize('value', [False, None])
def test_no_reference_rebase_contract_required(monkeypatch, value):
    current = loaded_request(monkeypatch)
    current['runtime_reference_not_rebased'] = value
    with pytest.raises(ValueError):
        worker.loaded_reference(current)


def test_dispatch_never_claims_physics_from_geometry(monkeypatch):
    current = request()
    current['mode'] = 'park'
    monkeypatch.setattr(worker, 'park', lambda source: dict(status='PASS_GEOMETRY_ONLY'))
    result = worker.plan(current)
    assert result['status'] == 'PASS_GEOMETRY_ONLY'
    assert result['execution_dispatch_not_qualified'] is True


def test_dispatch_rejects_mutation_of_actual_snapshot(monkeypatch):
    current = request()
    current['mode'] = 'park'

    def mutate(source):
        source['positions']['torso_joint'] += .1
        return dict(status='PASS_GEOMETRY_ONLY')

    monkeypatch.setattr(worker, 'park', mutate)
    with pytest.raises(RuntimeError, match='mutated'):
        worker.plan(current)


@pytest.mark.parametrize('mode', ['load_pair', 'retrieval', 'reset', 'attach', None])
def test_unknown_or_physical_operation_modes_rejected(mode):
    current = request()
    current['mode'] = mode
    with pytest.raises(ValueError, match='Unknown'):
        worker.plan(current)
