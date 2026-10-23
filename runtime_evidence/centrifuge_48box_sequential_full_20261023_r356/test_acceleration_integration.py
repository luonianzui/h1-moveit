import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from acceleration_source import restore_acceleration, CURRENT_PICKUP_DECLARATION
from acceleration_inputs import verify_acceleration_input
from leg_initialization_contract import LEG_JOINTS
from leg_initialization_runtime import verify_initialization
import sequential_worker as worker

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r318'


@pytest.mark.parametrize('name', ['execute_cycle.py', 'sequential_worker.py', 'inputs/robot_dual.urdf'])
def test_declared_delta_reverses_exactly_and_does_not_hide_unrelated_edits(name):
    actual = (ROOT / name).read_text()
    original = (PARENT / name).read_text().replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name)
    original = original.replace('install_lab_scene_48box_sequential_full_20261020_r318',
                                'install_lab_scene_48box_sequential_full_20261023_r356')
    assert restore_acceleration(name, actual) == original
    altered = actual.replace('import ', 'import unrelated_', 1) if name.endswith('.py') else actual.replace('<inertia ', '<inertia invalid="1" ', 1)
    assert restore_acceleration(name, altered) != original


def test_ground_trimmed_foot_proxy_preserves_whole_manipulation_model():
    source = (PARENT / 'inputs/robot_dual.urdf').read_text().replace(str(PARENT), str(ROOT)).replace(
        'install_lab_scene_48box_sequential_full_20261020_r318', 'install_lab_scene_48box_sequential_full_20261023_r356')
    current = (ROOT / 'inputs/robot_dual.urdf').read_bytes()
    result = verify_acceleration_input(source.encode(), current, 'robot_dual.urdf', ROOT.parents[1] / 'src/h1_2_description')
    assert len(result['records']) == 2
    assert all(row['target_lowest_world_z'] == 0. and not row['all_original_vertices_contained'] for row in result['records'])
    with pytest.raises(ValueError):
        verify_acceleration_input(source.encode(), current.replace(b'<mass value="', b'<mass value="9', 1),
                                  'robot_dual.urdf', ROOT.parents[1] / 'src/h1_2_description')


@pytest.mark.parametrize('mode', ['load_pickup', 'retrieval_pickup'])
def test_pickup_does_not_run_future_motion_or_mutate_snapshot(monkeypatch, mode):
    request = dict(mode=mode, sequential_contract='test', original_runtime_holder_references={})
    monkeypatch.setattr(worker, 'validate_request', lambda request: None)
    expected = dict(stages=[dict(samples=[dict(joint=.1)])], checked_samples=21)
    monkeypatch.setattr(worker, 'pickup', lambda *args, **kwargs: copy.deepcopy(expected))
    def forbidden(*args, **kwargs):
        raise AssertionError('Speculative future motion was invoked')
    for name in ('pilot', 'load_transport', 'retrieve_transport', 'SinglePlanner'):
        monkeypatch.setattr(worker, name, forbidden)
    original = copy.deepcopy(request)
    result = worker.plan(request)
    assert result['stages'] == expected['stages'] and request == original
    assert result['prospective_carry_geometry']['status'] == 'NOT_COMPUTED_NOT_A_CERTIFICATE'


def test_real_loaded_mode_still_requires_original_holder_validation(monkeypatch):
    monkeypatch.setattr(worker, 'validate_request', lambda request: None)
    def rejected(request):
        raise ValueError('Actual anchor invalid')
    monkeypatch.setattr(worker, 'loaded_reference', rejected)
    with pytest.raises(ValueError, match='Actual anchor'):
        worker.plan(dict(mode='load_pilot', sequential_contract='test'))


def initialization_fixture(root):
    (root / 'run').mkdir()
    (root / 'inputs').mkdir()
    initial = dict.fromkeys(LEG_JOINTS, .09)
    plan = json.dumps(dict(initial_positions=initial))
    (root / 'inputs/plan.json').write_text(plan)
    report = dict(status='PASS_STATIONARY_INITIALIZATION_ONLY', action_error_code=0,
                  original_plan_sha256=hashlib.sha256(plan.encode()).hexdigest(),
                  original_targets=[.09] * 12, hold_sim_seconds=5., hold_peak_error_rad=.0001,
                  after=initial)
    return initial, report


@pytest.mark.parametrize('field,value', [('status', 'FAIL'), ('action_error_code', -1),
    ('original_plan_sha256', 'invalid'), ('original_targets', [.01] * 12),
    ('hold_sim_seconds', 4.9), ('hold_peak_error_rad', .005)])
def test_invalid_startup_proof_rejected(tmp_path, field, value):
    initial, report = initialization_fixture(tmp_path)
    report[field] = value
    (tmp_path / 'run/leg_initialization.json').write_text(json.dumps(report))
    with pytest.raises(ValueError):
        verify_initialization(tmp_path, initial, initial)


def test_actual_endpoint_drift_cannot_use_old_pass(tmp_path):
    initial, report = initialization_fixture(tmp_path)
    (tmp_path / 'run/leg_initialization.json').write_text(json.dumps(report))
    assert verify_initialization(tmp_path, initial, initial) == report
    with pytest.raises(ValueError):
        verify_initialization(tmp_path, initial, dict.fromkeys(LEG_JOINTS, .11))


def test_original_targets_holds_force_and_posture_path_unchanged():
    for name in ('inputs/plan.json', 'inputs/button_plan.json', 'inputs/sequential_hints.json',
                 'sequential_planning.py', 'sequential_cycle.py', 'sequential_force.py',
                 'grip_limits.py', 'single_setdown.py', 'pilot_leveling.py', 'sequential_order.py'):
        current = (ROOT / name).read_bytes()
        if name == 'sequential_cycle.py':
            current = current.replace(b'from bounded_release_alignment import ', b'from release_alignment import ')
        assert current == (PARENT / name).read_bytes(), name
