import hashlib
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import dense_shortcut
import dual_worker
from dense_shortcut_source import restore_dense_function
from inherited_source_identity import restore_inherited_source


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_bucket_axis_exit_full_20261015_r282'


def request():
    return dict(mode='retrieval_pickup', retrieval=True,
                original_runtime_holder_references={})


def test_function_is_exactly_the_verified_three_edit_probe_and_inverse():
    candidate = inspect.getsource(dense_shortcut.free_path)
    assert hashlib.sha256(candidate.encode()).hexdigest() == '826bcb3e5b617c6cb37636b55e2dade2ee9ffeea8ad1bb1707305a1d25e0519e'
    restored = restore_dense_function(candidate)
    assert hashlib.sha256(restored.encode()).hexdigest() == '07551cfc35b7e6c4ea5d53fd0e0948934d815b317c7c7be53fcc50ae76946dd2'
    assert restored == inspect.getsource(dual_worker.free_path)
    assert '    def edge(start, end, resolution=.035):' in candidate
    assert 'edge(vectors[first_index], vectors[last_index], resolution=.002)' in candidate


@pytest.mark.parametrize('mode', ['load_pickup', 'load_transport', 'load_alignment',
                                'pilot', 'retrieval_transport', 'empty_return'])
def test_all_other_modes_keep_the_original_function(mode):
    assert dense_shortcut.select_free_path(dict(mode=mode), dual_worker.free_path) is dual_worker.free_path


def test_only_declared_empty_retrieval_pickup_uses_dense_shortcut():
    assert dense_shortcut.select_free_path(request(), dual_worker.free_path) is dense_shortcut.free_path


@pytest.mark.parametrize('value', [False, None, 1, 'true', [], {}])
def test_missing_or_nonboolean_retrieval_scope_fails_closed(value):
    current = dict(request(), retrieval=value)
    with pytest.raises(ValueError, match='scope'):
        dense_shortcut.select_free_path(current, dual_worker.free_path)


@pytest.mark.parametrize('holders', [None, {'left': {'box': 'left'}}, [], False])
def test_missing_or_nonempty_actual_holder_registry_fails_closed(holders):
    current = dict(request(), original_runtime_holder_references=holders)
    with pytest.raises(ValueError, match='holder'):
        dense_shortcut.select_free_path(current, dual_worker.free_path)


def test_worker_keeps_original_full_state_checks_and_only_declared_lift_provider():
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    source = (EVIDENCE / 'dual_worker.py').read_text()
    restored = restore_inherited_source('dual_worker.py', source)
    assert hashlib.sha256(restored.encode()).hexdigest() == records['dual_worker.py']['derived_sha256']
    assert hashlib.sha256((PARENT / 'dual_worker.py').read_bytes()).hexdigest() == records['dual_worker.py']['parent_sha256']
    method = inspect.getsource(dual_worker.PairedPlanner.empty_connector)
    assert 'select_free_path(self.request, free_path)(self.planner, first, last)' in method
    assert 'for fraction, joints in dense_edge(previous, target):' in method
    assert "self.planner.check(joints)['collision']" in method


@pytest.mark.parametrize('collision', [False, True])
def test_real_worker_selector_call_preserves_final_collision_rejection(monkeypatch, collision):
    calls = []
    current = request()
    first, last = {'joint': 0.}, {'joint': .01}

    def selected(planner, start, end):
        calls.append(('selected', planner, start, end))
        return [first, last]

    def selector(actual_request, legacy):
        calls.append(('selector', actual_request, legacy))
        return selected

    monkeypatch.setattr(dual_worker, 'select_free_path', selector)
    monkeypatch.setattr(dual_worker, 'dense_edge', lambda start, end: [(1., end)])
    planner = dual_worker.PairedPlanner.__new__(dual_worker.PairedPlanner)
    planner.request = current
    planner.checked = 0
    planner.planner = SimpleNamespace(initial={}, check=lambda joints: dict(collision=collision))
    if collision:
        with pytest.raises(RuntimeError, match='full-state dense FCL collision'):
            planner.empty_connector(first, last)
    else:
        assert planner.empty_connector(first, last) == [first, last]
        assert planner.checked == 2 and planner.planner.initial == last
    assert calls[0] == ('selector', current, dual_worker.free_path)
    assert calls[1] == ('selected', planner.planner, first, last)


def test_inherited_native_endpoint_and_timing_hooks_are_not_reversed_twice():
    for name in ('execute_fast.py', 'efficient_gripper_cycle.py'):
        source = (EVIDENCE / name).read_text()
        assert restore_inherited_source(name, source) == source
    records = {row['relative']: row for row in json.loads((EVIDENCE / 'derivation.json').read_text())['records']}
    restored = restore_inherited_source('execute_dual.py', (EVIDENCE / 'execute_dual.py').read_text())
    assert hashlib.sha256(restored.encode()).hexdigest() == records['execute_dual.py']['derived_sha256']
    source = restore_inherited_source('execute_cycle.py', (EVIDENCE / 'execute_cycle.py').read_text())
    assert 'from endpoint_overlap_cycle import EndpointOverlapCycle as DualCycle' in source
