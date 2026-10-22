import inspect
import json
from pathlib import Path

import pytest

import dense_tree


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_empty_protocol_full_20261015_r280'


def test_exact_inverse_preserves_rng_bounds_deadline_iterations_and_every_other_statement():
    candidate = (ROOT / 'dense_tree.py').read_text()
    assert candidate.count('resolution=.002') == 2
    assert candidate.replace('resolution=.002', 'resolution=.035', 1) == (PARENT / 'dense_shortcut.py').read_text()


def test_only_empty_actual_retrieval_selects_denser_tree():
    previous = object()
    request = dict(mode='retrieval_pickup', retrieval=True, original_runtime_holder_references={})
    assert dense_tree.select_free_path(request, previous) is dense_tree.free_path
    request['mode'] = 'load_pickup'
    assert dense_tree.select_free_path(request, previous) is previous


@pytest.mark.parametrize('invalid', ['retrieval', 'holder', 'missing_holder'])
def test_wrong_scope_is_rejected_before_any_geometry_or_robot_action(invalid):
    request = dict(mode='retrieval_pickup', retrieval=True, original_runtime_holder_references={})
    if invalid == 'retrieval':
        request['retrieval'] = False
    elif invalid == 'holder':
        request['original_runtime_holder_references'] = dict(left=dict(payload='left'))
    else:
        request.pop('original_runtime_holder_references')
    with pytest.raises(ValueError):
        dense_tree.select_free_path(request, object())


def test_original_failed_scene_result_remains_failed_not_relabelled():
    data = json.loads((PARENT / 'run/7_retrieval_pickup_result.json').read_text())
    assert data['status'] == 'FAIL'
    assert 'Paired empty full-state dense FCL collision' in data['error']
