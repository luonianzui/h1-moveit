import copy
from pathlib import Path

import numpy as np
import pytest

from dual_worker import paired_interpolation
from inherited_source_identity import restore_inherited_source
from lift_target_provider import stage_body_targets
from test_bucket_axis_exit import fixture


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_bucket_axis_exit_full_20261015_r282'
MODEL = ROOT.parents[1]/'src/h1_2_description/models/centrifuge/model.sdf'


def test_real_provider_uses_exact_qualified_prototype_targets_and_original_source_is_reversible():
    from bucket_axis_exit import bucket_axis_exit_targets
    from execute_loading import pose_matrix
    request,first,last,rims = fixture()
    original = copy.deepcopy(request)
    actual = list(stage_body_targets(request,'lift',first,last,81,None,paired_interpolation,MODEL))
    expected = bucket_axis_exit_targets(request,first,last,81,rims,pose_matrix,paired_interpolation)
    assert len(actual) == len(expected) == 81 and request == original
    for actual_row,expected_row in zip(actual,expected):
        for hand in ('left','right'):
            assert np.array_equal(actual_row[hand],expected_row[hand])
    assert restore_inherited_source('dual_worker.py',(ROOT/'dual_worker.py').read_text()) == (PARENT/'dual_worker.py').read_text()


@pytest.mark.parametrize('context',['other_mode','other_phase','prospective_empty','legacy_missing'])
def test_default_and_prospective_provider_remains_original_geometry(context):
    request,first,last,rims = fixture()
    phase = 'lift'
    if context == 'other_mode':
        request['mode'] = 'load_transport'
    elif context == 'other_phase':
        phase = 'outboard'
    elif context == 'prospective_empty':
        request['original_runtime_holder_references'] = {}
    else:
        request.pop('original_runtime_holder_references')
    actual = list(stage_body_targets(request,phase,first,last,81,None,paired_interpolation,MODEL))
    expected = list(paired_interpolation(first,last,81))
    for actual_row,expected_row in zip(actual,expected):
        for hand in ('left','right'):
            assert np.array_equal(actual_row[hand],expected_row[hand])


@pytest.mark.parametrize('invalid',['one_holder','wrong_payload','future_reference','missing_source','count','guides','not_retrieval'])
def test_qualified_provider_cannot_use_wrong_actual_identity_or_changed_original_contract(invalid):
    request,first,last,rims = fixture()
    count,guides = 81,None
    if invalid == 'one_holder':
        request['original_runtime_holder_references'].pop('right')
    elif invalid == 'wrong_payload':
        request['original_runtime_holder_references']['right']['payload'] = 'left'
    elif invalid == 'future_reference':
        request['original_runtime_holder_references']['right']['source_sim'] = request['source_sim']+1
    elif invalid == 'missing_source':
        request.pop('source_sim')
    elif invalid == 'count':
        count = 161
    elif invalid == 'guides':
        guides = []
    else:
        request['retrieval'] = False
    with pytest.raises(ValueError):
        stage_body_targets(request,'lift',first,last,count,guides,paired_interpolation,MODEL)
