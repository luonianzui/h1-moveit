import copy
import json
from pathlib import Path

import numpy as np
import pytest

from execute_loading import pose_matrix
from micro_alignment import alignment_stage_parameters


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent/'centrifuge_48box_bucket_axis_exit_full_20261015_r282'


def fixture():
    request = json.loads((PARENT/'run/4_load_alignment_request.json').read_text())
    route = json.loads((PARENT/'inputs/dual_route.json').read_text())
    first = {hand:pose_matrix(request['poses']['box48_'+payload]) for hand,payload in request['assignment'].items()}
    last = {hand:np.asarray(matrix) for hand,matrix in route['reverse_stages'][0]['first_bodies'].items()}
    guides = route['forward_stages'][-1]['samples']
    return request,first,last,guides


def test_actual_small_alignment_uses_same_body_goals_references_original_rate_and_warm_current_seed():
    request,first,last,guides = fixture()
    before = copy.deepcopy(request)
    assert alignment_stage_parameters(request,'forward_inside_basket_raise',first,last,81,guides)==(11,None)
    assert request==before
    result = json.loads((ROOT/'current_micro_alignment_probe.json').read_text())
    assert result['status']=='PASS_OFFLINE_CURRENT_HELPER_NOT_PHYSICAL'
    assert result['candidate_command_duration_sim_s']<result['original_command_duration_sim_s']-2.5


@pytest.mark.parametrize('context',['mode','phase','no_holders','distance','angle'])
def test_unselected_large_or_prospective_case_keeps_parent_parameters(context):
    request,first,last,guides = fixture()
    phase = 'forward_inside_basket_raise'
    if context=='mode':
        request['mode']='load_transport'
    elif context=='phase':
        phase='outboard'
    elif context=='no_holders':
        request['original_runtime_holder_references']={}
    elif context=='distance':
        last['left'][0,3]=first['left'][0,3]+.006
    else:
        from scipy.spatial.transform import Rotation
        last['left'][:3,:3]=first['left'][:3,:3]@Rotation.from_euler('z',2,degrees=True).as_matrix()
    assert alignment_stage_parameters(request,phase,first,last,81,guides)==(81,guides)


@pytest.mark.parametrize('invalid',['one_holder','wrong_payload','future_epoch','nonfinite','count','missing_guides','retrieval'])
def test_wrong_original_context_or_nonfinite_matrix_rejected(invalid):
    request,first,last,guides = fixture()
    count=81
    if invalid=='one_holder':
        request['original_runtime_holder_references'].pop('right')
    elif invalid=='wrong_payload':
        request['original_runtime_holder_references']['right']['payload']='left'
    elif invalid=='future_epoch':
        request['original_runtime_holder_references']['right']['source_sim']=request['source_sim']+1
    elif invalid=='nonfinite':
        first['left'][0,3]=float('nan')
    elif invalid=='count':
        count=11
    elif invalid=='missing_guides':
        guides=None
    else:
        request['retrieval']=True
    with pytest.raises(ValueError):
        alignment_stage_parameters(request,'forward_inside_basket_raise',first,last,count,guides)
