import copy
import itertools
import json
import math
from pathlib import Path

import numpy as np
import pytest

from bucket_axis_exit import bucket_axis_exit_targets
from dual_worker import paired_interpolation
from execute_loading import pose_matrix
from extraction_clearance import bucket_rim_bounds


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_dense_endpoint_full_20261015_r281'


def fixture():
    request = json.loads((PARENT/'run/9_retrieval_transport_request.json').read_text())
    first = {hand:pose_matrix(request['poses']['box48_'+payload]) for hand,payload in request['assignment'].items()}
    last = copy.deepcopy(first)
    for body in last.values():
        body[2,3] = 1.15975
    rims = bucket_rim_bounds(PARENT.parents[1]/'src/h1_2_description/models/centrifuge/model.sdf')
    return request,first,last,rims


def test_count_original_endpoints_world_orientations_and_actual_source_not_rebased():
    request,first,last,rims = fixture()
    original = copy.deepcopy(request)
    targets = bucket_axis_exit_targets(request,first,last,81,rims,pose_matrix,paired_interpolation)
    assert len(targets) == 81 and request == original
    for hand in first:
        assert np.allclose(targets[0][hand],first[hand],atol=1e-12,rtol=0)
        assert np.allclose(targets[-1][hand],last[hand],atol=1e-12,rtol=0)
        assert all(np.allclose(row[hand][:3,:3],first[hand][:3,:3],atol=1e-12,rtol=0) for row in targets)


def test_intermediate_centered_in_original_bucket_frame_and_above_mesh_rim_by_six_mm():
    request,first,last,rims = fixture()
    targets = bucket_axis_exit_targets(request,first,last,81,rims,pose_matrix,paired_interpolation)
    corners = np.array(list(itertools.product((-1.,1.),repeat=3))) * np.array([.1187450051085,.08814010043,.0595])/2
    machine = pose_matrix(request['poses']['centrifuge'])
    for hand,payload in request['assignment'].items():
        bucket = machine @ pose_matrix(request['poses']['bucket_'+payload])
        relative = np.linalg.inv(bucket) @ targets[40][hand]
        assert np.max(np.abs(relative[:2,3])) < 1e-12
        assert (corners @ relative[:3,:3].T + relative[:3,3])[:,2].min() >= rims[payload]+.006-1e-12


@pytest.mark.parametrize('invalid',['mode','retrieval','holders','assignment','count','nonfinite','rim'])
def test_wrong_scope_bad_identity_nonfinite_or_changed_target_count_rejected(invalid):
    request,first,last,rims = fixture()
    count = 81
    if invalid == 'mode':
        request['mode'] = 'load_transport'
    elif invalid == 'retrieval':
        request['retrieval'] = False
    elif invalid == 'holders':
        request['original_runtime_holder_references'].pop('right')
    elif invalid == 'assignment':
        request['assignment']['right'] = request['assignment']['left']
    elif invalid == 'count':
        count = 161
    elif invalid == 'nonfinite':
        first['right'][0,3] = math.nan
    else:
        rims['right'] = math.nan
    with pytest.raises(ValueError):
        bucket_axis_exit_targets(request,first,last,count,rims,pose_matrix,paired_interpolation)


def test_original_failed_result_stays_failed_and_new_offline_result_requires_fresh_execution():
    assert json.loads((PARENT/'run/9_retrieval_transport_result.json').read_text())['status'] == 'FAIL'
    data = json.loads((ROOT/'current_bucket_exit_probe.json').read_text())
    assert data['status'] == 'PASS_OFFLINE_CURRENT_PROVIDER_NOT_PHYSICAL'
    assert data['no_launch_command_attachment_rebase_or_collision_exemption']
    assert len(next(stage for stage in data['result']['stages'] if stage['phase']=='lift')['samples']) == 82
