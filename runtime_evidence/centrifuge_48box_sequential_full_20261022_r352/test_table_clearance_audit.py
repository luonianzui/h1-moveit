from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from execute_loading import pose_matrix
from table_bucket_clearance import bucket_collision_bounds, table_bucket_clearance
from table_clearance_audit import recompute_table_clearance, verify_table_record


EVIDENCE = Path(__file__).resolve().parent


def measured(payload):
    sample = json.loads((EVIDENCE / 'actual_table_clearance_native_inputs.json').read_text())
    bounds = bucket_collision_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')
    poses = deepcopy(sample['fault']['payload']['poses'])
    contacts = deepcopy(sample['contacts'][payload]['pairs'])
    return dict(payload=payload, poses=poses, contacts=contacts,
                clearance=table_bucket_clearance(poses, payload, contacts, bounds)), bounds


@pytest.mark.parametrize('payload', ['left', 'right'])
def test_direct_native_transform_independently_matches_runtime_projection(payload):
    data, bounds = measured(payload)
    independent = recompute_table_clearance(data['poses'], payload, data['contacts'], bounds)
    assert independent['valid'] and independent['clearance_m'] > .1
    assert independent['clearance_m'] == pytest.approx(data['clearance']['clearance_m'], abs=1e-10)
    assert verify_table_record('paired_retrieval_setdown', data, bounds)['valid']


def test_self_reported_valid_cannot_hide_bad_distance_or_actual_overlap():
    data, bounds = measured('left')
    data['clearance']['clearance_m'] = 1.
    assert not verify_table_record('paired_retrieval_setdown', data, bounds)['valid']
    data, bounds = measured('left')
    center = (pose_matrix(data['poses']['centrifuge']) @ pose_matrix(data['poses']['bucket_left']))[:3, 3]
    data['poses']['box48_left']['position'] = dict(zip(('x', 'y', 'z'), center.tolist()))
    assert not verify_table_record('paired_retrieval_setdown', data, bounds)['valid']


def test_wrong_phase_and_reduced_reported_margin_do_not_pass():
    data, bounds = measured('right')
    assert not verify_table_record('paired_retrieval_lift', data, bounds)['valid']
    data['clearance']['required_margin_m'] = .001
    assert not verify_table_record('paired_retrieval_setdown_fine', data, bounds)['valid']
