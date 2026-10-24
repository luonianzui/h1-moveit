from copy import deepcopy
import itertools
from pathlib import Path

import numpy as np
import pytest

from table_bucket_clearance import bucket_collision_bounds, footprint_clearance, TABLE_SETDOWN_PHASES


def corners(minimum_x=.2, minimum_y=.2, minimum_z=-.1):
    return np.array(list(itertools.product((minimum_x, minimum_x + .08),
                                          (minimum_y, minimum_y + .06),
                                          (minimum_z, minimum_z + .05))))


def bounds():
    return {bucket: dict(minimum=[-.045, -.067, -.030], maximum=[.045, .067, .025])
            for bucket in ('left', 'right')}


def test_low_table_payload_far_from_both_buckets_is_geometrically_clear():
    sample = corners()
    result = footprint_clearance(dict(left=sample, right=sample), bounds(), [])
    assert result['valid'] and result['clearance_m'] > .1
    assert result['required_margin_m'] == .003


@pytest.mark.parametrize('height', [-.1, .01, .1])
def test_overlapping_bucket_footprint_cannot_enter_table_phase_even_if_above_rim(height):
    sample = corners(-.04, -.06, height)
    assert not footprint_clearance(dict(left=sample, right=sample), bounds(), [])['valid']


def test_both_buckets_must_be_clear_not_just_the_named_payload_bucket():
    assert not footprint_clearance(dict(left=corners(), right=corners(-.04, -.06)), bounds(), [])['valid']


@pytest.mark.parametrize('horizontal,valid', [(.04799, False), (.0481, True)])
def test_original_three_mm_minimum_is_not_reduced(horizontal, valid):
    sample = corners(horizontal, -.02)
    assert footprint_clearance(dict(left=sample, right=sample), bounds(), [])['valid'] == valid


def test_zero_force_basket_contact_is_still_rejected():
    pair = dict(first='box48_left::body::collision', second='centrifuge::bucket_right::collision', force=0)
    assert not footprint_clearance(dict(left=corners(), right=corners()), bounds(), [pair])['valid']


def test_table_contact_does_not_replace_the_separate_qualified_support_gate():
    pair = dict(first='box48_left::body::collision', second='table::surface::collision', force=.3)
    assert footprint_clearance(dict(left=corners(), right=corners()), bounds(), [pair])['valid']


def test_nonfinite_native_geometry_and_missing_bucket_fail_closed():
    sample = corners()
    sample[0, 2] = np.nan
    assert not footprint_clearance(dict(left=sample, right=corners()), bounds(), [])['valid']
    assert not footprint_clearance(dict(left=corners()), bounds(), [])['valid']
    invalid = deepcopy(bounds())
    invalid['right']['maximum'][0] = float('inf')
    assert not footprint_clearance(dict(left=corners(), right=corners()), invalid, [])['valid']


def test_bounds_are_from_original_transformed_collision_stl():
    root = Path(__file__).resolve().parents[2]
    result = bucket_collision_bounds(root / 'src/h1_2_description/models/centrifuge/model.sdf')
    assert set(result) == {'left', 'right'}
    for row in result.values():
        assert row['maximum'][2] == pytest.approx(.025, abs=.00001)
        assert all(first < last for first, last in zip(row['minimum'], row['maximum']))


def test_alternate_guard_is_only_for_the_two_table_setdown_phases():
    assert TABLE_SETDOWN_PHASES == {'paired_retrieval_setdown', 'paired_retrieval_setdown_fine'}
    assert 'paired_retrieval_lift' not in TABLE_SETDOWN_PHASES
    assert 'paired_retrieval_static_grip_hold' not in TABLE_SETDOWN_PHASES
