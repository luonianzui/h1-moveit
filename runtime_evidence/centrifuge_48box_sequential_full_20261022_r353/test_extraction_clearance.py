import inspect
from pathlib import Path

import numpy as np
import pytest

from execute_cycle import FullCycle
from extraction_clearance import bucket_rim_bounds, corner_clearance


ROOT = Path(__file__).resolve().parents[2]


def corners(bottom):
    return np.array([[horizontal, lateral, vertical] for horizontal in (-.044, .044)
                     for lateral in (-.059, .059) for vertical in (bottom, bottom + .0595)])


def test_rim_is_transformed_original_collision_mesh_not_untransformed_stl():
    bounds = bucket_rim_bounds(ROOT / 'src/h1_2_description/models/centrifuge/model.sdf')
    assert set(bounds) == {'left', 'right'}
    assert all(.02499 < value < .02501 for value in bounds.values())


@pytest.mark.parametrize('bottom', [-.030, -.015, .024, .02799])
def test_no_contact_alone_does_not_prove_full_extraction(bottom):
    assert not corner_clearance(corners(bottom), [], .025)['valid']


@pytest.mark.parametrize('bottom', [.0281, .08, .10])
def test_all_corners_above_real_rim_and_no_contacts_pass(bottom):
    assert corner_clearance(corners(bottom), [], .025)['valid']


def test_even_zero_force_basket_contact_invalidates_free_hold():
    pair = dict(first='box48_left::body::collision', second='centrifuge::bucket_right::collision', force=0)
    assert not corner_clearance(corners(.08), [pair], .025)['valid']


def test_nonfinite_corners_fail_closed():
    sample = corners(.08)
    sample[0, 0] = np.nan
    assert not corner_clearance(sample, [], .025)['valid']


def test_hold_moves_after_original_full_lift_and_runs_continuous_clearance_gate():
    source = inspect.getsource(FullCycle.retrieve)
    assert "if phase == 'lift':" in source
    assert "if phase == 'pilot_lift':" not in source
    assert 'self.dwell(10)' in source and 'pilot_not_claimed_fully_extracted=True' in source
    assert 'clearance_metrics' in inspect.getsource(FullCycle.safety)
    assert 'Full extraction clearance or no-basket-contact lost' in inspect.getsource(FullCycle.safety)


def test_motion_planner_is_byte_identical_to_full_r195():
    current = Path(__file__).with_name('retrieval_worker.py')
    original = ROOT / 'runtime_evidence/centrifuge_48box_full_cycle_20261010_r195/retrieval_worker.py'
    assert current.read_bytes() == original.read_bytes()

