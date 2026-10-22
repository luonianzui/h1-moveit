import json
import math
from pathlib import Path

import numpy as np
import pytest

from loaded_retention import supported_corner_retention, retention_metrics


def corners(overhang=0, bottom=-.030003, top=.029497):
    return np.array([[-.04314, -.0624, bottom], [.045 + overhang, .0563, top]])


def contact(**changes):
    result = dict(first='box48_left::body::48box_outer_envelope',
                  second='centrifuge::bucket_left::bucket_left_collision', depth=.00003444, force=.33,
                  normals=[[1, 0, 0], [0, 0, 1]], force_vectors=[[.12, 0, 0], [0, 0, .33]])
    result.update(changes)
    return result


def metrics(points, pairs):
    return supported_corner_retention(points, pairs, 'left', np.eye(3))


@pytest.mark.parametrize('overhang', [0, .00000119685261, .00001295785449, .0005])
def test_contact_certified_solver_penetration(overhang):
    assert metrics(corners(overhang), [contact()])['valid']


@pytest.mark.parametrize('overhang', [.00100001, .01, .0011])
def test_escape_beyond_original_contact_limit_rejected(overhang):
    assert not metrics(corners(overhang), [contact()])['valid']


@pytest.mark.parametrize('changes', [dict(depth=.001001), dict(depth=math.nan), dict(force=math.nan),
                                  dict(force=0), dict(second='h1_2::left_dex1_finger_1::collision'),
                                  dict(second='centrifuge::bucket_right::bucket_right_collision')])
def test_support_and_contacts_fail_closed(changes):
    assert not metrics(corners(), [contact(**changes)])['valid']


@pytest.mark.parametrize('changes', [dict(normals=[[0, 0, 1]]), dict(force_vectors=[[math.nan, 0, 0]]),
                                  dict(normals=[]), dict(normals=[[math.nan, 0, 0]])])
def test_side_escape_requires_actual_finite_sidewall_contact(changes):
    assert not metrics(corners(.00001295785449), [contact(**changes)])['valid']


@pytest.mark.parametrize('bottom,top', [(-.031001, .029497), (-.030003, .030501)])
def test_no_overflow_and_original_bottom_contact_limit(bottom, top):
    assert not metrics(corners(bottom=bottom, top=top), [contact()])['valid']


def test_missing_support_rejected():
    assert not metrics(corners(), [])['valid']


def test_existing_one_mm_contact_bound_is_not_extended():
    assert not metrics(corners(.001001), [contact(depth=.001)])['valid']


def test_r163_actual_mesh_contact_certificate():
    fixture = json.loads(Path(__file__).with_name('inputs').joinpath('retention_fault_fixture_r163.json').read_text())
    result = retention_metrics(fixture['poses'], fixture['contacts'], fixture['payload'])
    assert result['valid'] and result['wall_contact'][0]
    assert .0000129 < result['side_overhang_m'] < .0000131
    assert .0000344 < result['maximum_contact_depth_m'] < .0000345


def test_zero_sidewall_reaction_does_not_erase_contact_manifold():
    assert metrics(corners(.00000725580415), [contact(force_vectors=[[0, 0, 0], [0, 0, .33]])])['valid']


def test_r165_actual_zero_reaction_wall_contacts():
    fixture = json.loads(Path(__file__).with_name('inputs').joinpath('retention_fault_fixture_r165.json').read_text())
    result = retention_metrics(fixture['poses'], fixture['contacts'], fixture['payload'])
    assert result['valid'] and result['wall_contact'][0]
    assert .0000072 < result['side_overhang_m'] < .0000073


def test_bottom_solver_penetration_requires_actual_floor_contact():
    assert metrics(corners(bottom=-.030503), [contact(depth=.000503)])['valid']
    assert not metrics(corners(bottom=-.030503), [contact(depth=.000503, normals=[[1, 0, 0]])])['valid']


def test_r166_actual_bottom_contact_within_original_depth_gate():
    fixture = json.loads(Path(__file__).with_name('inputs').joinpath('retention_fault_fixture_r166.json').read_text())
    result = retention_metrics(fixture['poses'], fixture['contacts'], fixture['payload'])
    assert result['valid'] and result['floor_contact']
    assert .000501 < result['bottom_penetration_m'] < .000503

