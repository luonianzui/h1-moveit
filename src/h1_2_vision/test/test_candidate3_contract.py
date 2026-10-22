import json
import math
from pathlib import Path

from h1_2_vision.candidate3_contract import (
    ASSET_ORDER,
    canonical_hash,
    dense_interpolate,
    interpolate_transform,
    load_sealed_assets,
    select_knots,
    yaw_transform,
)

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
ASSET_ROOT = WORKSPACE.joinpath(
    'runtime_evidence', 'closed_loop_parallel_826_2_20260826',
    'asset_snapshot')


def test_sealed_826_2_assets_and_candidate3_cache_are_bound():
    assets, cache_hash = load_sealed_assets(ASSET_ROOT)
    assert tuple(value['name'] for value in assets) == ASSET_ORDER
    assert len(cache_hash) == 64
    assert all(value['payload_hash'] == canonical_hash(value['payload'])
               for value in assets)


def test_asset_payload_tampering_fails_closed(tmp_path):
    for source in ASSET_ROOT.iterdir():
        target = tmp_path / source.name
        target.write_bytes(source.read_bytes())
    path = tmp_path / ASSET_ORDER[0]
    document = json.loads(path.read_text())
    document['payload']['stage'] = 'candidate1'
    path.write_text(json.dumps(document))
    try:
        load_sealed_assets(tmp_path)
    except ValueError as error:
        assert 'hash' in str(error)
    else:
        raise AssertionError('tampered asset was accepted')


def test_knots_and_dense_interpolation_bound_joint_change():
    source = np.linspace(np.zeros(7), np.full(7, math.radians(20)), 81)
    knots = select_knots(source, math.radians(4))
    assert knots[0] == 0 and knots[-1] == 80
    dense = dense_interpolate(source[knots], math.radians(0.49))
    changes = np.abs(np.diff(np.asarray(dense), axis=0))
    assert float(changes.max()) <= math.radians(0.49) + 1e-12


def test_transform_interpolation_preserves_endpoints():
    first = yaw_transform([0.1, -0.2, 0.3], math.radians(-5))
    second = yaw_transform([0.4, 0.5, 0.6], math.radians(20))
    assert np.allclose(interpolate_transform(first, second, 0), first)
    assert np.allclose(interpolate_transform(first, second, 1), second)
    middle = interpolate_transform(first, second, 0.5)
    assert np.allclose(middle[:3, 3], [0.25, 0.15, 0.45])
