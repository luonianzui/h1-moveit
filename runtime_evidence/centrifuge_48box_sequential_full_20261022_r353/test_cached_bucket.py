import copy
import json
from pathlib import Path

import numpy as np
import pytest

from cached_bucket import BASE_CORNERS, cached_bucket_corners, corners_from_keys, pose_key
from qualify_bucket_cache import original_function, verify_source


ROOT = Path(__file__).resolve().parent


@pytest.fixture
def poses():
    source = ROOT.parent / 'centrifuge_48box_sequential_runtime_perf_20261020_r1/actual_samples.json'
    return json.loads(source.read_text())['loaded_retention']['payload']['poses']


def test_only_declared_function_replaced_and_physics_unchanged():
    verify_source()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_actual_pose_exact_bytes_and_defensive_copy(poses, side):
    expected = original_function()(poses, side)
    result = cached_bucket_corners(poses, side)
    assert result.tobytes() == expected.tobytes()
    result[:] = 91
    assert cached_bucket_corners(poses, side).tobytes() == expected.tobytes()


def test_random_inputs_exact_original_bytes(poses):
    generator = np.random.default_rng(332)
    original = original_function()
    for iteration in range(5000):
        for name in ('centrifuge', 'bucket_left', 'box48_left'):
            values = generator.normal(size=7)
            poses[name] = dict(position=dict(zip('xyz', values[:3])),
                               orientation=dict(zip('xyzw', values[3:])))
        assert cached_bucket_corners(poses, 'left').tobytes() == original(poses, 'left').tobytes()
    assert corners_from_keys.cache_info().currsize == 512


@pytest.mark.parametrize('name', ['centrifuge', 'bucket_left', 'box48_left'])
@pytest.mark.parametrize('field,axis', [('position', 'x'), ('position', 'y'), ('position', 'z'),
                                      ('orientation', 'x'), ('orientation', 'y'),
                                      ('orientation', 'z'), ('orientation', 'w')])
def test_every_input_field_participates_without_rounding(poses, name, field, axis):
    before = copy.deepcopy(poses)
    value = poses[name].setdefault(field, {}).get(axis, 0)
    poses[name][field][axis] = float(np.nextafter(value, np.inf))
    assert pose_key(poses[name]) != pose_key(before[name])
    assert cached_bucket_corners(poses, 'left').tobytes() == original_function()(poses, 'left').tobytes()


def test_signed_zero_is_distinct_key(poses):
    poses['box48_left']['position']['x'] = 0.
    positive = pose_key(poses['box48_left'])
    poses['box48_left']['position']['x'] = -0.
    assert pose_key(poses['box48_left']) != positive


def test_internal_arrays_readonly(poses):
    result = corners_from_keys(pose_key(poses['box48_left']), pose_key(poses['centrifuge']),
                               pose_key(poses['bucket_left']))
    assert not result.flags.writeable and not BASE_CORNERS.flags.writeable
    with pytest.raises(ValueError):
        result[0, 0] = 0


def test_zero_quaternion_rejected_like_original(poses):
    poses['bucket_left']['orientation'] = dict.fromkeys('xyzw', 0.)
    for function in (cached_bucket_corners, original_function()):
        with pytest.raises(ValueError):
            function(poses, 'left')


def test_source_comparison_only_restores_exact_declared_function():
    from bucket_cache_source import CACHED_BUCKET_FUNCTION, ORIGINAL_BUCKET_FUNCTION
    from speed_source import restore_speed_source
    assert restore_speed_source('execute_loading.py', CACHED_BUCKET_FUNCTION) == ORIGINAL_BUCKET_FUNCTION
    changed = CACHED_BUCKET_FUNCTION.replace('poses, side)', 'poses, "left")', 1)
    assert restore_speed_source('execute_loading.py', changed) != ORIGINAL_BUCKET_FUNCTION
    assert restore_speed_source('unrelated.py', CACHED_BUCKET_FUNCTION) == CACHED_BUCKET_FUNCTION
    assert restore_speed_source('execute_loading.py', CACHED_BUCKET_FUNCTION + '\nother_changed=True') != ORIGINAL_BUCKET_FUNCTION
