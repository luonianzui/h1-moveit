import copy
import struct

import numpy as np
import pytest

from cached_bucket import bucket_from_keys, pose_key
from cached_pose import matrix_from_bytes


def old_key(value):
    position = value.get('position', {})
    orientation = value.get('orientation', {})
    return struct.pack('7d', *(position.get(axis, 0) for axis in 'xyz'),
                       *(orientation.get(axis, 0) for axis in 'xyzw'))


def test_keys_identical_for_arbitrary_double_bits():
    generator = np.random.default_rng(338)
    raw = generator.bytes(7 * 8 * 10000)
    for offset in range(0, len(raw), 56):
        values = struct.unpack('7d', raw[offset:offset + 56])
        pose = dict(position=dict(zip('xyz', values[:3])), orientation=dict(zip('xyzw', values[3:])))
        assert pose_key(pose) == old_key(pose)


@pytest.mark.parametrize('pose', [{}, {'position': {}}, {'orientation': {'w': 1.}},
                                  {'position': {'x': -0.}, 'orientation': {'w': -1.}}])
def test_default_fields_and_signed_zero_identical(pose):
    assert pose_key(pose) == old_key(pose)


def test_bucket_cache_exact_matrix_bytes_and_bounded():
    generator = np.random.default_rng(339)
    for iteration in range(1000):
        first, second = struct.pack('7d', *generator.normal(size=7)), struct.pack('7d', *generator.normal(size=7))
        expected = matrix_from_bytes(first) @ matrix_from_bytes(second)
        actual = bucket_from_keys(first, second)
        assert actual.tobytes() == expected.tobytes()
        assert not actual.flags.writeable
    assert bucket_from_keys.cache_info().currsize == 512


def test_bucket_cache_changed_field_never_reuses_old_pose():
    first = dict(orientation=dict(w=1.))
    second = dict(position=dict(x=1.), orientation=dict(w=1.))
    before = bucket_from_keys(pose_key(first), pose_key(second)).tobytes()
    second['position']['x'] = np.nextafter(1., 2.)
    assert bucket_from_keys(pose_key(first), pose_key(second)).tobytes() != before


@pytest.mark.parametrize('bad', [0., float('nan'), float('inf')])
def test_bad_orientation_not_cached_as_valid(bad):
    pose = struct.pack('7d', 0., 0., 0., 0., 0., 0., bad)
    with pytest.raises(ValueError):
        bucket_from_keys(pose, pose)
