import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cached_pose import cached_pose_matrix, matrix_from_bytes
from pose_cache_source import ORIGINAL_POSE_FUNCTION


scope = dict(np=np, Rotation=Rotation)
exec(ORIGINAL_POSE_FUNCTION, scope)
original = scope['pose_matrix']


def test_random_quaternions_and_positions_exact_original_bytes():
    rng = np.random.default_rng(330)
    for number in range(5000):
        position, orientation = rng.normal(size=3), rng.normal(size=4)
        pose = dict(position=dict(zip('xyz', position)), orientation=dict(zip('xyzw', orientation)))
        assert cached_pose_matrix(pose).tobytes() == original(pose).tobytes()
        assert cached_pose_matrix(pose).tobytes() == original(pose).tobytes()
    assert matrix_from_bytes.cache_info().currsize <= 512


@pytest.mark.parametrize('position', [0., -0., 1., 1e-12])
@pytest.mark.parametrize('sign', [1., -1.])
def test_signed_zero_and_missing_default_axes(position, sign):
    pose = dict(position=dict(x=position), orientation=dict(w=sign))
    assert cached_pose_matrix(pose).tobytes() == original(pose).tobytes()


@pytest.mark.parametrize('pose', [
    {}, dict(orientation=dict(w=0.)), dict(orientation=dict(w=float('nan'))),
    dict(position=dict(x=float('inf')), orientation=dict(w=1.)),
    dict(orientation=dict(x=float('inf'), w=1.)),
])
def test_invalid_poses_do_not_become_cached_valid_measurements(pose):
    with pytest.raises(ValueError):
        original(pose)
    with pytest.raises(ValueError):
        cached_pose_matrix(pose)


def test_input_and_output_mutations_cannot_poison_cache():
    pose = dict(position=dict(x=1.), orientation=dict(w=1.))
    result = cached_pose_matrix(pose)
    result[0, 3] = 99.
    assert cached_pose_matrix(pose)[0, 3] == 1.
    pose['position']['x'] = 2.
    assert cached_pose_matrix(pose)[0, 3] == 2.
    assert result.flags.writeable


def test_runtime_wrapper_and_empty_button_cap():
    from execute_loading import pose_matrix
    from sequential_speed_cycle import SpeedPriorityCycle
    from compression_cycle import CompressionCycle
    from types import SimpleNamespace
    pose = dict(orientation=dict(w=1.))
    assert pose_matrix(pose).tobytes() == original(pose).tobytes()
    cycle = object.__new__(SpeedPriorityCycle)
    cycle.holders = SimpleNamespace(rows={})
    cycle.current_damping_empty_approach_completed = True
    assert CompressionCycle.button_noncontact_rate(cycle) == 1.2
    assert cycle.button_noncontact_rate() == .8
    cycle.holders.rows['left'] = 'left'
    with pytest.raises(ValueError):
        cycle.button_noncontact_rate()
