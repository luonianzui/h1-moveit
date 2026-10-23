import numpy as np
import pytest

from command_sampling import dense_segments, duration, reduce_polyline


def path(values):
    return [dict(first=float(first), second=float(second)) for first, second in values]


def test_straight_geometry_endpoint_and_rate_preserved_without_201_control_points():
    samples = path(np.linspace([0, 0], [1, -.4], 201))
    result = reduce_polyline(samples)
    assert result['retained_points'] == 2
    assert result['samples'] == [samples[0], samples[-1]]
    assert result['maximum_joint_deviation_rad'] < 1e-14
    assert duration(result['samples'], .8) < duration(samples, .8) - 2
    assert result['dense_collision_reaudit_still_required']


def test_actual_turns_and_backtracking_cannot_be_shortcut_into_a_straight_line():
    samples = path([[0, 0], [1, 0], [1, .5], [.5, .5], [.5, 0]])
    assert reduce_polyline(samples)['samples'] == samples


def test_curved_geometry_approximation_is_bounded_in_every_joint():
    values = np.linspace(0, 1, 201)
    samples = path(np.c_[values, .02 * np.sin(values * np.pi)])
    result = reduce_polyline(samples)
    assert 2 < result['retained_points'] < 201
    assert result['maximum_joint_deviation_rad'] <= .000025
    assert result['samples'][0] == samples[0] and result['samples'][-1] == samples[-1]


def test_repeated_waypoints_do_not_mutate_input():
    samples = path([[0, 0]] * 20)
    result = reduce_polyline(samples)
    assert result['retained_points'] == 2 and len(samples) == 20


@pytest.mark.parametrize('samples,tolerance', [([], .01), ([{}], .01),
    ([{'joint':0}, {'other':0}], .01), ([{'joint':float('nan')}], .01),
    ([{'joint':0}], float('nan')), ([{'joint':0}], -.01)])
def test_unknown_nonfinite_or_unbounded_inputs_are_rejected(samples, tolerance):
    with pytest.raises(ValueError):
        reduce_polyline(samples, tolerance)


def test_dense_collision_points_keep_2mrad_bound_even_when_controller_has_two_points():
    samples = path([[0, 0], [.137, -.098]])
    dense = list(dense_segments(samples))
    assert len(dense) > 60 and dense[0] == samples[0] and dense[-1] == samples[-1]
    for previous, current in zip(dense, dense[1:]):
        assert max(abs(current[name] - previous[name]) for name in current) <= .002 + 1e-14


@pytest.mark.parametrize('rate,floor', [(0, .02), (float('inf'), .02), (.8, .019)])
def test_retiming_cannot_remove_20ms_control_floor(rate, floor):
    with pytest.raises(ValueError):
        duration(path([[0, 0], [.1, 0]]), rate, floor)
