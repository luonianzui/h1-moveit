import pytest

from motion_clock_watch import SimulationProgressWatch, motion_wall_budget, original_wall_budget


def test_r244_measured_partial_pilot_is_not_rejected_only_for_host_clock_factor():
    duration = 4.8234708845974215
    watch = SimulationProgressWatch(duration, 41.528, 0.)
    for index in range(1, 242):
        wall = index * .5
        actual_sim = 41.528 + wall / 27.308674340230805
        watch.observe(actual_sim, wall)
    assert 120.59510588645935 > original_wall_budget(duration)
    assert watch.budget_wall_s == pytest.approx((duration + .45) * 50)


def test_fresh_repeated_clock_messages_do_not_hide_stalled_simulation():
    watch = SimulationProgressWatch(4., 10., 0.)
    watch.observe(10., 2.)
    with pytest.raises(TimeoutError, match='stopped advancing'):
        watch.observe(10., 2.001)


def test_continuously_advancing_simulation_still_has_a_finite_wall_cap():
    watch = SimulationProgressWatch(100., 10., 0.)
    assert watch.budget_wall_s == 900.
    watch.observe(11., 900.)
    with pytest.raises(TimeoutError, match='finite wall budget'):
        watch.observe(11.001, 900.001)


@pytest.mark.parametrize('source_sim, wall', [(9., 1.), (11., -1.), (float('nan'), 1.), (11., float('inf'))])
def test_invalid_actual_clock_fails_closed(source_sim, wall):
    watch = SimulationProgressWatch(4., 10., 0.)
    with pytest.raises(ValueError):
        watch.observe(source_sim, wall)


@pytest.mark.parametrize('duration', [0., -1., float('nan'), float('inf')])
def test_invalid_command_duration_cannot_disable_deadline(duration):
    with pytest.raises(ValueError):
        motion_wall_budget(duration)


def test_short_command_retains_original_sixty_wall_second_minimum():
    assert motion_wall_budget(.1) == original_wall_budget(.1) == 60.
