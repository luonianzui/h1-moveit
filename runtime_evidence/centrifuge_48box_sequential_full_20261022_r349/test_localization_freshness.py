import math

import pytest

from localization_freshness import localization_freshness


def sample(**changes):
    values = dict(sim=64.18, source_sim=63.601, now=100.0, pose_wall=91.99, status_wall=91.99, status_valid=True)
    values.update(changes)
    return localization_freshness(**values)


def test_r171_slow_simulation_receipt_age_not_false_failure():
    assert sample()['valid']
    assert sample()['pose_age_wall_s'] > 8
    assert sample()['pose_age_sim_s'] < 1


@pytest.mark.parametrize('status_valid', [False, None, 'true', 1])
def test_invalid_status_never_exempted(status_valid):
    assert not sample(status_valid=status_valid)['valid']


@pytest.mark.parametrize('changes', [dict(source_sim=63.179), dict(source_sim=64.331),
                                  dict(source_sim=None), dict(pose_wall=39.99),
                                  dict(status_wall=39.99), dict(pose_wall=100.1),
                                  dict(status_wall=100.1)])
def test_stale_missing_ahead_and_wall_hang_fail_closed(changes):
    assert not sample(**changes)['valid']


@pytest.mark.parametrize('field', ['sim', 'source_sim', 'now', 'pose_wall', 'status_wall'])
def test_nonfinite_time_rejected(field):
    assert not sample(**{field:math.nan})['valid']

