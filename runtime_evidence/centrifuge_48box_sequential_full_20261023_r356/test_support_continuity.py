from support_continuity import SupportContinuity


def sample(**changes):
    result = dict(supported=True, contacts_valid=True, sidewall_certified=True, vertical_inside=True)
    result.update(changes)
    return result


def test_no_prior_positive_support():
    assert not SupportContinuity(.02).update(1, sample(supported=False))['valid']


def test_brief_zero_reaction_requires_contact_and_geometry():
    guard = SupportContinuity(.02)
    assert guard.update(1, sample())['valid']
    assert guard.update(1.019, sample(supported=False))['valid']
    assert not guard.update(1.021, sample(supported=False))['valid']


def test_positive_force_resets_gap():
    guard = SupportContinuity(.02)
    guard.update(1, sample())
    guard.update(1.019, sample(supported=False))
    assert guard.update(1.020, sample())['valid']
    assert guard.update(1.039, sample(supported=False))['valid']


def test_geometry_and_foreign_contact_never_exempted():
    for field in ('contacts_valid', 'sidewall_certified', 'vertical_inside'):
        guard = SupportContinuity(.02)
        guard.update(1, sample())
        assert not guard.update(1.001, sample(supported=False, **{field:False}))['valid']


def test_reversed_sim_time_rejected():
    guard = SupportContinuity(.02)
    guard.update(1, sample())
    assert not guard.update(.99, sample(supported=False))['valid']


def test_reversed_positive_sample_rejected():
    guard = SupportContinuity(.02)
    guard.update(1, sample())
    assert not guard.update(.99, sample())['valid']

