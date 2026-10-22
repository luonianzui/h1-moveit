from copy import deepcopy

import pytest

from audit_coherent import verify_bracket


def sample():
    return dict(lower_stamp=1., upper_stamp=1.01, fraction=.5, positions={'joint':.005}),


def test_source_bracket_is_recomputed_not_trusted_from_report():
    pair = sample()[0]
    assert verify_bracket(pair, 1.005, {1.:{'joint':0.}, 1.01:{'joint':.01}}) == pytest.approx({'joint':.005})


@pytest.mark.parametrize('change', ['missing', 'gap', 'fraction', 'positions', 'future'])
def test_commanded_or_modified_or_unbracketed_state_cannot_pass(change):
    pair = deepcopy(sample()[0])
    frames = {1.:{'joint':0.}, 1.01:{'joint':.01}}
    stamp = 1.005
    if change == 'missing':
        del frames[1.01]
    elif change == 'gap':
        pair['upper_stamp'] = 1.02
        frames[1.02] = {'joint':.02}
    elif change == 'fraction':
        pair['fraction'] = .6
    elif change == 'positions':
        pair['positions']['joint'] = .006
    else:
        stamp = 1.011
    with pytest.raises((AssertionError, KeyError)):
        verify_bracket(pair, stamp, frames)
