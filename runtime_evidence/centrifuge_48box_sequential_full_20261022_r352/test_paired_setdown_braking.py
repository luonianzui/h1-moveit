from copy import deepcopy

import pytest

from paired_setdown_braking import split_paired_setdown


def stage():
    return dict(phase='setdown', samples=[dict(torso_joint=0.)] +
                [dict(torso_joint=index / 1000) for index in range(121)],
                setdown_contract=dict(cartesian_samples=121, extra_initial_seed=True,
                                     vertical_span_by_hand_m=dict(left=.3, right=.3)))


def test_all_seed_and_target_points_and_original_last_ten_mm_are_preserved():
    original = stage()
    before = deepcopy(original)
    coarse, fine, split = split_paired_setdown(original)
    assert original == before
    assert split == 117 and len(coarse) == 118 and len(fine) == 5
    assert coarse[-1] == fine[0] == original['samples'][117]
    assert coarse[:-1] + fine == original['samples']
    assert coarse[0] == original['samples'][0]
    assert fine[-1] == original['samples'][-1]


@pytest.mark.parametrize('length', [0, 5, 121, 123])
def test_wrong_point_count_cannot_be_silently_trimmed(length):
    candidate = stage()
    candidate['samples'] = [dict(torso_joint=0.)] * length
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)


@pytest.mark.parametrize('field,value', [('cartesian_samples', 120), ('extra_initial_seed', False),
                                        ('vertical_span_by_hand_m', dict(left=.29, right=.3)),
                                        ('vertical_span_by_hand_m', dict(left=.3)),
                                        ('vertical_span_by_hand_m', dict(left=float('nan'), right=.3))])
def test_unsupported_cartesian_contract_is_rejected(field, value):
    candidate = stage()
    candidate['setdown_contract'][field] = value
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)


def test_unqualified_legacy_or_other_phase_is_rejected():
    candidate = stage()
    candidate.pop('setdown_contract')
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)
    candidate = stage()
    candidate['phase'] = 'transfer'
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)


@pytest.mark.parametrize('value', [float('inf'), float('nan')])
def test_nonfinite_command_is_rejected(value):
    candidate = stage()
    candidate['samples'][-1]['torso_joint'] = value
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)


def test_seed_discontinuity_and_joint_name_change_are_rejected():
    candidate = stage()
    candidate['samples'][0]['torso_joint'] = .001
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)
    candidate = stage()
    candidate['samples'][30] = dict(other_joint=0.)
    with pytest.raises(ValueError):
        split_paired_setdown(candidate)
