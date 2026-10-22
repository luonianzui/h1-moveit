import math


def split_paired_setdown(stage):
    contract = stage.get('setdown_contract', {})
    if stage.get('phase') != 'setdown' or contract.get('cartesian_samples') != 121 or contract.get('extra_initial_seed') is not True:
        raise ValueError('Expected paired 121-target setdown with one extra measured planning seed')
    spans = contract.get('vertical_span_by_hand_m', {})
    if set(spans) != {'left', 'right'} or any(not math.isfinite(value) or abs(value - .3) > 1e-9 for value in spans.values()):
        raise ValueError('Original paired 300mm vertical setdown must be preserved')
    samples = stage.get('samples', [])
    if len(samples) != 122:
        raise ValueError('Expected all 122 audited paired setdown points, including the seed')
    names = set(samples[0])
    if not names or any(set(sample) != names for sample in samples):
        raise ValueError('Paired setdown joint identities differ')
    if any(not math.isfinite(value) for sample in samples for value in sample.values()):
        raise ValueError('Nonfinite paired setdown joint value')
    if max(abs(samples[0][name] - samples[1][name]) for name in names) > .0003:
        raise ValueError('Extra seed does not agree with the first Cartesian target')
    split = 117
    coarse, fine = samples[:split + 1], samples[split:]
    assert coarse[:-1] + fine == samples
    assert len(fine) == 5 and .3 * (len(fine) - 1) / 120 == .01
    return coarse, fine, split
