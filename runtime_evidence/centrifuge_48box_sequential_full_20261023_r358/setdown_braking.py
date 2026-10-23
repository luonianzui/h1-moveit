def braking_setdown_segments(samples):
    if len(samples) != 121:
        raise ValueError('Expected the original 121-point 300mm setdown path')
    split = len(samples) - 5
    coarse = samples[:split + 1]
    fine = samples[split:]
    assert coarse[:-1] + fine == samples
    return coarse, fine, split

