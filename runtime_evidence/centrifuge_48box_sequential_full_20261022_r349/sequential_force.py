from fast_profile import RETRIEVAL_SOFT_GRASP_SECONDS
from soft_grasp import soft_grasp_profile


LOADING_FORCE_N = 16.
RETRIEVAL_FORCE_N = 16.5
UNCHANGED_OUTPUT_GUARD_N = 18.01


def retrieval_profile(initial, target):
    profile = soft_grasp_profile(initial, target, 0., LOADING_FORCE_N,
                                 duration=RETRIEVAL_SOFT_GRASP_SECONDS, intervals=60)
    for sample in profile:
        sample['effort'] = [value * RETRIEVAL_FORCE_N / LOADING_FORCE_N for value in sample['effort']]
    return profile
