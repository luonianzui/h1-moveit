from pathlib import Path

import pytest

from fast_profile import RETRIEVAL_SOFT_GRASP_SECONDS
from sequential_force import LOADING_FORCE_N, RETRIEVAL_FORCE_N, UNCHANGED_OUTPUT_GUARD_N, retrieval_profile
from soft_grasp import soft_grasp_profile


@pytest.mark.parametrize('initial', [[-.0199, -.0199], [-.01989, -.01991]])
def test_pid_headroom_preserves_original_position_velocity_and_time(initial):
    old = soft_grasp_profile(initial, -.01901, 0., 16.,
                             duration=RETRIEVAL_SOFT_GRASP_SECONDS, intervals=60)
    new = retrieval_profile(initial, -.01901)
    assert len(new) == len(old)
    for parent, candidate in zip(old, new):
        assert {key: value for key, value in candidate.items() if key != 'effort'} == {
            key: value for key, value in parent.items() if key != 'effort'}
        assert candidate['effort'] == [value * 16.5 / 16. for value in parent['effort']]
    assert new[-1]['effort'] == [16.5, 16.5]
    assert LOADING_FORCE_N == 16. and RETRIEVAL_FORCE_N == 16.5
    assert UNCHANGED_OUTPUT_GUARD_N == 18.01


def test_original_software_output_guard_still_active():
    source = (Path(__file__).parent / 'execute_loading.py').read_text()
    assert 'abs(value) > 18.01' in source
    assert "raise RuntimeError('Effort experiment output exceeds 18 N')" in source
