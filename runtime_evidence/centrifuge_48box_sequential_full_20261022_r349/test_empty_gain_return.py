from pathlib import Path

import pytest

from contact_phase_policy import bounded_motion
from dual_motion import common_arm_timing, TASK_JOINTS


@pytest.mark.parametrize('requested,expected', [(.8, .4), (.4, .4), (.3, .3)])
def test_only_failing_empty_return_has_calibrated_rate_and_original_precision(requested, expected):
    assert bounded_motion('machine_paired_empty_return', requested) == (expected, .002)


@pytest.mark.parametrize('phase', ['paired_load_approach', 'machine_paired_empty_reverse_forward_above_housing_outboard',
                                 'paired_load_forward_table_endpoint_descent', 'paired_retrieval_transfer'])
def test_other_empty_and_loaded_phases_are_not_slowed_or_relaxed(phase):
    assert bounded_motion(phase, .25) == (.25, .015)


def test_retiming_keeps_all_points_groups_and_twenty_ms_floor():
    initial = dict.fromkeys(TASK_JOINTS, 0.)
    samples = [dict(initial, right_elbow_joint=value) for value in (0., .16, .28, .40)]
    original = common_arm_timing(initial, samples, .8)
    candidate = common_arm_timing(initial, samples, bounded_motion('machine_paired_empty_return', .8)[0])
    assert candidate['points'] == original['points'] and candidate['groups'] == original['groups']
    assert candidate['duration_sim_s'] > original['duration_sim_s']
    assert all(later - earlier >= .020 - 1e-12
               for earlier, later in zip(candidate['times'], candidate['times'][1:]))


def test_actual_native_velocity_guard_and_full_actor_remain_required():
    root = Path(__file__).resolve().parent
    source = (root / 'execute_loading.py').read_text()
    assert "raise RuntimeError('Effort gripper velocity fault:" in source
    assert 'velocity_violation(value)' in source
    assert 'cycle = DualCycle(plan, buttons)' in (root / 'execute_cycle.py').read_text()
    assert 'steady_rotation_sim_s=30' in (root / 'seal_identity.py').read_text()
