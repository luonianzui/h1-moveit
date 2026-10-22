import ast
import copy
from pathlib import Path

import pytest

from audit_speed_paths import verify_axis_exit_timing, verify_published
from dual_motion import TASK_JOINTS
from speed_policy import common_timing, motion_limits


def command(phase, loaded):
    actual = dict.fromkeys(TASK_JOINTS, 0.)
    samples = [dict(actual), dict(actual, left_elbow_joint=.1)]
    rate, unused = motion_limits(phase, loaded)
    timing = common_timing(actual, samples, rate, ramp_s=.3 if phase == 'paired_retrieval_setdown' else .15)
    return dict(phase=phase, payload=dict(initial=actual, samples=samples, actual_rate_rad_s=rate,
        times=timing['times'], groups=timing['groups'],
        holders={'left': dict(payload='left')} if loaded else {}))


@pytest.mark.parametrize('phase,loaded', [
    ('seq_load_empty_transfer', False), ('seq_load_lower', True),
    ('seq_retrieval_retrieve_axis_exit', True), ('paired_retrieval_setdown', True),
    ('paired_retrieval_setdown_fine', True)])
def test_independent_all_phase_timing_checks(phase, loaded):
    actual = command(phase, loaded)
    verify_axis_exit_timing(actual)
    wrong = copy.deepcopy(actual)
    wrong['payload']['actual_rate_rad_s'] *= .5
    with pytest.raises(ValueError):
        verify_axis_exit_timing(wrong)
    wrong = copy.deepcopy(actual)
    wrong['payload']['times'][-1] += .001
    with pytest.raises(ValueError):
        verify_axis_exit_timing(wrong)


def test_independent_published_path_and_single_holder_remain_required():
    actual = command('seq_load_empty_transfer', False)
    planned = dict(mode='load_pickup', stages=[dict(phase='empty_transfer', samples=actual['payload']['samples'])])
    verify_published([actual], planned)
    wrong = copy.deepcopy(actual)
    wrong['payload']['samples'][-1]['left_elbow_joint'] += .001
    with pytest.raises(ValueError):
        verify_published([wrong], planned)
    actual['payload']['holders'] = dict(left={}, right={})
    with pytest.raises(ValueError):
        verify_published([actual], planned)


def test_all_new_sources_parse_and_physical_completion_is_not_duration_only():
    root = Path(__file__).resolve().parent
    for name in ('audit_speed_cycle.py', 'audit_speed_grip.py', 'audit_speed_physical.py',
                 'audit_speed_paths.py', 'sequential_speed_cycle.py'):
        ast.parse((root / name).read_text())
    text = (root / 'audit_speed_physical.py').read_text()
    for marker in ("executor=result['status'] == 'PASS'", "['lid', 'lid', 'rotor', 'rotor', 'lid']",
                   "steady_run_sim_seconds'] <= 32.4", "containment_faults == 0",
                   "'distinct_return_slots'", "terminal_hold_relaxed_one_second"):
        assert marker in text
