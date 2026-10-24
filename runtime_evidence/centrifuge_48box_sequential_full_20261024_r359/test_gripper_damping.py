from pathlib import Path

import pytest

from gripper_damping import verify_damping_fork, verify_damping_output
from gripper_damping_cycle import GripperDampingCycle
from stop_handover_cycle import StopHandoverCycle


EVIDENCE = Path(__file__).resolve().parent


def test_exact_four_derivative_fields_only():
    candidate = (EVIDENCE / 'inputs/controllers_servo.yaml').read_bytes()
    original = candidate.replace(b'        d: 6.0\n', b'        d: 3.0\n')
    assert verify_damping_fork(original, candidate)
    for edited in (candidate.replace(b'p: 500.0', b'p: 501.0', 1),
                   candidate.replace(b'u_clamp_max: 2.0', b'u_clamp_max: 3.0', 1),
                   candidate.replace(b'd: 6.0', b'd: 6.5', 1)):
        with pytest.raises(ValueError):
            verify_damping_fork(original, edited)


@pytest.mark.parametrize('output,code', [('Double value is: 3.0', 0), ('Double value is: 6.0', 1),
                                       ('Integer value is: 3', 0), ('', 0)])
def test_wrong_or_unread_damping_refused(output, code):
    with pytest.raises(ValueError):
        verify_damping_output(output, code)


def test_actual_damping_readback_passes():
    assert verify_damping_output('Double value is: 6.0\n', 0) == 6.


def test_loaded_speed_cannot_precede_current_empty_approach():
    cycle = GripperDampingCycle.__new__(GripperDampingCycle)
    cycle.current_damping_empty_approach_completed = False
    with pytest.raises(ValueError):
        cycle.move([], 'paired_retrieval_transfer', .25)


def test_failed_native_approach_does_not_qualify_damping(monkeypatch):
    def failed_motion(self, samples, phase, rate):
        raise RuntimeError('Original native velocity fault')
    monkeypatch.setattr(StopHandoverCycle, 'move', failed_motion)
    cycle = GripperDampingCycle.__new__(GripperDampingCycle)
    cycle.current_damping_empty_approach_completed = False
    with pytest.raises(RuntimeError):
        cycle.move([], 'paired_load_approach', .8)
    assert cycle.current_damping_empty_approach_completed is False


def test_qualification_follows_successful_parent_move_and_does_not_certify_loading(monkeypatch):
    calls = []
    def motion(self, samples, phase, rate):
        calls.append('original_guarded_motion')
    monkeypatch.setattr(StopHandoverCycle, 'move', motion)
    cycle = GripperDampingCycle.__new__(GripperDampingCycle)
    cycle.current_damping_empty_approach_completed = False
    records = []
    cycle.record = lambda kind, payload: records.append((kind, payload))
    cycle.move([], 'paired_load_approach', .8)
    assert calls == ['original_guarded_motion'] and cycle.current_damping_empty_approach_completed
    assert records[0][1]['only_current_initial_path_completed_not_general_calibration']
    assert records[0][1]['loaded_motion_still_not_preclaimed']


def test_complete_program_and_original_native_velocity_guard_preserved():
    assert GripperDampingCycle.run_complete is StopHandoverCycle.run_complete
    source = (EVIDENCE / 'execute_loading.py').read_text()
    assert "raise RuntimeError('Effort gripper velocity fault:" in source
    assert 'velocity_violation(value)' in source
