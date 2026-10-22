import copy
import json
from pathlib import Path

import pytest

from gain_contract import validate_calibration, verify_gain_fork, verify_gain_output


EVIDENCE = Path(__file__).resolve().parent
CALIBRATION = EVIDENCE.parent / 'centrifuge_48box_gain_reverse_calibration_20261013_r253'


def test_exact_single_gain_fork_preserves_every_other_byte():
    original = b'<robot><position_proportional_gain>0.2</position_proportional_gain><mass>1</mass></robot>'
    candidate = original.replace(b'>0.2<', b'>0.6<')
    assert verify_gain_fork(original, candidate)
    with pytest.raises(ValueError):
        verify_gain_fork(original, candidate.replace(b'>1<', b'>2<'))
    with pytest.raises(ValueError):
        verify_gain_fork(original, candidate + b'\n')


@pytest.mark.parametrize('output,code', [('Double value is: 0.2', 0), ('Double value is: 0.6', 1),
                                       ('Integer value is: 0.6', 0), ('', 0)])
def test_wrong_or_failed_actual_gain_readback_is_refused(output, code):
    with pytest.raises(ValueError):
        verify_gain_output(output, code)


def test_exact_actual_gain_readback():
    assert verify_gain_output('Double value is: 0.6\n', 0) == .6


def test_original_native_gate_and_all_six_calibration_legs_required():
    certificate = json.loads((CALIBRATION / 'gain_calibration_audit.json').read_text())
    assert validate_calibration(certificate)
    bad = copy.deepcopy(certificate)
    bad['native_tracking_peaks_rad']['machine_gain_calibration_0.4_return'] = .0021
    with pytest.raises(ValueError):
        validate_calibration(bad)
    bad = copy.deepcopy(certificate)
    bad['native_tracking_counts']['machine_gain_calibration_0.4_outward'] = 0
    with pytest.raises(ValueError):
        validate_calibration(bad)


def test_empty_calibration_cannot_be_relabelled_as_full_cycle():
    certificate = json.loads((CALIBRATION / 'gain_calibration_audit.json').read_text())
    certificate['full_cycle_execution_certified'] = True
    with pytest.raises(ValueError):
        validate_calibration(certificate)


def test_full_actor_and_original_loaded_contract_not_replaced_by_calibrator():
    executor = (EVIDENCE / 'execute_cycle.py').read_text()
    seal = (EVIDENCE / 'seal_identity.py').read_text()
    assert 'cycle = DualCycle(plan, buttons)' in executor
    assert 'ServoGainCalibration' not in executor and "cycle.run_complete()" in executor
    assert "steady_rotation_sim_s=30" in seal and 'actual_two_holder_registry_required=True' in seal
    assert "scope='two_box_load_close_spin_open_retrieve'" in seal
    assert 'full_cycle_execution_required=True' in seal
    assert 'gain_specific_calibration_required' in (EVIDENCE / 'audit_inherited_inputs.py').read_text()
