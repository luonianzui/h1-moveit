"""Offline tests for the result-only middle admission boundary."""

import hashlib
import json
import os
from pathlib import Path

from h1_2_vision import candidate3_middle_admission as admission

import pytest


SOURCE = (Path(__file__).resolve().parents[1] / 'h1_2_vision' /
          'candidate3_middle_admission.py')


def _sealed(path, value):
    path.write_text(
        json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')
    path.chmod(0o400)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _plan():
    return {
        'schema': 'test-plan',
        'phase_names': list(admission.PHASE_NAMES),
        'release_owner': admission.RELEASE_OWNER,
        'terminal_seam': {
            'return_e0': {
                'torso_yaw_rad': admission.E0_TORSO_RAD,
                'right_arm_rad': list(admission.E0_RIGHT_ARM_RAD),
                'right_fingers_m': dict(admission.E0_RIGHT_FINGERS),
            },
            'release_owner': admission.RELEASE_OWNER,
            'forbidden_middle_phases': ['right_release', 'right_retreat'],
            'valid': True,
        },
    }


def test_admission_calls_validate_result_exactly_once(tmp_path, monkeypatch):
    """Admission delegates semantic validation exactly once."""
    plan = _plan()
    path = tmp_path / 'result.json'
    digest = _sealed(path, {'schema': 'test', 'plan': plan})
    calls = []

    def validator(value):
        calls.append(value)
        return value['plan']

    monkeypatch.setattr(admission, 'validate_result', validator)
    assert admission.admit_result(path, digest) == plan
    assert len(calls) == 1


@pytest.mark.parametrize(('fault', 'status'), [
    ('mode', 'RESULT_MODE'),
    ('sha', 'RESULT_SHA256'),
    ('nlink', 'RESULT_NLINK'),
])
def test_file_gates_fail_before_semantic_validator(tmp_path, monkeypatch,
                                                   fault, status):
    """File identity failures precede semantic validation."""
    path = tmp_path / 'result.json'
    digest = _sealed(path, {'schema': 'test'})
    if fault == 'mode':
        path.chmod(0o600)
    elif fault == 'sha':
        digest = '0' * 64
    else:
        os.link(path, tmp_path / 'alias.json')
    monkeypatch.setattr(
        admission, 'validate_result',
        lambda _value: pytest.fail('semantic validator was reached'))
    with pytest.raises(admission.MiddleAdmissionError, match=status):
        admission.admit_result(path, digest)


def test_duplicate_key_and_nonfinite_json_fail_closed(tmp_path, monkeypatch):
    """Strict JSON rejects duplicate keys and nonfinite values."""
    monkeypatch.setattr(
        admission, 'validate_result',
        lambda _value: pytest.fail('semantic validator was reached'))
    for index, raw in enumerate((b'{"a":1,"a":2}\n', b'{"a":NaN}\n')):
        path = tmp_path / f'bad_{index}.json'
        path.write_bytes(raw)
        path.chmod(0o400)
        digest = hashlib.sha256(raw).hexdigest()
        with pytest.raises(admission.MiddleAdmissionError):
            admission.admit_result(path, digest)


@pytest.mark.parametrize(('mutation', 'status'), [
    (lambda plan: plan.__setitem__('phase_names', ['wrong']),
     'MIDDLE_PHASE_NAMES'),
    (lambda plan: plan['terminal_seam'].__setitem__('valid', False),
     'MIDDLE_TERMINAL_SEAM_VALID'),
    (lambda plan: plan.__setitem__('right_release', True),
     'MIDDLE_FORBIDDEN_RIGHT_RELEASE_RETREAT'),
])
def test_supplemental_topology_and_e0_gates_fail_closed(
        tmp_path, monkeypatch, mutation, status):
    """Supplemental topology and canonical E0 gates fail closed."""
    plan = _plan()
    mutation(plan)
    path = tmp_path / 'result.json'
    digest = _sealed(path, {'schema': 'test', 'plan': plan})
    monkeypatch.setattr(
        admission, 'validate_result', lambda value: value['plan'])
    with pytest.raises(admission.MiddleAdmissionError, match=status):
        admission.admit_result(path, digest)


def test_contract_rejection_is_wrapped_and_cli_exits_nonzero(
        tmp_path, monkeypatch, capsys):
    """The CLI wraps the production contract error and fails."""
    path = tmp_path / 'result.json'
    digest = _sealed(path, {'schema': 'test'})
    monkeypatch.setattr(
        admission, 'validate_result',
        lambda _value: (_ for _ in ()).throw(
            ValueError('CERTIFICATE_INVALID')))
    assert admission.main([
        '--result', str(path), '--result-sha256', digest,
    ]) == 1
    assert 'RESULT_CONTRACT:CERTIFICATE_INVALID' in capsys.readouterr().err


def test_source_is_ros_free_and_has_single_semantic_authority():
    """The admission boundary stays ROS-free and single-authority."""
    source = SOURCE.read_text(encoding='utf-8')
    assert 'import rclpy' not in source
    assert 'trajectory_msgs' not in source
    assert 'create_publisher' not in source
    assert 'make_certificate' not in source
    assert source.count('validate_result(result)') == 1
