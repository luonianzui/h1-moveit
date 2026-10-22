import hashlib
import json
import math
from pathlib import Path
import re


POSITION_GAIN = .6
CALIBRATION_NAME = 'centrifuge_48box_gain_reverse_calibration_20261013_r253'
CALIBRATION_MANIFEST = 'c7c85cf28e5cfd4045d1f8b6175ec29a7e6f4896d51edd162a8a963004674fda'
CALIBRATION_REPORT = '9d52f57bbb938ddd1ff464dad6b9d77174b2f4c420573e897ee50c0d160454cd'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify_gain_fork(original, candidate):
    old = b'<position_proportional_gain>0.2</position_proportional_gain>'
    new = b'<position_proportional_gain>0.6</position_proportional_gain>'
    if original.count(old) != 1 or candidate.count(new) != 1 or candidate.replace(new, old) != original:
        raise ValueError('Only one exact position gain change is authorized')
    return True


def verify_gain_output(output, returncode):
    if returncode != 0 or re.fullmatch(r'\s*Double value is: 0\.6\s*', output) is None:
        raise ValueError('Fresh Gazebo position gain must be exactly 0.6')
    return POSITION_GAIN


def validate_calibration(certificate):
    if (certificate.get('status') != 'PASS' or certificate.get('position_proportional_gain') != POSITION_GAIN or
            certificate.get('scope') != 'fixed_pelvis_empty_arm_gain06_500hz_calibration' or
            certificate.get('full_cycle_execution_certified') is not False or
            certificate.get('independent_native_frames_and_scalar_interpolation') is not True or
            certificate.get('reported_tracking_peaks_not_used_as_proof') is not True or
            certificate.get('medium_rate_two_mrad_passed') is not True):
        raise ValueError('Missing independent gain-specific empty-arm calibration')
    expected = {f'machine_gain_calibration_{rate}_{direction}'
                for rate in (.12, .4, .8) for direction in ('outward', 'return')}
    peaks, counts = certificate['native_tracking_peaks_rad'], certificate['native_tracking_counts']
    if set(peaks) != expected or set(counts) != expected:
        raise ValueError('Incomplete reciprocal calibration')
    for phase in expected:
        limit = .002 if '_0.4_' in phase else .015
        if not math.isfinite(peaks[phase]) or not 0 <= peaks[phase] <= limit or counts[phase] <= 0:
            raise ValueError('Original native tracking gate not passed')
    velocity = certificate['maximum_actual_finger_velocity_m_s']
    if not math.isfinite(velocity) or not 0 <= velocity <= .2:
        raise ValueError('Original actual finger velocity gate not passed')
    return True


def calibration_proof(evidence):
    evidence = Path(evidence)
    parent = evidence.parent / CALIBRATION_NAME
    assert digest(parent / 'SHA256SUMS') == CALIBRATION_MANIFEST
    assert digest(parent / 'REPORT.md') == CALIBRATION_REPORT
    rows = (parent / 'SHA256SUMS').read_text().splitlines()
    assert len(rows) == 403
    for line in rows:
        expected, relative = line.split(maxsplit=1)
        path = parent / relative.lstrip('*')
        assert digest(path) == expected and path.stat().st_mode & 0o777 == 0o444, relative
    certificate = json.loads((parent / 'gain_calibration_audit.json').read_text())
    validate_calibration(certificate)
    identity = json.loads((parent / 'identity.json').read_text())
    assert identity['position_proportional_gain'] == POSITION_GAIN and identity['manager_update_rate_hz'] == 500
    assert identity['full_cycle_execution_certified'] is False
    assert json.loads((parent / 'trace_audit.json').read_text())['status'] == 'PASS'
    readback = json.loads((parent / 'run/gain_parameter_readback1.json').read_text())
    assert readback['read_only_no_parameter_set'] is True and len(readback['attempts']) == 1
    parameter = readback['attempts'][0]
    verify_gain_output(parameter['stdout'], parameter['returncode'])
    records = []
    for name in ('robot_servo.urdf', 'robot_dual.urdf'):
        expected = (parent / 'inputs' / name).read_text().replace(str(parent), str(evidence))
        expected = expected.replace('install_lab_scene_48box_gain_reverse_calibration_20261013_r253',
                                    'install_lab_scene_48box_sequential_full_20261022_r353')
        expected = expected.replace('centrifuge_48box_gain_reverse_calibration_r253',
                                    'centrifuge_48box_sequential_full_r349').encode()
        actual = (evidence / 'inputs' / name).read_bytes()
        if name == 'robot_dual.urdf':
            from acceleration_inputs import verify_acceleration_input
            verify_acceleration_input(expected, actual, name, evidence.parents[1] / 'src/h1_2_description')
        else:
            assert actual == expected or (not actual.endswith(b'\n') and actual + b'\n' == expected), name
        records.append(dict(name=name, calibrated_model_sha256=digest(parent / 'inputs' / name),
                            current_sha256=digest(evidence / 'inputs' / name)))
    return dict(status='PASS', calibration_parent=str(parent), calibration_manifest_sha256=CALIBRATION_MANIFEST,
                calibration_report_sha256=CALIBRATION_REPORT, position_proportional_gain=POSITION_GAIN,
                manager_update_rate_hz=500, models=records, certificate=certificate,
                only_small_empty_arm_ramps_certified=True, loaded_motion_not_preclaimed=True,
                calibration_shape_changed_only_at_feet=True, new_scene_tracking_still_requires_actual_verification=True,
                full_cycle_execution_certified=False, fresh_complete_physical_execution_required=True,
                prior_certificate_gripper_control_not_current_dynamics=True,
                unchanged_arm_driver_parameters_not_current_gripper_velocity_certificate=True,
                current_gripper_pid_d=6., current_initial_approach_under_native_guards_required=True)


if __name__ == '__main__':
    evidence = Path(__file__).resolve().parent
    proof = calibration_proof(evidence)
    with (evidence / 'calibrated_gain_audit.json').open('x') as stream:
        json.dump(proof, stream, indent=2)
    print('GAIN_PREREQUISITE_PASS_NOT_FULL_CYCLE', flush=True)
