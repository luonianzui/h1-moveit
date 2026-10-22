"""Contract tests for the attempt11 post-navigation open matrix."""

import json
import math
from pathlib import Path

from h1_2_vision.candidate3_post_nav_open_matrix import full_audit


PACKAGE = Path(__file__).parents[1]


def passing_result():
    counts = [2, 3, 601, 601, 3, 2]
    return {
        'plan': {
            'contains_partial': False,
            'door_arc_target_count': 601,
            'execution_stage_count': 3,
            'reverse_audit_stage_count': 3,
            'stage_waypoint_counts': counts,
            'stages': [{} for _ in range(6)],
        },
        'certificate': {
            'valid': True,
            'status': 'PASS',
            'ik_complete': True,
            'collision_free': True,
            'clearance_passed': True,
            'minimum_environment_clearance_m': 0.010001,
            'maximum_adjacent_angular_change_rad': math.radians(4.8),
            'maximum_adjacent_gripper_change_m': 0.001,
            'forward_passed': True,
            'reverse_passed': True,
            'timing_passed': True,
            'torso_below_45_degrees': True,
            'maximum_absolute_torso_rad': math.radians(44.75),
            'no_partial': True,
            'audit_details': {
                'valid': True,
                'collision_free': True,
                'clearance_passed': True,
                'samples': sum(counts),
            },
        },
    }


def test_full_audit_requires_reverse_and_exact_sample_accounting():
    result = passing_result()
    assert full_audit(result)
    result['certificate']['reverse_passed'] = False
    assert not full_audit(result)
    result = passing_result()
    result['certificate']['audit_details']['samples'] -= 1
    assert not full_audit(result)


def test_characterization_uses_unmodified_attempt11_and_material_cases():
    value = json.loads((
        PACKAGE / 'config/candidate3_post_nav_open_characterization.json'
    ).read_text(encoding='utf-8'))
    assert value['baseline_cdr_sha256'] == {
        'epoch': (
            '98a0e2f9e4b030059721c29bc95ea373d3db65bcf67fdafe3a6f96bc84829c17'),
        'scene': (
            '78f41904253a7e30d3f04fd3e58dafb227a42c01b18d680d2e8fcdea73a9dca2'),
    }
    assert len(value['cases']) == 27
    assert any(case.get('base_x_mm') == 10.0 for case in value['cases'])
    assert any(case.get('base_yaw_deg') == -1.0 for case in value['cases'])

    admission = json.loads((
        PACKAGE / 'config/candidate3_post_nav_open_admission.json'
    ).read_text(encoding='utf-8'))
    assert admission['baseline_cdr_sha256'] == value['baseline_cdr_sha256']
    assert len(admission['cases']) == 37
    assert sum(case['expected'] == 'PASS'
               for case in admission['cases']) == 24
    assert sum(case['expected'] == 'REJECT'
               for case in admission['cases']) == 13


def test_matrix_worker_is_file_only_and_has_no_control_surface():
    source = (PACKAGE / 'h1_2_vision/candidate3_post_nav_open_matrix.py'
              ).read_text(encoding='utf-8')
    assert 'create_publisher' not in source
    assert 'ActionClient' not in source
    assert 'FollowJointTrajectory' not in source
    assert 'NavigateToPose' not in source
    assert 'subprocess.run' in source
