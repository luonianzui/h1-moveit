import math
from pathlib import Path

from h1_2_vision.candidate3_worker import (
    rotation_vector,
    simplify_collinear_positions,
    target_delta,
)
import numpy as np


SOURCE = (Path(__file__).parents[1] / 'h1_2_vision' /
          'candidate3_worker.py').read_text(encoding='utf-8')


def test_timing_simplifier_removes_only_collinear_samples():
    values = np.asarray([
        [0.0, 0.0],
        [0.25, 0.50],
        [0.50, 1.00],
        [0.75, 0.50],
        [1.00, 0.00],
    ])
    simplified = simplify_collinear_positions(values)
    assert np.array_equal(simplified, values[[0, 2, 4]])


def test_timing_simplifier_discards_only_microradian_ik_noise():
    values = np.asarray([
        [0.0, 0.0],
        [0.5, 0.5000004],
        [1.0, 1.0],
    ])
    assert np.array_equal(
        simplify_collinear_positions(values), values[[0, 2]])
    material = values.copy()
    material[1, 1] += 2.0e-4
    assert np.array_equal(simplify_collinear_positions(material), material)


def test_rotation_vector_is_axis_angle_in_world_coordinates():
    rotation = np.eye(4)
    angle = math.radians(12.5)
    rotation[:3, :3] = [
        [math.cos(angle), -math.sin(angle), 0],
        [math.sin(angle), math.cos(angle), 0],
        [0, 0, 1],
    ]
    assert np.allclose(rotation_vector(rotation), [0, 0, angle], atol=1e-12)


def test_worker_delegates_strict_world_audit_to_cpp_executable():
    assert "parser.add_argument('--clearance-auditor', required=True)" in SOURCE
    assert "'h1_candidate3_cpp_clearance_audit/v1'" in SOURCE
    assert 'subprocess.run(' in SOURCE
    assert 'allow_all_robot_self_pairs' not in SOURCE
    assert 'clearance_audited_trajectory_hash' in SOURCE


def test_cpp_auditor_reports_the_first_collision_pair():
    auditor = (Path(__file__).parents[2] / 'h1_2_moveit_demo' / 'src' /
               'candidate3_clearance_auditor.cpp').read_text(encoding='utf-8')
    assert 'self_request.contacts = true;' in auditor
    assert 'self_result.contacts.begin()->first.first' in auditor
    assert 'world_result.contacts.begin()->first.first' in auditor


def test_worker_preserves_dense_ik_seed_continuity_across_messages():
    assert 'self.previous_by_group = {}' in SOURCE
    assert 'self.previous_source_by_group = {}' in SOURCE
    assert 'self.previous_by_group.get(group)' in SOURCE
    assert 'predicted = previous + (source - previous_source)' in SOURCE
    assert 'nominal_branch_is_continuous' in SOURCE
    assert 'candidates[0][0] > MAXIMUM_ADJACENT_CHANGE_RAD' in SOURCE
    assert 'MESSAGE_GAP_S = 0.0' in SOURCE
    assert 'offset += duration + MESSAGE_GAP_S' in SOURCE
    assert 'select_knots(\n                source_positions, DENSE_RESOLUTION_RAD)' in SOURCE


def test_object_relative_stages_blend_at_neutral_boundaries():
    identity = np.eye(4)
    cabinet = identity.copy()
    cabinet[0, 3] = 0.03
    cube = identity.copy()
    cube[1, 3] = -0.06

    assert np.array_equal(target_delta(0, 0, 0, cabinet, cube), identity)
    assert np.array_equal(target_delta(0, 0, 1, cabinet, cube), cabinet)
    assert np.array_equal(target_delta(1, 1, 0, cabinet, cube), cabinet)
    assert np.array_equal(target_delta(1, 1, 1, cabinet, cube), identity)
    assert np.array_equal(target_delta(2, 1, 0, cabinet, cube), identity)
    assert np.array_equal(target_delta(2, 1, 1, cabinet, cube), cabinet)
    assert np.array_equal(target_delta(3, 0, 0, cabinet, cube), identity)
    assert np.array_equal(target_delta(3, 0, 1, cabinet, cube), cube)
    assert np.array_equal(target_delta(3, 2, 0, cabinet, cube), cube)
    assert np.array_equal(target_delta(3, 2, 1, cabinet, cube), cabinet)
    assert np.array_equal(target_delta(3, 5, 0, cabinet, cube), cabinet)
    assert np.array_equal(target_delta(3, 5, 1, cabinet, cube), identity)
