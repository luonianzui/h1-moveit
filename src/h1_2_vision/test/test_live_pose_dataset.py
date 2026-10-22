"""Unit contracts for live moved-object pose evidence."""

import math

from h1_2_vision.live_pose_dataset import (
    expected_quaternion,
    pose_errors,
    summarize,
)
import numpy as np
import pytest


def sample(translation, rotation, depth=0.001, reprojection=0.1):
    return {
        'translation_error_m': translation,
        'rotation_error_rad': rotation,
        'maximum_depth_residual_m': depth,
        'maximum_reprojection_error_px': reprojection,
        'consecutive_frames': 7,
        'nearest_depth_skew_ns': 1_000_000,
        'nearest_cloud_skew_ns': 2_000_000,
    }


def test_pose_errors_are_sign_invariant_and_metric():
    expected = expected_quaternion(math.pi / 2)
    translation, rotation = pose_errors(
        [1.003, 2.004, 3.0], -expected,
        [1.0, 2.0, 3.0], expected)
    assert translation == pytest.approx(0.005)
    assert rotation == pytest.approx(0.0)


def test_pose_errors_reject_nonfinite_and_zero_quaternion():
    with pytest.raises(ValueError):
        pose_errors([math.nan, 0, 0], [0, 0, 0, 1],
                    [0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(ValueError):
        pose_errors([0, 0, 0], [0, 0, 0, 0],
                    [0, 0, 0], [0, 0, 0, 1])


def test_summary_preserves_worst_case_and_rms():
    result = summarize([sample(0.001, 0.01), sample(0.003, 0.03)])
    assert result['sample_count'] == 2
    assert result['translation_error_max_m'] == pytest.approx(0.003)
    assert result['translation_error_rms_m'] == pytest.approx(
        math.sqrt(5e-6))
    assert result['rotation_error_max_rad'] == pytest.approx(0.03)
    assert result['depth_residual_max_m'] == pytest.approx(0.001)
    assert result['maximum_nearest_cloud_skew_ns'] == 2_000_000


def test_summary_rejects_empty_or_nonfinite_samples():
    with pytest.raises(ValueError):
        summarize([])
    bad = sample(0.0, 0.0)
    bad['maximum_depth_residual_m'] = np.inf
    with pytest.raises(ValueError):
        summarize([bad])


def test_collector_has_schema_and_nonoverwriting_output_contract():
    source = (__import__('pathlib').Path(__file__).parents[1] /
              'h1_2_vision/live_pose_dataset.py').read_text()
    assert "SCHEMA = 'h1_apriltag_live_pose_dataset/v1'" in source
    assert 'os.O_EXCL' in source
    assert 'if self.samples:\n                self.fail' in source
    assert "'valid': False" in source
    assert "'valid': True" in source
