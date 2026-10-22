"""Contracts for the exact-STL left wrist-pitch depth self-mask."""

import ast
from pathlib import Path
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
POINT_CLOUD_PATH = PACKAGE_ROOT / 'h1_2_vision' / 'depth_point_cloud.py'
GAZEBO_LAUNCH_PATH = (
    SOURCE_ROOT / 'h1_2_moveit_config' / 'launch' / 'gazebo.launch.py')
PARAMETER = 'wrist_visual_self_filter_margin'


def _literal_parameter_values(tree, parameter):
    """Return literal values paired with the requested parameter name."""
    values = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            if (len(node.args) >= 2 and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == parameter):
                values.append(ast.literal_eval(node.args[1]))
        elif isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == parameter:
                    values.append(ast.literal_eval(value))
    return values


class LeftWristPitchSelfFilterContract(unittest.TestCase):
    """Keep the mask exact, timestamped, bounded, and perception-only."""

    @classmethod
    def setUpClass(cls):
        """Parse source without importing ROS dependencies."""
        cls.source = POINT_CLOUD_PATH.read_text(encoding='utf-8')
        cls.tree = ast.parse(cls.source, filename=str(POINT_CLOUD_PATH))
        cls.launch_source = GAZEBO_LAUNCH_PATH.read_text(encoding='utf-8')
        cls.launch_tree = ast.parse(
            cls.launch_source, filename=str(GAZEBO_LAUNCH_PATH))

    def _function(self, name):
        return next(
            node for node in self.tree.body
            if isinstance(node, ast.FunctionDef) and node.name == name)

    def test_default_is_zero_and_launch_band_is_seven_millimetres(self):
        """Require explicit opt-in below the independent 10 mm gate."""
        self.assertEqual(
            _literal_parameter_values(self.tree, PARAMETER), [0.0])
        self.assertEqual(
            _literal_parameter_values(self.launch_tree, PARAMETER), [0.007])
        self.assertIn(
            'self._wrist_visual_self_filter_margin <= 0.007',
            self.source)

    def test_filter_uses_shipped_mesh_and_exact_image_time(self):
        """Bind filtering to the collision mesh pose for each depth frame."""
        self.assertIn("'left_wrist_pitch_link.STL'", self.source)
        self.assertIn("'right_wrist_pitch_link.STL'", self.source)
        self.assertEqual(self.source.count("'wrist_yaw_link.STL'"), 2)
        filter_node = next(
            node for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef)
            and node.name == '_filter_wrist_visual_self')
        filter_source = ast.get_source_segment(self.source, filter_node)
        self.assertIn('Time.from_msg(cloud_header.stamp)', filter_source)
        self.assertIn('_points_within_triangle_surface_margin', filter_source)
        self.assertIn('return None', filter_source)
        self.assertNotIn('AllowedCollision', filter_source)
        self.assertNotIn('applyCollision', filter_source)

    def test_exact_triangle_distance_preserves_aabb_voids(self):
        """Prove the helper does not turn the mesh into a padded box."""
        function_names = (
            '_points_inside_euclidean_aabb_margin',
            '_point_triangle_squared_distances',
            '_build_triangle_surface_index',
            '_points_within_triangle_surface_margin',
        )
        nodes = [self._function(name) for name in function_names]
        namespace = {'np': np}
        module = ast.Module(body=nodes, type_ignores=[])
        exec(compile(module, str(POINT_CLOUD_PATH), 'exec'), namespace)
        helper = namespace['_points_within_triangle_surface_margin']
        triangles = np.array([[[0.0, 0.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0]]])
        minimum = np.min(triangles, axis=(0, 1))
        maximum = np.max(triangles, axis=(0, 1))
        points = np.array([
            [0.25, 0.25, 0.007],
            [0.25, 0.25, 0.007001],
            [0.90, 0.90, 0.0],
            [0.50, -0.007, 0.0],
        ])
        surface_index = namespace['_build_triangle_surface_index'](
            triangles, 0.007)
        actual = helper(
            points, triangles, minimum, maximum, 0.007, surface_index)
        np.testing.assert_array_equal(actual, [True, False, False, True])

    def test_spatial_index_does_not_change_exact_result(self):
        """Require acceleration to preserve the unindexed exact decision."""
        function_names = (
            '_points_inside_euclidean_aabb_margin',
            '_point_triangle_squared_distances',
            '_build_triangle_surface_index',
            '_points_within_triangle_surface_margin',
        )
        nodes = [self._function(name) for name in function_names]
        namespace = {'np': np}
        exec(compile(
            ast.Module(body=nodes, type_ignores=[]),
            str(POINT_CLOUD_PATH), 'exec'), namespace)
        rng = np.random.default_rng(826)
        triangles = rng.uniform(-0.03, 0.03, size=(24, 3, 3))
        points = rng.uniform(-0.04, 0.04, size=(50, 3))
        minimum = np.min(triangles, axis=(0, 1))
        maximum = np.max(triangles, axis=(0, 1))
        helper = namespace['_points_within_triangle_surface_margin']
        expected = helper(
            points, triangles, minimum, maximum, 0.007)
        index = namespace['_build_triangle_surface_index'](
            triangles, 0.007)
        actual = helper(
            points, triangles, minimum, maximum, 0.007, index)
        np.testing.assert_array_equal(actual, expected)

    def test_frame_is_dropped_before_publication_if_wrist_tf_is_missing(self):
        """Forbid retaining a self-contaminated frame on TF failure."""
        callback = next(
            node for node in ast.walk(self.tree)
            if isinstance(node, ast.FunctionDef) and node.name == '_depth_callback')
        callback_source = ast.get_source_segment(self.source, callback)
        expected = (
            'points = self._filter_wrist_visual_self('
            'points, message.header)')
        self.assertIn(expected, callback_source)
        self.assertGreaterEqual(callback_source.count('if points is None:'), 2)

    def test_runtime_text_keeps_mask_separate_from_safety_geometry(self):
        """Prevent the render tolerance from becoming a collision waiver."""
        for required in ('exact shipped STL', '10 mm', 'PlanningScene geometry'):
            self.assertIn(required, self.source)
            self.assertIn(required, self.launch_source)


if __name__ == '__main__':
    unittest.main()
