"""Source contract for the bounded right finger-2 depth self-mask."""

import ast
import math
from pathlib import Path
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
POINT_CLOUD_PATH = PACKAGE_ROOT / 'h1_2_vision' / 'depth_point_cloud.py'
GAZEBO_LAUNCH_PATH = (
    SOURCE_ROOT / 'h1_2_moveit_config' / 'launch' / 'gazebo.launch.py')
PARAMETER = 'right_finger2_slider_self_filter_margin'


def _literal_parameter_values(tree, parameter):
    """Return literal values paired with the requested parameter name."""
    values = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Call, ast.Dict)):
            continue
        if isinstance(node, ast.Call):
            if (len(node.args) >= 2 and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == parameter):
                values.append(ast.literal_eval(node.args[1]))
            continue
        for key, value in zip(node.keys, node.values):
            if isinstance(key, ast.Constant) and key.value == parameter:
                values.append(ast.literal_eval(value))
    return values


class RightFinger2SliderSelfFilterContract(unittest.TestCase):
    """Keep the bounded exception local to the intended depth self-mask."""

    @classmethod
    def setUpClass(cls):
        """Parse both Python sources without importing ROS dependencies."""
        cls.point_cloud_source = POINT_CLOUD_PATH.read_text(encoding='utf-8')
        cls.point_cloud_tree = ast.parse(
            cls.point_cloud_source, filename=str(POINT_CLOUD_PATH))
        cls.launch_source = GAZEBO_LAUNCH_PATH.read_text(encoding='utf-8')
        cls.launch_tree = ast.parse(
            cls.launch_source, filename=str(GAZEBO_LAUNCH_PATH))

    def test_node_default_is_zero_and_launch_opt_in_is_five_millimetres(self):
        """Require a safe default and one explicit measured Gazebo value."""
        self.assertEqual(
            _literal_parameter_values(self.point_cloud_tree, PARAMETER),
            [0.0])
        self.assertEqual(
            _literal_parameter_values(self.launch_tree, PARAMETER),
            [0.005])

    def test_parameter_is_finite_and_capped_at_five_millimetres(self):
        """Fail closed on NaN, infinity, negative, or excessive margins."""
        self.assertIn(
            'np.isfinite(self._right_finger2_slider_self_filter_margin)',
            self.point_cloud_source)
        self.assertIn(
            'self._right_finger2_slider_self_filter_margin <= 0.005',
            self.point_cloud_source)
        string_literals = [
            node.value for node in ast.walk(self.point_cloud_tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)]
        self.assertIn(
            'right_finger2_slider_self_filter_margin must be finite and '
            'in [0, 0.005] m', string_literals)

    def test_margin_is_only_a_rounded_euclidean_depth_aabb(self):
        """Keep the margin out of collision and PlanningScene geometry."""
        filter_node = next(
            node for node in ast.walk(self.point_cloud_tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == '_filter_right_finger2_slider_self')
        filter_source = ast.get_source_segment(
            self.point_cloud_source, filter_node)
        self.assertIn('_points_inside_euclidean_aabb_margin', filter_source)
        self.assertNotIn('applyCollision', filter_source)
        self.assertNotIn('AllowedCollision', filter_source)
        for node in ast.walk(filter_node):
            if not isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
                continue
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if isinstance(target, ast.Attribute):
                    self.assertNotIn(
                        target.attr,
                        ('_right_finger2_slider_minimum',
                         '_right_finger2_slider_maximum'))

    def test_euclidean_mask_numeric_boundary_and_corner(self):
        """Prove the 5 mm mask does not become an 8.66 mm box at corners."""
        helper_node = next(
            node for node in self.point_cloud_tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == '_points_inside_euclidean_aabb_margin')
        namespace = {'np': np}
        module = ast.Module(body=[helper_node], type_ignores=[])
        exec(compile(module, str(POINT_CLOUD_PATH), 'exec'), namespace)
        helper = namespace[helper_node.name]
        minimum = np.array([0.0, -0.005, -0.0065])
        maximum = np.array([0.080, 0.009, 0.008])
        measured = np.array([[-0.000134, 0.010134, -0.004]])
        self.assertFalse(helper(measured, minimum, maximum, 0.001)[0])
        self.assertTrue(helper(measured, minimum, maximum, 0.005)[0])
        boundary = np.array([[0.085, 0.009, 0.008]])
        outside = np.array([[0.085001, 0.009, 0.008]])
        self.assertTrue(helper(boundary, minimum, maximum, 0.005)[0])
        self.assertFalse(helper(outside, minimum, maximum, 0.005)[0])
        corner_delta = 0.005 / math.sqrt(3.0) + 1.0e-6
        corner = maximum + corner_delta
        self.assertFalse(helper(
            np.array([corner]), minimum, maximum, 0.005)[0])

    def test_exact_image_time_transform_and_multithreaded_executor(self):
        """Forbid latest-TF fallback while the arm is moving."""
        self.assertIn('Time.from_msg(cloud_header.stamp)',
                      self.point_cloud_source)
        self.assertIn('MultiThreadedExecutor(num_threads=2)',
                      self.point_cloud_source)
        self.assertIn('if points is None:', self.point_cloud_source)

    def test_comments_and_runtime_log_state_the_narrow_tolerance(self):
        """Prevent the perception tolerance from being described as safety."""
        for required in ('self-render/TF', '10 mm'):
            self.assertIn(required, self.point_cloud_source)
            self.assertIn(required, self.launch_source)
        self.assertIn('no PlanningScene geometry', self.point_cloud_source)
        self.assertIn('PlanningScene geometry', self.launch_source)


if __name__ == '__main__':
    unittest.main()
