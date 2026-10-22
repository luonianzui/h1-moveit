"""Contract tests for the AprilTag-bound cube OctoMap source mask."""

import ast
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
POINT_CLOUD = (
    WORKSPACE / 'src/h1_2_vision/h1_2_vision/depth_point_cloud.py')
GAZEBO_LAUNCH = (
    WORKSPACE / 'src/h1_2_moveit_config/launch/gazebo.launch.py')


def test_cube_mask_uses_hashed_tracked_state_and_exact_timestamp_tf():
    """The legacy unowned /cube_pose path must not drive OctoMap masking."""
    source = POINT_CLOUD.read_text(encoding='utf-8')
    tree = ast.parse(source)
    assert 'TrackedObjectState' in source
    assert "'target_pose_topic', '/tag_pose/cube'" in source
    assert 'message.config_hash != self._target_config_hash' in source
    assert 'target.pose_world.pose.orientation' in source
    assert 'Time.from_msg(cloud_header.stamp)' in source
    assert 'return None' in source
    assert any(
        isinstance(node, ast.FunctionDef) and
        node.name == '_filter_recognized_target'
        for node in ast.walk(tree))


def test_gazebo_launch_routes_cube_tag_state_to_depth_filter():
    """The standard launch must supply the live AprilTag cube topic."""
    source = GAZEBO_LAUNCH.read_text(encoding='utf-8')
    assert "'target_pose_topic': '/tag_pose/cube'" in source
    assert "'target_filter_margin': 0.005" in source
    assert "'target_size_x': 0.080" in source
    assert "'target_size_y': 0.160" in source
    assert "'target_size_z': 0.030" in source
