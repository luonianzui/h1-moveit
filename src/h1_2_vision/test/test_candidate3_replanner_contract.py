from pathlib import Path

from h1_2_perception_msgs.msg import TrackedObjectState
from h1_2_vision.dynamic_scene import object_freshness_error


SOURCE_PATH = Path(__file__).parents[1] / 'h1_2_vision' / 'candidate3_replanner.py'
SOURCE = SOURCE_PATH.read_text(encoding='utf-8')


def test_replanner_has_only_plan_only_publishers():
    expected = {
        "'/candidate3_plan'",
        "'/plan_certificate'",
        "'/candidate3_replan/status'",
    }
    # Publisher topic literals are kept on the continuation lines, so assert
    # the exact allow-list independently and forbid all execution surfaces.
    assert all(topic in SOURCE for topic in expected)
    assert SOURCE.count('self.create_publisher(') == 3
    assert 'ActionClient' not in SOURCE
    assert 'FollowJointTrajectory' not in SOURCE
    assert '/joint_trajectory' not in SOURCE
    assert '/apply_planning_scene' not in SOURCE
    assert 'execute(' not in SOURCE


def test_worker_command_is_plan_only_and_asset_root_is_explicit():
    assert "'asset_root', ''" in SOURCE
    assert "'h1_2_vision.candidate3_worker'" in SOURCE
    assert "'--epoch'" in SOURCE
    assert "'--scene'" in SOURCE
    assert "'--asset-root'" in SOURCE
    assert "'--clearance-auditor'" in SOURCE
    assert "'--clearance-timeout-s'" in SOURCE
    assert "'--output'" in SOURCE


def test_new_epoch_publishes_fail_closed_invalidation():
    assert "message.status = 'SCENE_EPOCH_CHANGED'" in SOURCE
    assert 'message.valid = False' in SOURCE


def test_dynamic_scene_rejects_stale_tag_objects():
    dynamic_source = (SOURCE_PATH.parent / 'dynamic_scene.py').read_text(
        encoding='utf-8')
    assert "return f'STALE_OBJECT:{name}'" in dynamic_source
    value = TrackedObjectState()
    value.header.stamp.sec = 1
    assert object_freshness_error(
        1_600_000_001, {'cube': value}, 0.5) == 'STALE_OBJECT:cube'
    value.header.stamp.sec = 2
    assert object_freshness_error(
        1_600_000_001, {'cube': value}, 0.5) == 'STALE_OBJECT:cube'
