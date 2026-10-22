"""Explicit fail-closed contracts for the stage-I negative matrix."""

from collections import defaultdict, deque
import copy
import json
import math
from pathlib import Path

from builtin_interfaces.msg import Time
from h1_2_perception_msgs.msg import TrackedObjectState
from h1_2_vision.candidate3_worker import reverse_audit
from h1_2_vision.dynamic_scene import object_freshness_error
from h1_2_vision.tag_config import load_tag_map
from h1_2_vision.tag_localizer import TagLocalizer
import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
PACKAGE = WORKSPACE / 'src/h1_2_vision'
CONFIG = PACKAGE / 'config/tag_map.yaml'
TEXTURES = [
    WORKSPACE / 'src/h1_2_description/models/incubator/materials/textures',
    WORKSPACE / 'src/h1_2_description/models/green_cube/materials/textures',
]
EVIDENCE = (
    WORKSPACE / 'runtime_evidence/826_apriltag_plan_only_20260826')
POSITIVE_SUMMARY = EVIDENCE / 'stage_i_positive_g5/summary.json'


def tracked(stamp_seconds):
    value = TrackedObjectState()
    value.header.stamp.sec = stamp_seconds
    value.valid = True
    value.stable = True
    return value


def stability_harness(required=3, translation_limit=0.001):
    value = TagLocalizer.__new__(TagLocalizer)
    value.last_stamp = {}
    value.history = defaultdict(lambda: deque(maxlen=required))
    value.consecutive = defaultdict(int)
    value.settings = {'consecutive_frames': required}
    value.config = {'objects': {'cube': {
        'stable_translation_m': translation_limit,
        'stable_rotation_rad': math.radians(1.0),
    }}}
    return value


def pose_at(x):
    value = np.eye(4)
    value[0, 3] = x
    return value


def stamp(index):
    value = Time()
    value.sec = index
    return value


def test_cabinet_and_cube_occlusion_expire_without_pose_fallback():
    now = 2_000_000_000
    assert object_freshness_error(
        now, {'cabinet': tracked(1)}, 0.25) == 'STALE_OBJECT:cabinet'
    assert object_freshness_error(
        now, {'cube': tracked(1)}, 0.25) == 'STALE_OBJECT:cube'


def test_wrong_tag_id_is_ignored_and_cannot_create_an_object_pose():
    config = load_tag_map(CONFIG, TEXTURES)
    assert config['tags_by_id'].get(999) is None
    source = (PACKAGE / 'h1_2_vision/tag_localizer.py').read_text()
    assert "record = self.config['tags_by_id'].get(int(marker_id))" in source
    assert 'if record is None:\n                    continue' in source


def test_stale_image_and_unavailable_exact_tf_have_no_latest_fallback():
    assert object_freshness_error(
        2_000_000_000, {'cube': tracked(1)}, 0.25) == 'STALE_OBJECT:cube'
    source = (PACKAGE / 'h1_2_vision/tag_localizer.py').read_text()
    assert 'Time.from_msg(color_msg.header.stamp)' in source
    assert 'timeout=Duration(seconds=0.0)' in source
    assert 'exact-stamp TF rejected tag' in source
    assert 'lookup_transform(\n            target, source, Time()' not in source


def test_pose_jump_resets_stability_window():
    localizer = stability_harness()
    assert TagLocalizer.stable(localizer, 'cube', pose_at(0), stamp(1))[0] is False
    assert TagLocalizer.stable(
        localizer, 'cube', pose_at(0.0002), stamp(2))[0] is False
    accepted, consecutive = TagLocalizer.stable(
        localizer, 'cube', pose_at(0.020), stamp(3))
    assert accepted is False
    assert consecutive == 0
    assert len(localizer.history['cube']) == 1


def test_continuously_moving_environment_never_freezes():
    localizer = stability_harness()
    accepted = []
    for index in range(1, 13):
        result, _ = TagLocalizer.stable(
            localizer, 'cube', pose_at(index * 0.002), stamp(index))
        accepted.append(result)
    assert not any(accepted)


def test_new_obstacle_epoch_invalidates_previous_certificate():
    source = (PACKAGE / 'h1_2_vision/candidate3_replanner.py').read_text()
    assert 'if self.published_key is not None and key != self.published_key:' in source
    assert 'self.publish_invalidated(key)' in source
    assert "message.status = 'SCENE_EPOCH_CHANGED'" in source
    assert 'message.valid = False' in source


def trajectory_message(width, values):
    return {
        'joint_names': [f'joint_{index}' for index in range(width)],
        'points': [{'positions': list(point)} for point in values],
    }


def paired_stages():
    arm = [[0.0] * 7, [0.1] * 7]
    fingers = [[-0.019] * 2, [0.010] * 2]
    forward = {
        'messages': [trajectory_message(7, arm),
                     trajectory_message(2, fingers)],
    }
    reverse = {
        'messages': [trajectory_message(2, fingers[::-1]),
                     trajectory_message(7, arm[::-1])],
    }
    stages = [copy.deepcopy(forward) for _ in range(4)]
    stages.extend(copy.deepcopy(reverse) for _ in range(3))
    return stages


def test_reverse_path_mutation_fails_closed():
    stages = paired_stages()
    assert reverse_audit(stages)
    stages[4]['messages'][1]['points'][0]['positions'][0] += 0.001
    assert not reverse_audit(stages)


def test_matrix_contains_explicit_ik_and_clearance_failures():
    summary = json.loads(POSITIVE_SUMMARY.read_text())
    failures = [value for value in summary['cases'] if not value['valid']]
    assert any('IK_NO_SOLUTION' in value['status'] for value in failures)
    clearance = [value for value in failures if value['status'] == 'CLEARANCE']
    assert clearance
    assert all(value['collision_free'] is True for value in clearance)
    assert all(value['minimum_environment_clearance_m'] < 0.010
               for value in clearance)
