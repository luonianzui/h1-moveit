"""Unit and static authority tests for the middle-v1 live capture."""

from pathlib import Path

from h1_2_perception_msgs.msg import SceneEpoch
from h1_2_vision.candidate3_middle_capture import (
    _validate_pair,
    CaptureError,
    REQUIRED_WORLD_IDS,
)
from h1_2_vision.candidate3_middle_contract import (
    SEAM_JOINT_NAMES,
    SEAM_NOMINAL,
)
from moveit_msgs.msg import CollisionObject, PlanningScene
import pytest


SOURCE = (Path(__file__).parents[1] / 'h1_2_vision' /
          'candidate3_middle_capture.py').read_text(encoding='utf-8')


def _pair():
    epoch = SceneEpoch()
    epoch.epoch_id = 7
    epoch.scene_hash = 'a' * 64
    epoch.valid = True
    epoch.stable = True
    epoch.latch_state = 'unlocked'
    epoch.right_door_support_state = 'DETACHED_NO_ENDPOINT_PLAN_ONLY'
    epoch.left_cube_support_state = 'DETACHED_NO_ENDPOINT_PLAN_ONLY'
    epoch.mechanism_state.name = ['door_hinge_joint', 'handle_latch_joint']
    epoch.mechanism_state.position = [0.0, 1.5707963267948966]
    epoch.robot_state.name = list(SEAM_JOINT_NAMES)
    epoch.robot_state.position = list(SEAM_NOMINAL)

    scene = PlanningScene()
    scene.name = f'h1_scene_epoch:7:{epoch.scene_hash}'
    scene.is_diff = False
    scene.robot_state.joint_state = epoch.robot_state
    for object_id in sorted(REQUIRED_WORLD_IDS):
        value = CollisionObject()
        value.id = object_id
        scene.world.collision_objects.append(value)
    return epoch, scene


def test_capture_source_has_subscription_only_authority():
    assert 'create_subscription(' in SOURCE
    assert 'create_publisher(' not in SOURCE
    assert 'create_client(' not in SOURCE
    assert 'ActionClient' not in SOURCE
    assert 'controller_commands_issued' in SOURCE
    assert SOURCE.count('/joint_trajectory') == 5


def test_valid_q90_pair_passes():
    epoch, scene = _pair()
    result = _validate_pair(epoch, scene, 1.5707963267948966, 0.01, 0.01)
    assert result['scene_epoch_id'] == 7
    assert result['controller_commands_issued'] == 0
    assert result['mechanism_positions'] == [0.0, 1.5707963267948966]


@pytest.mark.parametrize('field', [
    'right_door_support_state', 'left_cube_support_state'])
def test_attached_support_is_rejected(field):
    epoch, scene = _pair()
    setattr(epoch, field, 'attached')
    with pytest.raises(CaptureError, match='SUPPORT_NOT_DETACHED'):
        _validate_pair(epoch, scene, 1.5707963267948966, 0.01, 0.01)


def test_seam_and_scene_binding_are_fail_closed():
    epoch, scene = _pair()
    epoch.robot_state.position[0] += 0.01
    with pytest.raises(CaptureError, match='SEAM_JOINT'):
        _validate_pair(epoch, scene, 1.5707963267948966, 0.01, 0.01)
    epoch, scene = _pair()
    scene.name = 'wrong'
    with pytest.raises(CaptureError, match='SCENE_BINDING'):
        _validate_pair(epoch, scene, 1.5707963267948966, 0.01, 0.01)
