from acceleration_source import restore_acceleration, restore_acceleration_bytes
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pilot_centering import centered_pilot_target
from pilot_leveling import leveled_retrieval_pilot_target
from test_pilot_centering import as_pose
from test_pilot_leveling_inverse import undo_pilot_leveling


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r317'


def test_only_declared_runtime_target_changes_sealed_parent():
    identity_path = PARENT / 'identity.json'
    assert hashlib.sha256(identity_path.read_bytes()).hexdigest() == 'fa257b89e093693a637b6cb0ad448fa5f8011d6955c063fd209d976a4de6d2db'
    assert undo_pilot_leveling((ROOT / 'sequential_planning.py').read_text()) == (PARENT / 'sequential_planning.py').read_text()
    for name in ('sequential_cycle.py', 'sequential_worker.py', 'sequential_geometry.py', 'sequential_force.py',
                 'pilot_settle.py', 'single_setdown.py', 'pilot_centering.py', 'execute_dual.py'):
        assert restore_acceleration_bytes(name, (ROOT / name).read_bytes()) == (PARENT / name).read_bytes()


@pytest.mark.parametrize('payload', ['left', 'right'])
@pytest.mark.parametrize('tilt', [-1.9, -.96484698755, 0., .96484698755, 1.9])
@pytest.mark.parametrize('yaw', [-2.7, -.58, .31, 2.9])
def test_leveling_keeps_actual_yaw_center_and_fifteen_mm_axis_lift(payload, tilt, yaw):
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('zy', [yaw, math.radians(tilt)]).as_matrix()
    body[:3, 3] = [.6, .08, 1.052]
    machine, bucket = np.eye(4), np.eye(4)
    machine[:3, 3] = [.52, 0., .815]
    bucket[:3, :3] = Rotation.from_euler('xyz', [.01, -.01, .57]).as_matrix()
    bucket[:3, 3] = [.08, .08, .2375]
    poses = dict(centrifuge=as_pose(machine), **{'bucket_' + payload: as_pose(bucket)})
    original_body, original_poses = body.copy(), copy.deepcopy(poses)
    old = centered_pilot_target(body, poses, payload)
    target = leveled_retrieval_pilot_target(body, poses, payload)
    assert np.array_equal(target[:3, 3], old[:3, 3])
    assert np.array_equal(body, original_body) and poses == original_poses
    assert np.allclose(target[:3, 2], [0., 0., 1.], rtol=0, atol=1e-12)
    assert math.atan2(target[1, 0], target[0, 0]) == pytest.approx(math.atan2(body[1, 0], body[0, 0]), abs=1e-12)


@pytest.mark.parametrize('tilt', [2., 2.112487136655531, 5., 180.])
def test_out_of_original_bound_not_hidden_by_level_target(tilt):
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('x', tilt, degrees=True).as_matrix()
    poses = dict(centrifuge=as_pose(np.eye(4)), bucket_left=as_pose(np.eye(4)))
    with pytest.raises(ValueError, match='two-degree'):
        leveled_retrieval_pilot_target(body, poses, 'left')
