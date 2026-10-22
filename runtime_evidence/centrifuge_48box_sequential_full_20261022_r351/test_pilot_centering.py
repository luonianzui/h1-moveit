import copy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from alignment_scene import pose_matrix
from pilot_centering import centered_pilot_target
import sequential_planning


def as_pose(matrix):
    return dict(position=dict(zip('xyz', matrix[:3, 3])),
                orientation=dict(zip('xyzw', Rotation.from_matrix(matrix[:3,:3]).as_quat())))


@pytest.mark.parametrize('payload', ['left', 'right'])
@pytest.mark.parametrize('offset', [[.0009, .0018, -.00025], [-.001, -.0015, .0001], [0., 0., 0.]])
@pytest.mark.parametrize('tilt', [0., .0168, -.0168])
def test_actual_bucket_both_axes_centered_without_changing_box_orientation(payload, offset, tilt):
    machine = np.eye(4)
    machine[:3,3] = [.52, 0., .815]
    relative = np.eye(4)
    relative[:3,:3] = Rotation.from_euler('xyz', [0., tilt, .45]).as_matrix()
    relative[:3,3] = [.08, .07 if payload == 'left' else -.07, .2375]
    poses = dict(centrifuge=as_pose(machine), **{'bucket_' + payload: as_pose(relative)})
    local = np.eye(4)
    local[:3,3] = offset
    body = machine @ relative @ local
    original = body.copy()
    target = centered_pilot_target(body, poses, payload)
    resulting = np.linalg.inv(machine @ relative) @ target
    assert np.allclose(resulting[:2,3], [0.,0.], atol=1e-12, rtol=0)
    assert resulting[2,3] == pytest.approx(offset[2] + .015, abs=1e-12)
    assert np.array_equal(body, original)
    assert np.array_equal(target[:3,:3], body[:3,:3])


def test_retrieval_dispatch_uses_actual_centered_target_and_original_checks(monkeypatch):
    machine, bucket, body = np.eye(4), np.eye(4), np.eye(4)
    body[:3,3] = [.001,.002,1.]
    bucket[2,3] = 1.
    request = dict(hand='left', payload='left', positions={'joint':0.},
                   poses=dict(centrifuge=as_pose(machine), bucket_left=as_pose(bucket), box48_left=as_pose(body)))
    original = copy.deepcopy(request)
    observed = {}
    planner = SimpleNamespace(names=['joint'], attach=lambda *args: None)
    def stage(first, last, seed, phase, **keywords):
        observed.update(target=last, phase=phase, keywords=keywords)
        return dict(samples=[seed, seed])
    planner.stage = stage
    monkeypatch.setattr(sequential_planning, 'SinglePlanner', lambda *args: planner)
    monkeypatch.setattr(sequential_planning, 'result', lambda *args: {})
    sequential_planning.pilot(request, np.eye(4), retrieval=True)
    assert request == original
    assert np.allclose(observed['target'][:3,3], [0.,0.,1.015])
    assert observed['phase'] == 'single_pilot'
    assert observed['keywords'] == dict(count=21, support_start='basket', refine=False)


def test_nonexplicit_payload_rejected():
    with pytest.raises(ValueError):
        centered_pilot_target(np.eye(4), {}, 'both')
