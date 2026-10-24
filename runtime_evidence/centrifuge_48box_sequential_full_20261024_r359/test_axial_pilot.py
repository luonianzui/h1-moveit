import copy
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from axial_pilot import axial_pilot_target
from test_pilot_centering import as_pose
import speed_planning


@pytest.mark.parametrize('payload', ['left', 'right'])
@pytest.mark.parametrize('offset', [[.00093, .001815, -.00025], [-.001, -.0015, 0.]])
@pytest.mark.parametrize('tilt', [-1.9, 0., 1.9])
def test_axis_translation_preserves_orientation_and_y_offset(payload, offset, tilt):
    machine = np.eye(4)
    machine[:3, 3] = [.52, 0., .815]
    bucket = np.eye(4)
    bucket[:3, :3] = Rotation.from_euler('zy', [.5, math.radians(tilt)]).as_matrix()
    bucket[:3, 3] = [.08, .07 if payload == 'left' else -.07, .2375]
    local = np.eye(4)
    local[:3, 3] = offset
    body = machine @ bucket @ local
    poses = dict(centrifuge=as_pose(machine), **{'bucket_' + payload: as_pose(bucket)})
    original = body.copy(), copy.deepcopy(poses)
    target = axial_pilot_target(body, poses, payload)
    final = np.linalg.inv(machine @ bucket) @ target
    assert np.array_equal(target[:3, :3], body[:3, :3])
    assert np.allclose(final[:2, 3], [0., offset[1]], atol=1e-12, rtol=0)
    assert final[2, 3] == pytest.approx(offset[2] + .015, abs=1e-12)
    assert np.array_equal(body, original[0]) and poses == original[1]


@pytest.mark.parametrize('tilt', [2., 2.1, 180.])
def test_original_initial_tilt_guard_kept(tilt):
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('x', tilt, degrees=True).as_matrix()
    with pytest.raises(ValueError, match='two-degree'):
        axial_pilot_target(body, {}, 'left')


def test_runtime_routes_only_retrieval_pilot_to_axial_planner(monkeypatch):
    request = dict(mode='retrieval_pilot', sequential_contract='test', original_runtime_holder_references={'left': {'payload': 'left'}})
    calls = []
    monkeypatch.setattr(speed_planning, 'validate_request', lambda actual: calls.append('validate'))
    monkeypatch.setattr(speed_planning, 'loaded_reference', lambda actual: calls.append('original_holder') or 'ref')
    monkeypatch.setattr(speed_planning, 'retrieval_pilot', lambda actual, reference: calls.append(reference) or {'marker':'axial'})
    monkeypatch.setattr(speed_planning, 'original_plan', lambda actual: pytest.fail('Wrong runtime dispatch'))
    planned = speed_planning.plan(request)
    assert calls == ['validate', 'original_holder', 'ref'] and planned['marker'] == 'axial'
    assert planned['original_runtime_holder_references'] == request['original_runtime_holder_references']


def test_old_worker_and_pilot_geometry_unchanged():
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261021_r340'
    for name in ('sequential_planning.py', 'sequential_worker.py', 'sequential_geometry.py', 'pilot_leveling.py', 'pilot_centering.py'):
        assert (root / name).read_bytes() == (parent / name).read_bytes()


def test_axial_exit_fork_has_only_declared_centering_and_endpoint_updates():
    root = Path(__file__).resolve().parent
    original = (root / 'speed_retrieval_geometry.py').read_text()
    candidate = (root / 'axial_transport.py').read_text()
    changes = [
        ('    from centered_exit import CenteredExitPlanner\n', ''),
        ('planner = CenteredExitPlanner(request, reference)', 'planner = SpeedRetrievalPlanner(request, reference)'),
        ("body, seed = np.array(stage['last_body']), stage['samples'][-1]", "body, seed = target, stage['samples'][-1]"),
    ]
    for changed, previous in changes:
        assert candidate.count(changed) == 1
        candidate = candidate.replace(changed, previous, 1)
    old = "    axis_exit[:3, 3] += (-bucket[:3, 0] * local[0, 3] - bucket[:3, 1] * local[1, 3]\\n                        + bucket[:3, 2] * max(0., float(rim + .006 - bottom)))\\n".replace('\\n', '\n')
    new = "    axis_exit[:3, 3] += bucket[:3, 2] * max(0., float(rim + .006 - bottom))\\n".replace('\\n', '\n')
    assert candidate.count(new) == 1
    assert candidate.replace(new, old, 1) == original


def test_independent_pilot_audit_rejects_old_y_centering_and_leveling():
    from audit_speed_paths import verify_centered_pilot
    body = np.eye(4)
    body[:3, :3] = Rotation.from_euler('x', 1., degrees=True).as_matrix()
    body[:3, 3] = [.001, .002, 1.]
    poses = dict(centrifuge=as_pose(np.eye(4)), bucket_left=as_pose(np.eye(4)),
                 box48_left=as_pose(body))
    request = dict(mode='retrieval_pilot', payload='left', poses=poses)
    from sequential_geometry import pose_matrix
    actual = pose_matrix(poses['box48_left'])
    target = axial_pilot_target(actual, poses, 'left')
    plan = dict(stages=[dict(phase='single_pilot', first_body=actual.tolist(), last_body=target.tolist())])
    verify_centered_pilot(request, plan)
    wrong_y = copy.deepcopy(plan)
    wrong_y['stages'][0]['last_body'][1][3] = 0.
    with pytest.raises(ValueError, match='Y offset'):
        verify_centered_pilot(request, wrong_y)
    wrong_orientation = copy.deepcopy(plan)
    wrong_orientation['stages'][0]['last_body'] = np.eye(4).tolist()
    with pytest.raises(ValueError, match='orientation'):
        verify_centered_pilot(request, wrong_orientation)
