from acceleration_source import restore_acceleration, restore_acceleration_bytes
import copy
from test_return_height_inverse import undo_return_height
import math
from pathlib import Path
import threading

import pytest

from camera_pose_guard import pose_check, pose_values, requested_pose, verified_restore


TARGET = dict(position=dict(x=1.65, y=-1.55, z=1.95),
              orientation=dict(x=0., y=0., z=0., w=1.))


def snapshot(pose=None, sequence=1, received=10.):
    return dict(pose=copy.deepcopy(TARGET if pose is None else pose),
                sequence=sequence, received_monotonic=received)


class Reader:
    def __init__(self, rows):
        self.rows = iter(rows)

    def snapshot(self):
        return next(self.rows)


def test_correct_native_pose_skips_service_even_if_ack_would_timeout():
    def forbidden(*args):
        raise AssertionError('Redundant RPC is forbidden')
    result = verified_restore('request', 'load', Reader([snapshot()]), TARGET,
                              threading.Event(), forbidden, lambda: 10.)
    assert result['status'] == 'PASS' and not result['service_rows']


@pytest.mark.parametrize('accepted', [True, False])
def test_actual_fresh_camera_pose_not_rpc_ack_proves_restoration(accepted):
    wrong = copy.deepcopy(TARGET)
    wrong['position']['x'] += .1
    restore = lambda *args: (accepted, [dict(accepted=accepted)], 'timeout' if not accepted else None)
    result = verified_restore('request', 'load', Reader([snapshot(wrong), snapshot(sequence=2)]),
        TARGET, threading.Event(), restore, lambda: 10.)
    assert result['status'] == 'PASS' and result['pose']['sequence'] == 2


def test_ack_without_new_pose_is_not_camera_success():
    wrong = copy.deepcopy(TARGET)
    wrong['position']['x'] += .1
    ticks = iter([10., 10., 14.])
    with pytest.raises(RuntimeError, match='fresh matching'):
        verified_restore('request', 'phase', Reader([snapshot(wrong)]), TARGET,
            threading.Event(), lambda *args: (True, [], None), lambda: next(ticks))


@pytest.mark.parametrize('row', [None, snapshot(received=8.99), snapshot(received=10.01)])
def test_missing_stale_or_future_readback_does_not_turn_ack_into_proof(row):
    with pytest.raises(RuntimeError, match='cannot be verified'):
        verified_restore('request', 'phase', Reader([row]), TARGET,
            threading.Event(), lambda *args: (True, [], None), lambda: 10.)


@pytest.mark.parametrize('field,value', [('x', math.nan), ('y', math.inf), ('z', -math.inf)])
def test_nonfinite_native_position_rejected(field, value):
    pose = copy.deepcopy(TARGET)
    pose['position'][field] = value
    with pytest.raises(ValueError):
        pose_values(pose)


@pytest.mark.parametrize('value', [0., .9, 1.01, math.nan])
def test_nonunit_or_nonfinite_quaternion_rejected(value):
    pose = copy.deepcopy(TARGET)
    pose['orientation']['w'] = value
    with pytest.raises(ValueError):
        pose_values(pose)


def test_quaternion_double_cover_and_missing_zero_proto_fields_are_valid():
    pose = dict(position=TARGET['position'], orientation=dict(w=-1.))
    check = pose_check(snapshot(pose), TARGET, 10.)
    assert check['valid'] and check['matches']


@pytest.mark.parametrize('distance,angle,expected', [(.0019, .09, True), (.0021, 0., False), (0., .11, False)])
def test_camera_only_error_thresholds(distance, angle, expected):
    pose = copy.deepcopy(TARGET)
    pose['position']['x'] += distance
    pose['orientation'] = dict(w=math.cos(math.radians(angle) / 2), z=math.sin(math.radians(angle) / 2))
    assert pose_check(snapshot(pose), TARGET, 10.)['matches'] is expected


def test_original_front_request_is_strictly_decoded():
    import json
    root = Path(__file__).resolve().parent
    original = root.parent / 'centrifuge_48box_sequential_full_20261019_r299/run/camera_request.json'
    target = requested_pose(json.loads(original.read_text())['request'])
    assert target['position'] == TARGET['position']
    assert pose_check(snapshot(target), target, 10.)['matches']


def test_recorder_uses_native_pose_guard_and_preserves_capture_gates():
    root = Path(__file__).resolve().parent
    current = (root / 'record_window.py').read_text()
    original = (root.parent / 'centrifuge_48box_sequential_full_20261019_r299/record_window.py').read_text()
    assert current.replace('from camera_pose_guard import CameraGuard', 'from camera_guard import CameraGuard') == original


@pytest.mark.parametrize('name', ['sequential_planning.py',
    'sequential_force.py', 'sequential_geometry.py', 'sequential_worker.py', 'sequential_order.py',
    'execute_sequential.py', 'execute_dual.py'])
def test_camera_fix_keeps_r299_actions_byte_exact_after_identity_rebinding(name):
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261019_r299'
    current = restore_acceleration(name, (root / name).read_text()).replace(str(root), str(parent)).replace(root.name, parent.name)
    current = current.replace('install_lab_scene_48box_sequential_full_20261022_r352',
                              'install_lab_scene_48box_sequential_full_20261019_r299')
    current = current.replace('h1_seq_full_r337', 'h1_seq_full_r299').replace(
        'centrifuge_48box_sequential_full_r349', 'centrifuge_48box_sequential_full_r299')
    if name == 'sequential_planning.py':
        current = undo_return_height(current)
        current = current.replace('from load_bound_reserve import InteriorLoadPlanner\n', '')
        current = current.replace('planner = InteriorLoadPlanner(request, reference)', 'planner = SinglePlanner(request, reference)')
        current = current.replace('from pilot_centering import centered_pilot_target\n', '')
        current = current.replace("    if retrieval:\n        target = centered_pilot_target(body, request['poses'], request['payload'])\n", '')
    if name == 'sequential_force.py':
        current = current.replace('RETRIEVAL_FORCE_N = 16.5', 'RETRIEVAL_FORCE_N = 17.5')
    if name == 'sequential_worker.py':
        current = current.replace('from release_alignment import plan_release_alignment\n', '')
        current = current.replace(", 'load_release_align'}", "}")
        current = current.replace("        if mode == 'load_release_align':\n            result = plan_release_alignment(request, reference)\n        elif mode.endswith('_pilot'):", "        if mode.endswith('_pilot'):")
    assert current == (parent / name).read_text()
