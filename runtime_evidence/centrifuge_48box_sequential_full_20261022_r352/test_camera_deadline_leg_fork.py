import hashlib
import inspect
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from camera_guard import CameraGuard, restore_with_retries
from execute_cycle import FullCycle
from leg_endpoint import restore_leg_endpoint_source
from button_ramp import INTERPOLATION_GATE
from inherited_source_identity import restore_inherited_source


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_post_retreat_gripper_20261014_r278'


def test_stationary_leg_source_only_declared_duration_fork():
    manifest = json.loads((PARENT / 'identity.json').read_text())['sha256']
    source = PARENT / 'execute_cycle.py'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[source.name]
    expected = source.read_text().replace(str(PARENT), str(EVIDENCE))
    expected = expected.replace('install_lab_scene_48box_initial_approach_20261014_r273',
                                'install_lab_scene_48box_sequential_full_20261022_r352')
    expected = expected.replace('centrifuge_48box_initial_approach_r273',
                                'centrifuge_48box_sequential_full_r349')
    expected = expected.replace('measured=measured, smooth_duration_sim_s=5, unchanged_deviation_gate_rad=.025,',
        'measured=measured, smooth_duration_sim_s=1, original_smooth_duration_sim_s=5,\n'
        '                    unchanged_deviation_gate_rad=.025,')
    expected = expected.replace('last.time_from_start.sec = 5', 'last.time_from_start.sec = 1')
    current = restore_inherited_source('execute_cycle.py', (EVIDENCE / 'execute_cycle.py').read_text())
    for folder in ('centrifuge_48box_dense_endpoint_full_20261015_r281',
                   'centrifuge_48box_empty_protocol_full_20261015_r280',
                   'centrifuge_48box_coarse_ramp_full_20261014_r279'):
        recent = EVIDENCE.parent / folder
        adapter = recent / 'inherited_source_identity.py'
        identity = json.loads((recent / 'identity.json').read_text())['sha256']
        assert hashlib.sha256(adapter.read_bytes()).hexdigest() == identity[adapter.name]
        namespace = {}
        exec(compile(adapter.read_text(), str(adapter), 'exec'), namespace)
        current = namespace['restore_inherited_source']('execute_cycle.py', current)
    assert current.count(INTERPOLATION_GATE) == 1
    assert current == expected
    method = inspect.getsource(FullCycle.initialize_leg_posture)
    assert 'last.time_from_start.nanosec = 50000000' in method
    assert 'positions=targets, velocities=[0.] * 12' in method
    assert 'position=.005, velocity=.03' in method
    assert 'goal.goal_time_tolerance.sec = 10' in method
    assert 'self.dwell(1)' in method


@pytest.mark.parametrize('still_alive', [False, True])
def test_close_covers_three_five_second_attempts_and_preserves_failure(tmp_path, still_alive):
    guard = CameraGuard.__new__(CameraGuard)
    guard.folder = tmp_path
    guard.stop_event = threading.Event()
    guard.rows = []
    guard.fault = None
    joins = []
    guard.thread = SimpleNamespace(join=lambda **options: joins.append(options),
                                   is_alive=lambda: still_alive)
    guard.close()
    assert guard.stop_event.is_set()
    assert joins == [dict(timeout=16)]
    report = json.loads((tmp_path / 'camera_guard.json').read_text())
    assert report['status'] == ('FAIL' if still_alive else 'PASS')
    assert report['maximum_attempts_per_restore'] == 3
    assert report['service_timeout_wall_s'] == 5
    assert report['gui_request_timeout_wall_s'] == 3
    assert report['shutdown_join_timeout_wall_s'] == 16


def test_longer_deadline_does_not_accept_exit_code_zero_without_ack():
    calls = []

    def request(command, **options):
        calls.append((command, options))
        return SimpleNamespace(returncode=0, stdout='', stderr='Service call timed out')

    accepted, rows, error = restore_with_retries('original-pose', 'actual-phase', request,
                                                lambda duration: None)
    assert not accepted and error and len(rows) == len(calls) == 3
    assert all(options['timeout'] == 5 for command, options in calls)
    assert all(command[command.index('--timeout') + 1] == '3000' for command, options in calls)
    assert all(not row['accepted'] for row in rows)
