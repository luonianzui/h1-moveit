import inspect
import json
from pathlib import Path

import pytest

from dispatch_gate import validate_dispatch
import execute_cycle


def fixture(root):
    (root / 'run').mkdir()
    rows = [dict(pid=11, created=1, role='server'), dict(pid=12, created=2, role='gui')]
    (root / 'run/controller_health.json').write_text(json.dumps(dict(status='PASS', gazebo_processes=rows)))
    (root / 'run/launch_pid.json').write_text(json.dumps(dict(pid=10, created=0)))
    (root / 'run/video_pid.json').write_text(json.dumps(dict(pid=13, created=3, command=['ffmpeg', str(root / 'run/full_cycle.mp4')])))
    (root / 'run/full_cycle.mp4').write_bytes(b'ftyp')
    (root / 'run/camera_response.log').write_text('data: true')


def test_live_certified_scene_and_recorder_pass_before_claim(tmp_path):
    fixture(tmp_path)
    result = validate_dispatch(tmp_path, lambda entry: True)
    assert result['status'] == 'PASS' and result['before_execution_claim']


@pytest.mark.parametrize('pid', [10, 11, 12, 13])
def test_dead_or_reused_process_rejects(tmp_path, pid):
    fixture(tmp_path)
    with pytest.raises(AssertionError):
        validate_dispatch(tmp_path, lambda entry: entry['pid'] != pid)


@pytest.mark.parametrize('name', ['controller_health.json', 'video_pid.json', 'full_cycle.mp4', 'camera_response.log'])
def test_missing_startup_member_rejects(tmp_path, name):
    fixture(tmp_path)
    (tmp_path / 'run' / name).unlink()
    with pytest.raises((AssertionError, FileNotFoundError)):
        validate_dispatch(tmp_path, lambda entry: True)


def test_unhealthy_unfresh_unacknowledged_or_already_claimed_rejects(tmp_path):
    fixture(tmp_path)
    (tmp_path / 'run/controller_health.json').write_text(json.dumps(dict(status='FAIL', gazebo_processes=[])))
    with pytest.raises(AssertionError):
        validate_dispatch(tmp_path, lambda entry: True)
    (tmp_path / 'run/execution.claim').touch()
    with pytest.raises(AssertionError):
        validate_dispatch(tmp_path, lambda entry: True)


def test_gate_precedes_claim_and_ros_initialization():
    source = inspect.getsource(execute_cycle.main)
    assert source.index('validate_dispatch') < source.index("'run/execution.claim'") < source.index('rclpy.init')

