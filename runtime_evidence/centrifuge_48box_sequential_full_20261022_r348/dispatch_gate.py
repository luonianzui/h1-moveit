import json
from pathlib import Path

import psutil


def process_alive(entry):
    try:
        process = psutil.Process(entry['pid'])
        return (process.is_running() and process.status() != psutil.STATUS_ZOMBIE and
                abs(process.create_time() - entry['created']) < .02)
    except (psutil.Error, KeyError, TypeError, ValueError):
        return False


def fresh_gazebo_processes(root):
    entry = json.loads((root / 'run/launch_pid.json').read_text())
    if not process_alive(entry):
        return []
    rows = []
    for process in psutil.Process(entry['pid']).children(recursive=True):
        try:
            command = process.cmdline()
            role = 'server' if command and command[0].startswith('gz sim server') else (
                'gui' if command and command[0].startswith('gz sim gui') else None)
            if role and process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
                rows.append(dict(pid=process.pid, created=process.create_time(), role=role))
        except psutil.Error:
            pass
    return rows


def validate_dispatch(root, alive=process_alive):
    root = Path(root)
    assert not (root / 'run/execution.claim').exists(), 'Identity already claimed'
    health = json.loads((root / 'run/controller_health.json').read_text())
    assert health['status'] == 'PASS', 'Controllers not certified'
    scoped = health['gazebo_processes']
    assert {entry['role'] for entry in scoped} == {'server', 'gui'}, 'Fresh Gazebo server/GUI missing'
    launch = json.loads((root / 'run/launch_pid.json').read_text())
    video = json.loads((root / 'run/video_pid.json').read_text())
    assert all(alive(entry) for entry in scoped + [launch, video]), 'Certified process died or PID reused'
    assert 'data: true' in (root / 'run/camera_response.log').read_text(), 'Camera not acknowledged'
    path = root / 'run/full_cycle.mp4'
    assert any(argument in (str(path), 'location=' + str(path)) for argument in video['command']) and path.is_file() and path.stat().st_size > 0, 'Video not started'
    return dict(status='PASS', before_execution_claim=True, gazebo_processes=scoped,
                video_pid=video['pid'], no_robot_command=True)

