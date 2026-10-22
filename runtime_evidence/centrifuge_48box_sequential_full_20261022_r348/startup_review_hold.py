import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import psutil

from controller_health import command_health
from dispatch_gate import fresh_gazebo_processes
from scene_contract import read_contract
from startup_window import validate_startup_window
from startup_stats_stream import latest_world_stats
from video_capture import recorder_alive


ROOT = Path(__file__).resolve().parent


def read_json(root, relative):
    return json.loads((root / relative).read_text())


def require_unclaimed(root):
    if any((root / 'run' / name).exists() for name in ('execution.claim', 'process_identities.json', 'result.json', 'review_resume.claim')):
        raise ValueError('Only a never-dispatched startup scene may use the review pause')


def validate_paused(stats):
    stamp = stats.get('simTime', {})
    if not isinstance(stamp, dict) or not any(key in stamp for key in ('sec', 'nsec')):
        raise ValueError('Missing actual simulation timestamp')
    seconds = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) * 1e-9
    if stats.get('paused') is not True or not math.isfinite(seconds):
        raise ValueError('Actual finite paused world statistics required')
    validate_startup_window(seconds)
    return seconds


def world_control(paused):
    command = ['gz', 'service', '-s', '/world/h1_world/control', '--reqtype', 'gz.msgs.WorldControl',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
               'pause: true' if paused else 'pause: false']
    response = subprocess.run(command, capture_output=True, text=True, timeout=8)
    if response.returncode != 0 or 'data: true' not in response.stdout:
        raise RuntimeError('Startup pause/resume service did not acknowledge: ' + response.stdout + response.stderr)
    return dict(argv=command, stdout=response.stdout, stderr=response.stderr, returncode=response.returncode)


def world_stats():
    response = subprocess.run(['gz', 'topic', '-e', '-t', '/world/h1_world/stats', '--json-output', '-n', '1'],
                              capture_output=True, text=True, timeout=8, check=True)
    return latest_world_stats(response.stdout)


def require_scene(root):
    require_unclaimed(root)
    health = read_json(root, 'run/controller_health.json')
    if health['status'] != 'PASS' or {entry['role'] for entry in fresh_gazebo_processes(root)} != {'server', 'gui'}:
        raise ValueError('Owned fresh server, GUI and original six active controllers required')
    entry = read_json(root, 'run/launch_pid.json')
    if abs(psutil.Process(entry['pid']).create_time() - entry['created']) >= .02:
        raise ValueError('Original launch identity changed')


def pause_for_review(root):
    require_scene(root)
    control = world_control(True)
    stats = world_stats()
    stamp = validate_paused(stats)
    with (root / 'run/startup_review_pause.json').open('x') as stream:
        json.dump(dict(status='PASS_PAUSED_BEFORE_ANY_EXECUTOR_OR_PAYLOAD', sim=stamp, stats=stats,
                       control=control, no_reset_detach_or_joint_command=True,
                       original_fresh_window_sim_s=25, current_fresh_window_sim_s=45, review_wall_wait_not_action_time=True), stream, indent=2)


def resume_after_review(root):
    require_scene(root)
    read_contract(root)
    identity = read_json(root, 'identity.json')
    for relative, expected in identity['sha256'].items():
        if hashlib.sha256((root / relative).read_bytes()).hexdigest() != expected:
            raise ValueError('Sealed input changed: ' + relative)
    review = read_json(root, 'run/independent_first_frame_review.json')
    if (review['status'] != 'REVIEWED_BEFORE_EXECUTOR' or not review['actual_image_tool_inspection'] or
            review['source_sha256'] != hashlib.sha256((root / 'run/startup_frame.png').read_bytes()).hexdigest()):
        raise ValueError('The actual current recorded first frame must be reviewed before resume')
    if not recorder_alive(read_json(root, 'run/video_pid.json')):
        raise ValueError('Current original recorder must remain alive')
    paused = read_json(root, 'run/startup_review_pause.json')
    stamp = validate_paused(world_stats())
    if abs(stamp - paused['sim']) > .0005:
        raise ValueError('Physics advanced during first-frame review')
    response = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=20)
    if command_health(response.returncode, response.stdout)['status'] != 'PASS':
        raise ValueError('Original six active controllers lost while paused')
    with (root / 'run/review_resume.claim').open('x') as stream:
        json.dump(dict(status='ONCE_RESUME_THEN_IMMEDIATE_ORIGINAL_DISPATCH', paused_sim=stamp,
                       actual_review_sha256=hashlib.sha256((root / 'run/independent_first_frame_review.json').read_bytes()).hexdigest(),
                       original_fresh_window_sim_s=25, current_fresh_window_sim_s=45, no_reset_or_guard_relaxation=True), stream, indent=2)
    control = world_control(False)
    with (root / 'run/startup_review_resume.json').open('x') as stream:
        json.dump(dict(status='PASS_RESUMED_FOR_IMMEDIATE_ORIGINAL_DISPATCH', control=control,
                       last_paused_sim=stamp, controller_stdout=response.stdout), stream, indent=2)


if __name__ == '__main__':
    if sys.argv[1:] == ['pause']:
        pause_for_review(ROOT)
    elif sys.argv[1:] == ['resume']:
        resume_after_review(ROOT)
        os.execv(sys.executable, [sys.executable, '-u', str(ROOT / 'launch_run.py')])
    else:
        raise ValueError('Exactly pause or resume is required')
