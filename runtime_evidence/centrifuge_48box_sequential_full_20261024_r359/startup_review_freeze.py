import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil

from controller_health import command_health
from scene_contract import read_contract
from startup_review_hold import read_json, require_scene, world_stats
from startup_window import MAXIMUM_FRESH_START_SIM_S, validate_startup_window
from video_capture import recorder_alive


ROOT = Path(__file__).resolve().parent


def measured_clock(stats):
    stamp = stats.get('simTime', {})
    if not isinstance(stamp, dict) or not any(key in stamp for key in ('sec', 'nsec')):
        raise ValueError('Missing actual simulation timestamp')
    seconds = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) * 1e-9
    validate_startup_window(seconds)
    return seconds


def owned_server_process(entry):
    process = psutil.Process(entry['pid'])
    if (not process.is_running() or process.status() == psutil.STATUS_ZOMBIE or
            abs(process.create_time() - entry['created']) >= .02 or
            process.cmdline() != entry['command'] or not entry['command'][0].startswith('gz sim server')):
        raise ValueError('Exact owned original server identity required')
    return process


def send_server_signal(entry, requested):
    if requested not in (signal.SIGSTOP, signal.SIGCONT):
        raise ValueError('Only startup SIGSTOP or SIGCONT is supported')
    process = owned_server_process(entry)
    if (process.status() == psutil.STATUS_STOPPED) != (requested == signal.SIGCONT):
        raise ValueError('Server is not in the required original startup process state')
    process.send_signal(requested)
    return process


def freeze_for_review(root):
    require_scene(root)
    if (root / 'run/startup_review_freeze.json').exists():
        raise ValueError('Startup review freeze identity already used')
    stats = world_stats()
    stamp = measured_clock(stats)
    servers = [entry for entry in read_json(root, 'run/controller_health.json')['gazebo_processes'] if entry['role'] == 'server']
    if len(servers) != 1:
        raise ValueError('Exactly one original owned server required')
    entry = dict(servers[0])
    entry['command'] = psutil.Process(entry['pid']).cmdline()
    process = send_server_signal(entry, signal.SIGSTOP)
    deadline = time.monotonic() + 3
    while process.status() != psutil.STATUS_STOPPED and time.monotonic() < deadline:
        time.sleep(.01)
    if process.status() != psutil.STATUS_STOPPED:
        raise RuntimeError('Original server did not enter the explicit startup review stop')
    with (root / 'run/startup_review_freeze.json').open('x') as stream:
        json.dump(dict(status='PASS_UNCLAIMED_SERVER_FROZEN_FOR_ACTUAL_IMAGE_REVIEW', server=entry,
                       last_observed_sim=stamp, stats_before_stop=stats, signal_sent='SIGSTOP',
                       no_world_control_pause_or_duplicate_clock_filter=True,
                       original_relay_source_step_and_all_guards_unchanged=True,
                       no_reset_detach_or_joint_command=True), stream, indent=2)


def resume_after_review(root):
    require_scene(root)
    read_contract(root)
    for relative, expected in read_json(root, 'identity.json')['sha256'].items():
        if hashlib.sha256((root / relative).read_bytes()).hexdigest() != expected:
            raise ValueError('Sealed input changed: ' + relative)
    review = read_json(root, 'run/independent_first_frame_review.json')
    if (review['status'] != 'REVIEWED_BEFORE_EXECUTOR' or not review['actual_image_tool_inspection'] or
            review['source_sha256'] != hashlib.sha256((root / 'run/startup_frame.png').read_bytes()).hexdigest()):
        raise ValueError('Actual current original first frame review required before resume')
    if not recorder_alive(read_json(root, 'run/video_pid.json')):
        raise ValueError('Original current recorder identity lost')
    frozen = read_json(root, 'run/startup_review_freeze.json')
    if owned_server_process(frozen['server']).status() != psutil.STATUS_STOPPED:
        raise ValueError('Original server was resumed before the independent review')
    with (root / 'run/review_resume.claim').open('x') as stream:
        json.dump(dict(status='ONCE_SIGCONT_THEN_IMMEDIATE_ORIGINAL_DISPATCH', server=frozen['server'],
                       original_fresh_window_sim_s=25, current_fresh_window_sim_s=MAXIMUM_FRESH_START_SIM_S,
                       startup_age_gate_explicitly_relaxed=True, identity_and_live_health_guards_preserved=True), stream, indent=2)
    send_server_signal(frozen['server'], signal.SIGCONT)
    response = subprocess.run(['ros2', 'control', 'list_controllers'], capture_output=True, text=True, timeout=8)
    if command_health(response.returncode, response.stdout)['status'] != 'PASS':
        raise ValueError('Original six active controllers lost after resume')
    with (root / 'run/startup_review_resume.json').open('x') as stream:
        json.dump(dict(status='PASS_SIGCONT_THEN_ORIGINAL_DISPATCH', server=frozen['server'],
                       signal_sent='SIGCONT', controller_stdout=response.stdout,
                       no_world_pause_reset_detach_or_clock_filter=True), stream, indent=2)


if __name__ == '__main__':
    if sys.argv[1:] == ['freeze']:
        freeze_for_review(ROOT)
    elif sys.argv[1:] == ['resume']:
        resume_after_review(ROOT)
        os.execv(sys.executable, [sys.executable, '-u', str(ROOT / 'launch_run.py')])
    else:
        raise ValueError('Exactly freeze or resume is required')
