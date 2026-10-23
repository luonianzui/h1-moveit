import json
from pathlib import Path
import subprocess
import sys
import time

import psutil
from owned_telemetry import capture_owned_telemetry


EVIDENCE = Path(__file__).resolve().parent


def main():
    capture_owned_telemetry(EVIDENCE)
    children = {}
    streams = []
    records = {}
    started_ns = {}
    for role, script, relative in (
            ('executor', 'execute_sequential.py', 'run/execution.log'),
            ('guard', 'sequential_run_guard.py', 'run/run_guard.log'),
            ('disk', 'sequential_disk_watch.py', 'run/disk_watch.log'),
            ('supervisor', 'supervise_completion.py', 'supervisor.log')):
        stream = (EVIDENCE / relative).open('x')
        streams.append(stream)
        command = [sys.executable, str(EVIDENCE / script)]
        started_ns[role] = time.monotonic_ns()
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        children[role] = process
        records[role] = dict(pid=process.pid, created=psutil.Process(process.pid).create_time(), command=command)
    with (EVIDENCE / 'run/process_identities.json').open('x') as stream:
        json.dump(records, stream, indent=2)
    print('EXECUTOR_STARTED', records['executor'], flush=True)
    outcomes, finished_ns = {}, {}
    for role, process in children.items():
        outcomes[role] = process.wait()
        finished_ns[role] = time.monotonic_ns()
    with (EVIDENCE / 'run/executor_wall_clock.json').open('x') as stream:
        elapsed = (finished_ns['executor'] - started_ns['executor']) / 1e9
        json.dump(dict(clock='monotonic_ns', started_ns=started_ns['executor'],
                       observed_exit_ns=finished_ns['executor'], elapsed_wall_s=elapsed,
                       scope='executor_spawn_through_observed_exit_including_imports_prechecks_plans_and_shutdown',
                       excludes_scene_launch_video_review_and_recording_finalization=True,
                       within_180_wall_s=elapsed <= 180., execution_returncode=outcomes['executor'],
                       not_independent_physical_certificate=True), stream, indent=2)
    with (EVIDENCE / 'run/process_exit_status.json').open('x') as stream:
        json.dump(outcomes, stream, indent=2)
    for stream in streams:
        stream.close()
    print('FINISHED', outcomes, flush=True)
    assert all(status == 0 for status in outcomes.values()), outcomes


if __name__ == '__main__':
    main()
