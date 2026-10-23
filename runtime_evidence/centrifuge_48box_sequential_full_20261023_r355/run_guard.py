import json
import os
from pathlib import Path
import shutil
import signal
import time

import psutil
from video_capture import recorder_alive
from interrupt_control import verify_interrupt_delivery
from owned_telemetry import owned_telemetry_fault


EVIDENCE = Path(__file__).resolve().parent
MINIMUM_FREE_BYTES = 512 * 1024 ** 2


def stop_executor(target):
    if target is None:
        raise RuntimeError('No certified executor; no signal sent')
    process = psutil.Process(target['pid'])
    assert process.create_time() == target['create_time']
    assert process.cmdline()[-1] == str(EVIDENCE / 'execute_cycle.py')
    verify_interrupt_delivery(process.pid)
    os.kill(process.pid, signal.SIGINT)


def main():
    telemetry = None
    video = json.loads((EVIDENCE / 'run/video_pid.json').read_text())
    video_path = EVIDENCE / 'run/full_cycle.mp4'
    target = None
    deadline = time.monotonic() + 7200
    last_size = video_path.stat().st_size
    last_growth = time.monotonic()
    minimum_seen = shutil.disk_usage(EVIDENCE).free
    observations = 0
    while time.monotonic() < deadline:
        if (EVIDENCE / 'run/result.json').exists():
            outcome = dict(status='PASS', no_stop_sent=True, minimum_free_bytes=minimum_seen,
                           observations=observations)
            break
        if target is None:
            if telemetry is None:
                telemetry = json.loads((EVIDENCE / 'run/owned_telemetry.json').read_text())
            matches = [process.info for process in psutil.process_iter(['pid', 'create_time', 'cmdline'])
                       if process.info['cmdline'] and process.info['cmdline'][-1] == str(EVIDENCE / 'execute_cycle.py')]
            if len(matches) > 1:
                raise RuntimeError('Ambiguous executor; no signal sent')
            target = matches[0] if matches else None
        free = shutil.disk_usage(EVIDENCE).free
        minimum_seen = min(minimum_seen, free)
        current_size = video_path.stat().st_size if video_path.exists() else 0
        if current_size > last_size:
            last_growth = time.monotonic()
        last_size = current_size
        reason = owned_telemetry_fault(telemetry)
        if free < MINIMUM_FREE_BYTES:
            reason = 'disk_below_512MiB'
        elif not recorder_alive(video):
            reason = 'recorder_missing_or_identity_changed'
        elif time.monotonic() - last_growth > 20:
            reason = 'video_not_growing_20_wall_seconds'
        observations += 1
        if reason:
            stop_executor(target)
            outcome = dict(status='STOP_AND_HOLD', reason=reason, executor=target,
                           signal='SIGINT', minimum_free_bytes=minimum_seen, observations=observations)
            break
        time.sleep(1)
    else:
        raise TimeoutError('Guard deadline; no signal sent')
    with (EVIDENCE / 'run/run_guard.json').open('x') as stream:
        json.dump(outcome, stream, indent=2)
    print(json.dumps(outcome), flush=True)


if __name__ == '__main__':
    main()
