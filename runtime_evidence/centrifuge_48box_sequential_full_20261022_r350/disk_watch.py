import json
import os
from pathlib import Path
import shutil
import signal
import time

import psutil
from interrupt_control import verify_interrupt_delivery


EVIDENCE = Path(__file__).resolve().parent
MINIMUM_FREE_BYTES = 1024 ** 3


def main():
    deadline = time.monotonic() + 7200
    minimum_seen = shutil.disk_usage(EVIDENCE).free
    target = None
    while time.monotonic() < deadline:
        free = shutil.disk_usage(EVIDENCE).free
        minimum_seen = min(minimum_seen, free)
        if (EVIDENCE / 'run/result.json').exists():
            outcome = dict(status='PASS', no_stop_sent=True, minimum_free_bytes=minimum_seen)
            break
        if target is None:
            matches = []
            for process in psutil.process_iter(['pid', 'create_time', 'cmdline']):
                arguments = process.info['cmdline'] or []
                if arguments and arguments[-1] == str(EVIDENCE / 'execute_cycle.py'):
                    matches.append(process.info)
            if len(matches) > 1:
                raise RuntimeError('Ambiguous executor; no signal sent')
            if matches:
                target = matches[0]
        if free < MINIMUM_FREE_BYTES:
            if target is None:
                raise RuntimeError('Disk low without certified executor; no signal sent')
            process = psutil.Process(target['pid'])
            assert process.create_time() == target['create_time']
            assert process.cmdline()[-1] == str(EVIDENCE / 'execute_cycle.py')
            verify_interrupt_delivery(process.pid)
            os.kill(process.pid, signal.SIGINT)
            outcome = dict(status='DISK_STOP_AND_HOLD', executor=target,
                           signal='SIGINT', minimum_free_bytes=minimum_seen)
            break
        time.sleep(5)
    else:
        raise TimeoutError('Disk observation deadline; no signal sent')
    with (EVIDENCE / 'run/disk_watch.json').open('x') as stream:
        json.dump(outcome, stream, indent=2)
    print(json.dumps(outcome), flush=True)


if __name__ == '__main__':
    main()
