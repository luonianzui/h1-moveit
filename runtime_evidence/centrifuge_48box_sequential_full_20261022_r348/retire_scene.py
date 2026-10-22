import json
from pathlib import Path
import signal
import time

import psutil
from retirement_race import active_process, describe_and_interrupt

EVIDENCE = Path(__file__).resolve().parent
identity = json.loads((EVIDENCE / 'run/launch_pid.json').read_text())
targets = {}
try:
    process = psutil.Process(identity['pid'])
    assert abs(process.create_time() - identity['created']) < .01
    targets[process.pid] = process
    targets.update({child.pid: child for child in process.children(recursive=True)})
except psutil.NoSuchProcess:
    pass
for process in psutil.process_iter(['cmdline']):
    if any(str(EVIDENCE / 'inputs/lab_fast_world.sdf') in argument for argument in process.info['cmdline'] or []):
        assert identity['created'] <= process.create_time() <= identity['created'] + 120
        targets[process.pid] = process
        targets.update({child.pid: child for child in process.children(recursive=True)})
records = [describe_and_interrupt(child) for child in targets.values()]
deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    remaining = [child.pid for child in targets.values() if active_process(child)]
    if not remaining:
        break
    time.sleep(.25)
audit = dict(status='PASS' if not remaining else 'FAIL', original_children=records,
             remaining=remaining, normal_sigint=True, no_reset_detach_release=True)
with (EVIDENCE / 'run/retirement.json').open('x') as stream:
    json.dump(audit, stream, indent=2)
print(json.dumps(audit))
assert not remaining

