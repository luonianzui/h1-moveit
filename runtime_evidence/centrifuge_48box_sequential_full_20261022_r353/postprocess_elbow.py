import json
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parent
deadline = time.monotonic() + 7200
while time.monotonic() < deadline:
    if (ROOT / 'run/completion_finalization.json').is_file() and (ROOT / 'run/process_exit_status.json').is_file():
        break
    time.sleep(5)
else:
    raise TimeoutError('No normal completion; no reset or release issued')
statuses = {}
for name in ('audit_trace.py', 'audit_cycle.py', 'audit_dual_grip.py', 'audit_dual_speed.py',
             'audit_elbow.py', 'summarize_actual_phases.py', 'extract_actual_video_phases.py',
             'verify_dual_protection.py'):
    command = [sys.executable, str(ROOT / name)]
    if name == 'verify_dual_protection.py':
        command.append('after')
    with (ROOT / ('posthoc_' + name.removesuffix('.py') + '.log')).open('x') as stream:
        code = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT).returncode
    statuses[name] = code
    print('ELBOW_POSTHOC', name, code, flush=True)
with (ROOT / 'posthoc_statuses.json').open('x') as stream:
    json.dump(dict(execution_status=json.loads((ROOT / 'run/result.json').read_text())['status'],
                   returncodes=statuses, old_speed_gate_not_redefined=True,
                   current_task_is_elbow_posture_full_recording_not_wallclock_180_claim=True), stream, indent=2)
print('ELBOW_POSTHOC_FINISHED', statuses, flush=True)
