import json
from pathlib import Path
import subprocess


EVIDENCE = Path(__file__).resolve().parent
result_path = EVIDENCE / 'run/result.json'
result = json.loads(result_path.read_text()) if result_path.is_file() else None
pause = subprocess.run(
    ['gz', 'service', '-s', '/world/h1_world/control', '--reqtype', 'gz.msgs.WorldControl',
     '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'pause: true'],
    capture_output=True, text=True, timeout=8)
assert pause.returncode == 0 and 'data: true' in pause.stdout, pause.stdout + pause.stderr
stats_result = subprocess.run(
    ['gz', 'topic', '-e', '-t', '/world/h1_world/stats', '--json-output', '-n', '1'],
    capture_output=True, text=True, timeout=8, check=True)
stats = json.loads(stats_result.stdout)
assert stats.get('paused') is True, stats
record = {
    'status': 'PASS',
    'result_status': result.get('status') if result else 'MISSING_RESULT',
    'holding': result.get('holding') if result else 'unknown',
    'pause_requested': True,
    'paused_stats': stats,
    'no_reset_or_detach': True,
    'failure_policy': 'executor STOP_AND_HOLD; pause simulation; do not reset, detach, or release',
}
with (EVIDENCE / 'run/pause.json').open('x') as stream:
    json.dump(record, stream, indent=2)
print(json.dumps(record, separators=(',', ':')))

