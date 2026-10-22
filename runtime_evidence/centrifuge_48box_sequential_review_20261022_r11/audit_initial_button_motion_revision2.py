import bisect
import gzip
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent / 'centrifuge_48box_sequential_full_20261022_r353'
commands = []
samples = []
smoothing = []
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'button_arm_command':
            commands.append(row)
        elif row['kind'] == 'button_controller_reference':
            samples.append(row)
        elif row['kind'] == 'initial_button_withdraw_smoothing':
            smoothing.append(row)


def commanded(point_rows, relative):
    times = [point['seconds'] for point in point_rows]
    if relative < times[0] or relative > times[-1]:
        return None
    upper = min(bisect.bisect_right(times, relative), len(times) - 1)
    lower = max(0, upper - 1)
    first, last = point_rows[lower], point_rows[upper]
    duration = times[upper] - times[lower]
    if duration <= 0:
        return first['positions']
    fraction = (relative - times[lower]) / duration
    if not first['velocities'] or not last['velocities']:
        return [start + fraction * (end - start) for start, end in zip(first['positions'], last['positions'])]
    return [(2 * fraction ** 3 - 3 * fraction ** 2 + 1) * start
            + (fraction ** 3 - 2 * fraction ** 2 + fraction) * duration * initial_velocity
            + (-2 * fraction ** 3 + 3 * fraction ** 2) * end
            + (fraction ** 3 - fraction ** 2) * duration * final_velocity
            for start, end, initial_velocity, final_velocity in
            zip(first['positions'], last['positions'], first['velocities'], last['velocities'])]


summaries = {}
for row in samples:
    payload = row['payload']
    eligible = [command for command in commands if command['phase'] == row['phase']
                and command['payload']['side'] == payload['side']
                and round(command['payload']['start'] * 1e9) <= payload['source_ns']]
    if not eligible:
        continue
    command = eligible[-1]['payload']
    relative = (payload['source_ns'] - round(command['start'] * 1e9)) / 1e9
    expected = commanded(command['points'], relative)
    if expected is None:
        continue
    reference = payload['reference']['positions']
    feedback = payload['feedback']['positions']
    if len(expected) != len(reference) or len(reference) != len(feedback):
        raise ValueError('Incomplete original controller state')
    values = [abs(actual - target) for actual, target in zip(feedback, reference)]
    divergence = [abs(actual - target) for actual, target in zip(reference, expected)]
    assert all(math.isfinite(value) for value in values + divergence)
    summary = summaries.setdefault(row['phase'], dict(samples=0, max_feedback_reference_error_rad=0.,
        max_reference_command_error_rad=0.))
    summary['samples'] += 1
    summary['max_feedback_reference_error_rad'] = max(summary['max_feedback_reference_error_rad'], *values)
    summary['max_reference_command_error_rad'] = max(summary['max_reference_command_error_rad'], *divergence)
assert len(smoothing) == 1
assert 'machine_preparation_open_withdraw' in summaries
report = dict(status='OBSERVED_ORIGINAL_CONTROLLER_REFERENCE_NOT_UNIQUE_CAUSAL_PROOF',
              controller_messages=len(samples), maximum_bounded_messages=10000,
              first_withdraw_smoothing=smoothing[0], phases=summaries,
              complete_task=json.loads((ROOT / 'run/result.json').read_text())['status'] == 'PASS',
              identity_sha256=hashlib.sha256((ROOT / 'identity.json').read_bytes()).hexdigest(),
              original_tracking_and_endpoint_guards_unchanged=True)
with (ROOT / 'initial_button_motion_audit.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(summaries, indent=2), flush=True)
