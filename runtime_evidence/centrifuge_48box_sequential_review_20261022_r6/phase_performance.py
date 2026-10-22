import collections
import gzip
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r348'
contacts = collections.defaultdict(lambda: dict(messages=0, contact_points=0, maximum_points=0, basket_contact_messages=0))
machine = collections.defaultdict(lambda: dict(samples=0, peak_abs_rpm=0., peak_abs_brake_torque_nm=0.))
holders = collections.defaultdict(set)
with gzip.open(RUN / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        phase, payload = row['phase'], row['payload']
        if row['kind'] == 'contact':
            count = sum(len(pair.get('positions', [])) for pair in payload['pairs'])
            contact = contacts[phase]
            contact['messages'] += 1
            contact['contact_points'] += count
            contact['maximum_points'] = max(contact['maximum_points'], count)
            contact['basket_contact_messages'] += int(any('bucket_' in pair['first'] + pair['second'] for pair in payload['pairs']))
        if row['kind'] == 'machine':
            status = machine[phase]
            status['samples'] += 1
            status['peak_abs_rpm'] = max(status['peak_abs_rpm'], abs(payload['rotor_rpm']))
            status['peak_abs_brake_torque_nm'] = max(status['peak_abs_brake_torque_nm'], abs(payload['motor_torque_nm']))
        if row['kind'] == 'command':
            holders[phase].add(len(payload.get('holders', {})))
world = [json.loads(line) for line in (ROOT / 'world_stats.jsonl').open()]
world = [row for row in world if row['status'] == 'READ_ONLY_SAMPLE']
intervals = []
def seconds(value):
    return float(value.get('sec', 0)) + float(value.get('nsec', 0)) * 1e-9
for start, finish in zip(world, world[1:]):
    real = seconds(finish['stats']['realTime']) - seconds(start['stats']['realTime'])
    simulated = seconds(finish['stats']['simTime']) - seconds(start['stats']['simTime'])
    if real > 0:
        intervals.append(dict(start_phase=start['phase'], end_phase=finish['phase'],
            same_phase=start['phase'] == finish['phase'], real_s=real, simulated_s=simulated, rtf=simulated / real))
result = dict(status='PHASE_CORRELATION_NOT_FUNCTION_CAUSAL_PROFILE',
    task_status=json.loads((RUN / 'run/result.json').read_text())['status'],
    world_intervals=intervals, contact_counts=dict(contacts), machine_observations=dict(machine),
    actual_holder_counts={name:sorted(values) for name,values in holders.items()},
    contact_messages_not_solver_cost=True,
    read_only_and_no_frequency_or_guard_changes=True)
with (ROOT / 'phase_performance.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(dict(status=result['status'], task_status=result['task_status'],
    retrieval_pilot=machine.get('seq_retrieval_single_pilot'), intervals=len(intervals))))
