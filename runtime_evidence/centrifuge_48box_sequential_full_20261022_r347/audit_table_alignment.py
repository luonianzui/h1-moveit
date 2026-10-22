import gzip
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
measurements = {}
plans = []
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rb') as stream:
    for raw in stream:
        header, separator, remainder = raw.partition(b', "payload": ')
        event = json.loads(header + b'}') if separator else json.loads(raw)
        if event['kind'] == 'table_alignment_measured':
            row = json.loads(raw)
            data = row['payload']
            hand = data['hand']
            assert hand == data['payload'] and data['positive_table_pairs']
            assert all('table::' in pair['first'] + pair['second'] and pair['force'] > 0.
                       for pair in data['positive_table_pairs'])
            previous = measurements.setdefault(hand, [])
            assert data['attempt'] == len(previous) and data['attempt'] <= 2
            if previous:
                assert data['original_holders'] == previous[0]['payload']['original_holders']
            assert data['metrics']['position_error_m'] <= .0035
            assert data['metrics']['orientation_error_deg'] <= .25
            assert abs(data['metrics']['height_error_m']) <= .0001
            previous.append(row)
        elif event['kind'] == 'sequential_actual_collision_plan':
            data = json.loads(raw)['payload']
            if data['mode'] == 'retrieval_table_alignment':
                assert data['single_hand'] in measurements
                assert data['original_runtime_holder_references'] == measurements[data['single_hand']][-1]['payload']['original_holders']
                plans.append(data)
task = json.loads((ROOT / 'run/result.json').read_text())
if task['status'] == 'PASS':
    assert set(measurements) == {'left', 'right'}
    assert all(rows[-1]['payload']['metrics']['within_goal'] for rows in measurements.values())
with (ROOT / 'table_alignment_audit.json').open('x') as stream:
    json.dump(dict(status='PASS_SUPPORTED_ALIGNMENT_OBSERVATIONS_NOT_COMPLETE_TASK',
        complete_task=task['status'] == 'PASS', measurements=measurements,
        actual_correction_plans=len(plans), corrective_branch_exercised=bool(plans),
        exact_published_geometry_separately_verified=True), stream, indent=2)
print('TABLE_ALIGNMENT_AUDIT', len(plans), task['status'], flush=True)
