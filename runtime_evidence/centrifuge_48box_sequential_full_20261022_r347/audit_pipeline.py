import gzip
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r333'


def planning_intervals(root):
    intervals, joins = [], []
    pending = None
    completed = 0
    with gzip.open(root / 'run/loading_trace.jsonl.gz', 'rb') as stream:
        for raw in stream:
            prefix, separator, unused = raw.partition(b', "payload": ')
            header = json.loads(prefix + b'}') if separator else json.loads(raw)
            if header['kind'] == 'sequential_actual_planning_started':
                if pending is not None:
                    raise ValueError('Overlapping or unfinished planner request')
                pending = json.loads(raw)
            elif header['kind'] == 'sequential_actual_collision_plan':
                if pending is None:
                    raise ValueError('Unmatched planning result')
                intervals.append(dict(mode=pending['payload']['mode'], hand=pending['payload']['hand'],
                    wall_s=header['wall']-pending['wall'], sim_s=header['sim']-pending['sim']))
                pending = None
            elif header['kind'] == 'joined_segments_declared':
                joins.append(json.loads(raw)['payload'])
            elif header['kind'] == 'joined_segments_completed':
                completed += 1
    if pending is not None:
        raise ValueError('Unfinished actual planning request')
    return intervals, joins, completed


def main():
    assert json.loads((ROOT / 'run/result.json').read_text())['status'] == 'PASS'
    lifecycle = json.loads((ROOT / 'run/planner_lifecycle.json').read_text())
    assert lifecycle['process_launches'] == 1 and lifecycle['returncode'] == 0
    receipts = lifecycle['completed']
    assert len(receipts) == lifecycle['requests'] >= 19
    assert {row['pid'] for row in receipts} == {lifecycle['pid']}
    assert [row['sequence'] for row in receipts] == list(range(1, len(receipts)+1))
    results = sorted((ROOT / 'run').glob('[0-9]*_result.complete.json'), key=lambda path: int(path.name.split('_')[0]))
    assert len(results) == len(receipts)
    for path, expected in zip(results, receipts):
        assert json.loads(path.read_text()) == expected
        destination = path.with_name(path.name.replace('.complete.json', '.json'))
        source = destination.with_name(destination.name.replace('_result.json', '_request.json'))
        assert hashlib.sha256(source.read_bytes()).hexdigest() == expected['source_sha256']
        assert hashlib.sha256(destination.read_bytes()).hexdigest() == expected['result_sha256']
    intervals, joins, completed = planning_intervals(ROOT)
    baseline, unused, unused_count = planning_intervals(PARENT)
    assert len(intervals) == len(receipts) and len(joins) == completed == 4
    assert sorted(tuple(row['segments']) for row in joins) == sorted([
        ('load_lift', 'load_transfer'), ('load_lift', 'load_transfer'),
        ('retrieve_outboard', 'retrieve_transfer'), ('retrieve_outboard', 'retrieve_transfer')])
    current_seconds = sum(row['wall_s'] for row in intervals)
    parent_seconds = sum(row['wall_s'] for row in baseline)
    with (ROOT / 'pipeline_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', actual_single_process=lifecycle['pid'],
            actual_planning_requests=len(intervals), actual_joined_pairs=completed,
            planning_intervals=intervals, actual_planning_wall_s=current_seconds,
            r333_planning_wall_s=parent_seconds, observed_planning_difference_s=parent_seconds-current_seconds,
            not_a_same_state_performance_benchmark=True, steady_spin_and_grip_observations_not_shortened=True), stream, indent=2)
    print('PIPELINE_ACTUAL_AUDIT_PASS', current_seconds, parent_seconds, flush=True)


if __name__ == '__main__':
    main()
