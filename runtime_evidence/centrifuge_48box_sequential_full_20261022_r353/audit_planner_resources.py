import gzip
import hashlib
import json
from pathlib import Path

from planner_resources import signature
from prefetch_cycle import drift_within_bounds

ROOT = Path(__file__).resolve().parent
assert json.loads((ROOT / 'run/result.json').read_text())['status'] == 'PASS'
resources = [json.loads(line) for line in (ROOT / 'run/planner_resources.jsonl').read_text().splitlines()]
lifecycle = json.loads((ROOT / 'run/planner_lifecycle.json').read_text())
assert len(resources) == lifecycle['requests'] == len(lifecycle['completed'])
final = resources[-1]['resources']
assert final['model_builds'] == 1 and final['model_reuses'] > 22
assert final['fresh_scenes'] == final['model_reuses'] + 1
assert not final['shared_mutable_scene_or_robot_state'] and not final['cached_paths_or_holder_references']
for name, expected in final['model_dependencies'].items():
    assert tuple(expected['signature']) == signature(Path(name))
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected['sha256']
prefetched, drifts, observations, phases = [], [], [], {}
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        prefix, separator, unused = line.partition(', "payload": ')
        header = json.loads(prefix + '}') if separator else json.loads(line)
        kind = header['kind']
        if kind in ('stationary_prefetch_consumed', 'sequential_actual_snapshot_checked',
                    'static_grip_hold_pass', 'sequential_retrieval_pilot_settle_pass',
                    'stationary_prefetch_drift_rejected'):
            row = json.loads(line)
            assert kind != 'stationary_prefetch_drift_rejected'
            if kind == 'stationary_prefetch_consumed':
                assert observations and observations[-1]['sim'] <= row['sim']
                expected = ('static_grip_hold_pass' if row['payload']['mode'] == 'load_transport'
                    else 'sequential_retrieval_pilot_settle_pass')
                assert observations[-1]['kind'] == expected
                assert row['payload']['consumed_sim'] - row['payload']['submitted_sim'] >= 1.
                prefetched.append(row)
            elif kind == 'sequential_actual_snapshot_checked':
                assert drift_within_bounds(row['payload'])
                drifts.append(row)
            else:
                assert row['payload']['duration_sim_s'] >= 1.
                observations.append(row)
assert len(prefetched) == 2
assert {(row['payload']['mode'], row['payload']['hand']) for row in prefetched} == {
    (mode, hand) for mode in ('load_transport',) for hand in ('left', 'right')}
assert len(drifts) == lifecycle['requests']
with (ROOT / 'planner_resources_audit.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_STATIC_RESOURCE_REUSE_AND_GUARDED_STATIONARY_PREFETCH',
        actual_requests=lifecycle['requests'], resources=final,
        worker_compute_wall_s=sum(row['planning_wall_s'] for row in lifecycle['completed']),
        prefetch_remaining_blocked_wall_s=sum(row['payload']['remaining_wait_wall_s'] for row in prefetched),
        two_loading_observations_and_original_postsettle_retrieval_preserved=True,
        original_live_snapshot_and_holder_checks_passed=True, prefetched=prefetched,
        resource_builds_not_full_task_speedup_certificate=True), stream, indent=2)
print('PLANNER_RESOURCES_ACTUAL_PASS', lifecycle['requests'], flush=True)
