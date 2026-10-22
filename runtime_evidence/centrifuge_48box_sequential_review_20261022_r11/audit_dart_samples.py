import bisect
import collections
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r353/run'
runtime = json.loads((RUN / 'dart_cache_runtime_identity.json').read_text())
clock = json.loads((RUN / 'executor_wall_clock.json').read_text())
started, ended = clock['started_ns'] / 1e9, clock['observed_exit_ns'] / 1e9
phases = sorted((json.loads(line) for line in (ROOT / 'server_thread_samples.jsonl').read_text().splitlines()),
                key=lambda row: row['monotonic'])
stamps = [row['monotonic'] for row in phases]
rows = [json.loads(line) for line in (RUN / 'dart_constraint_samples.jsonl').read_text().splitlines()]
assert all(row['pid'] == runtime['pid'] and row['tid'] == runtime['tid'] for row in rows)
assert all(finish['call'] - start['call'] == 1024 for start, finish in zip(rows, rows[1:]))
task = [row for row in rows if started <= row['mono'] <= ended]
assert len(task) >= 100
grouped = collections.defaultdict(list)
for row in task:
    position = bisect.bisect_right(stamps, row['mono']) - 1
    phase = phases[position]['phase'] if position >= 0 and row['mono'] - stamps[position] <= 2 else 'phase_not_observed_within_2s'
    grouped[phase].append(row)

def describe(values):
    ordered = sorted(values)
    return dict(minimum=min(ordered), median=statistics.median(ordered),
                mean=statistics.mean(ordered), maximum=max(ordered),
                p95=ordered[min(len(ordered) - 1, int(len(ordered) * .95))])

def summary(records):
    return dict(samples=len(records), dimension=describe([row['dimension'] for row in records]),
                constraints=describe([row['constraints'] for row in records]),
                assembly_us=describe([row['assembly_s'] * 1e6 for row in records]),
                lcp_us=describe([row['lcp_s'] * 1e6 for row in records]),
                apply_us=describe([row['apply_s'] * 1e6 for row in records]),
                types=dict(sum((collections.Counter(row['types']) for row in records), collections.Counter())),
                dimensions_histogram=dict(collections.Counter(row['dimension'] for row in records)),
                sampled_fallback_count=sum(row['fallback'] for row in records))
result = dict(status='PASS_ACTUAL_SOLVER_GROUP_SAMPLES_NOT_COMPLETE_CENSUS',
              pid=runtime['pid'], tid=runtime['tid'], sample_period_calls=1024,
              original_library_and_candidate_mapped=True,
              total_recorded_samples=len(rows), task=summary(task),
              phases={name: summary(values) for name, values in grouped.items()},
              timing_scope='per_sample_wall_microseconds_in_one_actual_solver_call_not_CPU_causal_share',
              sampled_types_are_actual_solver_constraints_not_contact_messages=True,
              limitations=['periodic_group_order_can_bias_every_1024th_sampling',
                           'phase_labels_are_readonly_observer_nearest_preceding_within_2s',
                           'not_all_constraints_or_all_steps_sampled',
                           'not_comparable_component_timer_to_parent_without_same_instrumentation'])
with (ROOT / 'dart_solver_audit.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(dict(status=result['status'], samples=len(task), task=result['task'])), flush=True)
