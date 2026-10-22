import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r351'


def read(path):
    return json.loads(path.read_text())


world = [json.loads(line) for line in (ROOT / 'world_stats.jsonl').read_text().splitlines()]
assert not any(row['status'] == 'READ_ONLY_SAMPLE' for row in world)
failure = (ROOT / 'audit_dart_samples_stdout.log').read_text()
assert 'assert len(task) >= 100' in failure and 'AssertionError' in failure
result = dict(status='INCOMPLETE_TASK_INSUFFICIENT_WORLD_AND_SOLVER_SAMPLES',
    task_status=read(RUN / 'run/result.json')['status'],
    failed_prefix_wall_s=read(RUN / 'run/executor_wall_clock.json')['elapsed_wall_s'],
    complete_task_comparable=False, full_task_speedup=None, sampled_world_rtf=None,
    world_observations=world, solver_original_audit_retained=True,
    solver_minimum_100_samples_not_relaxed=True,
    cpu_profile=read(ROOT / 'native_profile_audit.json')['metadata'],
    source_identity_sha256=hashlib.sha256((RUN / 'identity.json').read_bytes()).hexdigest(),
    planner_requests=read(RUN / 'run/planner_lifecycle.json')['requests'],
    floor_boundary_live_fix_not_reached=True)
assert result['task_status'] == 'FAIL' and result['planner_requests'] == 0
with (ROOT / 'performance_limits.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
