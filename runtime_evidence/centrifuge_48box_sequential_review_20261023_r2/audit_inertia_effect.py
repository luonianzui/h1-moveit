import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r355'
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r354'
PARENT_REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r1'

def read(folder, name):
    return json.loads((folder / name).read_text())

def symbols(folder):
    document = read(folder, 'native_impulse_audit.json')
    return [dict(name=row['name'], library=Path(row['library']).name,
                 flat_pct=row['flat_pct'], cumulative_pct=row['cumulative_pct'])
            for row in document['symbols'] if any(token in row['name'] for token in
                ('getArticulatedInertia(', 'updateVelocityChangeFD(', 'Skeleton::updateVelocityChange(', 'solveConstrainedGroup('))]

current = read(RUN, 'run/executor_wall_clock.json')
parent = read(PARENT, 'run/executor_wall_clock.json')
samples = read(ROOT, 'dart_solver_audit.json')
assert samples['inertia_counters_last']['inertia_queries'] > samples['inertia_counters_first']['inertia_queries']
assert samples['inertia_counters_last']['inertia_slot_hits'] > samples['inertia_counters_first']['inertia_slot_hits']
task_pass = read(RUN, 'run/result.json')['status'] == 'PASS'
result = dict(status='PASS_MEASURED_INERTIA_USE_NOT_CONTROLLED_WHOLE_TASK_CAUSAL_PROOF',
    task_pass=task_pass, current_wall_s=current['elapsed_wall_s'], parent_wall_s=parent['elapsed_wall_s'],
    saved_wall_s=parent['elapsed_wall_s']-current['elapsed_wall_s'] if task_pass else None,
    within_180_wall_s=task_pass and current['elapsed_wall_s'] <= 180.,
    component=read(RUN, 'dart_cache_dependencies.json')['inertia_qualification'],
    dirty_probe=read(RUN, 'dart_cache_dependencies.json')['dirty_qualification'],
    current_rtf=read(ROOT, 'world_stats_summary.json')['interval_ratio'],
    parent_rtf=read(PARENT_REVIEW, 'world_stats_summary.json')['interval_ratio'],
    actual_first_counters=samples['inertia_counters_first'], actual_last_counters=samples['inertia_counters_last'],
    parent_symbols=symbols(PARENT_REVIEW), current_symbols=symbols(ROOT),
    unchanged_action_and_physics_contract=True, inclusive_percentages_overlap_not_wall_time=True,
    limitations=['different_actual_contacts_plans_and_process_layouts', 'one_new_full_run_not_same_snapshot_AB',
                 'dirty_flag_preserved_no_cross_step_numeric_cache', 'component_speed_not_whole_task_speed'])
with (ROOT / 'inertia_effect.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, ensure_ascii=False))
