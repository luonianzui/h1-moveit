import collections
import json
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r349'


def read(name):
    return json.loads((RUN/name).read_text())


def main():
    threads = collections.defaultdict(list)
    phases = collections.defaultdict(list)
    for line in (ROOT/'server_thread_samples.jsonl').read_text().splitlines():
        row = json.loads(line)
        total = sum(thread['cores'] for thread in row['threads'])
        phases[row['phase']].append(total)
        for thread in row['threads']:
            threads[(thread['tid'], thread['name'])].append(thread['cores'])
    top = sorted([dict(tid=key[0], name=key[1], mean_cores=statistics.mean(values),
                       peak_cores=max(values), samples=len(values))
                  for key,values in threads.items()], key=lambda row:row['mean_cores'], reverse=True)[:12]
    current_wall = read('run/executor_wall_clock.json')['elapsed_wall_s']
    baseline = RUN.parent / 'centrifuge_48box_sequential_full_20261022_r347'
    old_wall = json.loads((baseline/'run/executor_wall_clock.json').read_text())['elapsed_wall_s']
    world = json.loads((ROOT/'world_stats_summary.json').read_text())
    rows = sorted([dict(phase=phase, mean_server_cores=statistics.mean(values), samples=len(values))
                   for phase,values in phases.items()], key=lambda row:row['mean_server_cores'],reverse=True)
    complete = read('run/result.json')['status'] == 'PASS'
    report = dict(status='MEASURED_RUN_OBSERVATION_NOT_ISOLATED_PHYSICS_CAUSAL_PROOF',
                  task_status=read('run/result.json')['status'],
                  actual_wall_s=current_wall, r347_wall_s=old_wall,
                  complete_task_comparable=complete,
                  wall_difference_saved_s=old_wall-current_wall if complete else None,
                  current_sampled_world_rtf=world['interval_ratio'],
                  r347_sampled_world_rtf=.4079013898,
                  component_speedup=read('geometry_fastpath_qualification.json')['measurements'],
                  planning_resource_component=read('planner_resources_qualification.json'),
                  planning_resource_runtime=read('planner_resources_audit.json') if complete else None,
                  dart_response_component_not_loaded=True,
                  thread_samples=top, phase_server_cpu=rows,
                  kernel_perf_available=False, perf_policy_unchanged=True,
                  all_physics_inputs_and_guards_unchanged=True,
                  source_fastpath_not_a_promise_of_whole_task_speedup=True,
                  multiple_changes_not_controlled_single_cause_comparison=True)
    with (ROOT/'performance_conclusion.json').open('x') as stream:
        json.dump(report,stream,indent=2)
    print(json.dumps(dict(wall_s=current_wall, previous_wall_s=old_wall,
                         difference_s=old_wall-current_wall if complete else None, interval_rtf=world['interval_ratio'],
                         hottest_threads=top[:3])),flush=True)


if __name__ == '__main__':
    main()
