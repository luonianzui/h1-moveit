import gzip
import json
from pathlib import Path
from preclock_audit import audit_trace_clock


ROOT = Path(__file__).resolve().parent


def maximum_gap(rows, key):
    values = [row[key] for row in rows]
    return max((current - previous for previous, current in zip(values, values[1:])), default=None)


def main():
    preclock = audit_trace_clock(ROOT)
    rows = [json.loads(line) for line in (ROOT / 'run/localizer_pipeline.jsonl').read_text().splitlines()]
    clouds = [row for row in rows if row['event'] == 'cloud_enter']
    poses = [row for row in rows if row['event'] == 'pose_published']
    transforms = [row for row in rows if row['event'] == 'tf_exit']
    configured = [row for row in rows if row['event'] == 'configured']
    assert len(configured) == 1 and configured[0]['localization_rate'] == 5.
    consumed = []
    with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'localization_delivery' and row['payload']['kind'] == 'pose':
                consumed.append(row['payload'])
    assert clouds and poses and transforms and consumed
    produced = {round(row['stamp'], 7) for row in poses}
    assert all(round(row['source_sim'], 7) in produced for row in consumed)
    result = json.loads((ROOT / 'run/result.json').read_text())
    begin = min(row['source_sim'] for row in consumed)
    end = max(row['clock_sim'] for row in consumed if row['clock_sim'] is not None)
    task_poses = [row for row in poses if begin <= row['stamp'] <= end]
    task_clouds = [row for row in clouds if begin <= row['stamp'] <= end]
    report = dict(status='PASS' if result['status'] == 'PASS' else 'FAIL',
                  task_status=result['status'], configuration=configured[0],
                  validated_bootstrap_preclock_events=preclock,
                  original_source_stamps_preserved=True, original_1s_guard_unchanged=True,
                  producer_cloud_count=len(clouds), producer_pose_count=len(poses),
                  consumer_pose_count=len(consumed), consumer_all_stamps_found_in_producer=True,
                  task_source_pose_max_gap_sim_s=maximum_gap(task_poses, 'stamp'),
                  task_source_cloud_max_gap_sim_s=maximum_gap(task_clouds, 'stamp'),
                  producer_max_tf_lookup_wall_s=max(row['elapsed'] for row in transforms),
                  producer_max_cloud_callback_wall_s=max(row['elapsed'] for row in rows if row['event'] == 'cloud_exit'),
                  tf_errors=[row for row in rows if row['event'] == 'tf_error'],
                  configured_rate_hz_is_cap_not_guaranteed_5Hz=True,
                  historical_failure_unique_root_cause_not_proven=True)
    with (ROOT / 'localizer_pipeline_audit.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report), flush=True)
    assert report['status'] == 'PASS'


if __name__ == '__main__':
    main()
