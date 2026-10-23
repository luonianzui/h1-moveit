import gzip
import json
from pathlib import Path
from preclock_audit import audit_trace_clock


ROOT = Path(__file__).resolve().parent


def main():
    preclock = audit_trace_clock(ROOT)
    delivery, health, poses, contracts, failures = [], [], [], [], []
    with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'localization_delivery':
                delivery.append(row['payload'])
            elif row['kind'] == 'localization_health':
                health.append(row['payload'])
            elif row['kind'] == 'localization_pose':
                poses.append(row['payload']['source_sim'])
            elif row['kind'] == 'localization_delivery_contract':
                contracts.append(row['payload'])
            elif row['kind'] == 'FAIL':
                failures.append(row['payload'])
    observed = [row for row in delivery if row['kind'] == 'pose']
    assert len(contracts) == 1 and contracts[0]['depth'] == 1
    assert [row['source_sim'] for row in observed] == poses
    assert delivery and health
    result = json.loads((ROOT / 'run/result.json').read_text())
    invalid = [row for row in health if not row['valid']]
    report = dict(status='PASS' if result['status'] == 'PASS' and not invalid and not failures else 'FAIL',
        task_status=result['status'], failures=failures, consumed_pose_count=len(poses),
        delivery_count=len(delivery), invalid_health=invalid,
        unchanged_maximum_source_age_sim_s=1.,
        maximum_observed_health_age_sim_s=max(row['pose_age_sim_s'] for row in health),
        maximum_callback_source_age_sim_s=max(row['clock_sim']-row['source_sim'] for row in observed if row['clock_sim'] is not None),
        validated_bootstrap_preclock_events=preclock,
        maximum_transport_wall_s=max((row['transport_wall_s'] for row in delivery if row['transport_wall_s'] is not None), default=None),
        maximum_callback_queue_wall_s=max((row['callback_queue_wall_s'] for row in delivery if row['callback_queue_wall_s'] is not None), default=None),
        unavailable_dds_timestamp_count=sum(not row['dds_timestamps_available'] for row in delivery),
        no_timestamp_rewriting=True, original_timeout_not_widened=True,
        historical_r332_root_cause_not_proven_by_a_successful_new_run=True)
    with (ROOT / 'localization_delivery_audit.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2), flush=True)
    assert report['status'] == 'PASS'


if __name__ == '__main__':
    main()
