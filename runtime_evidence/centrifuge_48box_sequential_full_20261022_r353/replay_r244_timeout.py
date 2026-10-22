import gzip
import hashlib
import json
from pathlib import Path

from motion_clock_watch import SimulationProgressWatch


EVIDENCE = Path(__file__).resolve().parent
PARENT = EVIDENCE.parent / 'centrifuge_48box_camera_retry_20261013_r244'


def main():
    metadata = json.loads((PARENT / 'run/trace_metadata.json').read_text())
    assert metadata['status'] == 'CLOSED'
    watcher = None
    checks = 0
    command = None
    failure = None
    hasher = hashlib.sha256()
    with gzip.open(PARENT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            hasher.update(line.encode())
            event = json.loads(line)
            if event['kind'] == 'command' and event['phase'] == 'paired_load_pilot_lift':
                assert watcher is None
                command = event
                watcher = SimulationProgressWatch(event['payload']['times'][-1], event['sim'], event['wall'])
            if watcher is not None and event['phase'] == 'paired_load_pilot_lift':
                watcher.observe(event['sim'], event['wall'])
                checks += 1
            if event['kind'] == 'FAIL':
                failure = event
    assert hasher.hexdigest() == metadata['utf8_sha256']
    assert command is not None and failure is not None and checks > 100
    end = command['payload']['start'] + command['payload']['times'][-1] + .15
    assert failure['sim'] < end
    result = dict(status='PREFIX_WALL_ALLOWANCE_PASS_NOT_PHYSICAL_OR_COMPLETE_PASS', checks=checks,
                  original_trace_sha256=hasher.hexdigest(), finite_candidate_budget_wall_s=watcher.budget_wall_s,
                  actual_failure_sim=failure['sim'], original_command_finish_sim=end,
                  actual_command_samples_rates_timestamps_not_changed=True,
                  actual_grip_tracking_collision_freshness_and_endpoint_gates_not_relaxed=True,
                  no_future_state_or_complete_task_pass_predicted=True)
    with (EVIDENCE / 'r244_timeout_replay.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
