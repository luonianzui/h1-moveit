import gzip
import json
from pathlib import Path
from empty_gap_policy import eligible

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r345'


def observe(root):
    schedules, endpoints = [], []
    with gzip.open(root / 'run/loading_trace.jsonl.gz', 'rb') as stream:
        for raw in stream:
            header, separator, remainder = raw.partition(b', "payload": ')
            row = json.loads(header + b'}') if separator else json.loads(raw)
            if row['kind'] not in ('command', 'button_arm_command', 'interval_endpoint_pass'):
                continue
            row = json.loads(raw)
            payload = row['payload']
            if row['kind'] == 'interval_endpoint_pass':
                endpoints.append(dict(phase=row['phase'], **payload))
            else:
                duration = payload['times'][-1] if row['kind'] == 'command' else payload['duration']
                schedules.append(dict(kind=row['kind'], phase=row['phase'],
                    observed_sim=row['sim'], start_sim=payload['start'], duration_sim_s=duration,
                    future_lead_sim_s=payload['start']-row['sim']))
    return schedules, endpoints


def main():
    before, unused = observe(PARENT)
    current, endpoints = observe(ROOT)
    scheduled_stop = [row for row in current if row['phase'] == 'machine_stop_press']
    ordinary = [row for row in current if row['phase'] != 'machine_stop_press']
    shortened = [row for row in ordinary if eligible(row['phase'], {})]
    unchanged = [row for row in ordinary if not eligible(row['phase'], {})]
    original_ordinary = [row for row in before if row['phase'] != 'machine_stop_press']
    assert len(scheduled_stop) == 1 and scheduled_stop[0]['future_lead_sim_s'] >= .15
    assert shortened and all(abs(row['future_lead_sim_s']-.16) < 1e-6 for row in shortened)
    assert unchanged and all(abs(row['future_lead_sim_s']-.3) < 1e-6 for row in unchanged)
    assert all(abs(row['future_lead_sim_s']-.3) < 1e-6 for row in original_ordinary)
    assert len(endpoints) == sum(row['kind'] == 'command' for row in current)
    assert all(row['overlap_enabled'] == eligible(row['phase'], {}) and row['measured_hold_sim_s'] > .05 and
               row['measured_stable_start_sim'] >= row['scheduled_end_sim'] and
               row['original_endpoint_limit_rad'] == .002 for row in endpoints)
    old_wall = json.loads((PARENT / 'run/executor_wall_clock.json').read_text())
    new_wall = json.loads((ROOT / 'run/executor_wall_clock.json').read_text())
    result = json.loads((ROOT / 'run/result.json').read_text())
    report = dict(status='PASS' if result['status'] == 'PASS' else 'FAIL',
        observed_short_lead_commands=len(shortened), unchanged_lead_commands=len(unchanged),
        original_commands=len(original_ordinary),
        original_lead_sim_s=.3, new_lead_sim_s=.16, schedules=current, endpoints=endpoints,
        actual_schedule_lead_budget_reduction_sim_s=len(shortened)*.14,
        original_phase_aligned_stop_schedule_preserved=scheduled_stop,
        gripper_acceptance_not_used_as_exact_publication_timestamp=True,
        observed_endpoint_hold_min_sim_s=min(row['measured_hold_sim_s'] for row in endpoints),
        parent_wall=old_wall, current_wall=new_wall,
        not_controlled_same_snapshot_wall_benchmark=True,
        ai_calls_between_actions=0,
        full_task_and_persistent_single_process_separately_audited=True)
    with (ROOT / 'interval_audit.json').open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({key:value for key,value in report.items() if key not in ('schedules','endpoints')}), flush=True)
    assert report['status'] == 'PASS'


if __name__ == '__main__':
    main()
