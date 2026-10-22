import json
from pathlib import Path

from trace_io import open_trace


EVIDENCE = Path(__file__).resolve().parent


def measured_duration(first_sim, final_sim, complete_physics, independent_grip):
    if first_sim is None or final_sim is None or final_sim <= first_sim:
        return dict(status='FAIL', complete=False, duration_sim_s=None, under_three_minutes_sim=False)
    duration = final_sim - first_sim
    complete = complete_physics and independent_grip
    return dict(status='PASS' if complete and duration <= 180 else 'FAIL', complete=complete,
                duration_sim_s=duration, under_three_minutes_sim=complete and duration <= 180)


def main():
    first = None
    final = None
    milestones = []
    returns = set()
    failures = []
    with open_trace(EVIDENCE) as stream:
        for line in stream:
            event = json.loads(line)
            if first is None and event['sim'] is not None:
                first = event
            if event['kind'] in ('LOADING_PASS', 'MACHINE_CYCLE_PASS', 'table_return_pass', 'PASS'):
                milestones.append(dict(kind=event['kind'], phase=event['phase'], sim=event['sim'], wall=event['wall']))
            if event['kind'] == 'table_return_pass':
                returns.add(event['payload']['payload'])
            if event['kind'] == 'PASS':
                assert final is None
                final = event
            if event['kind'].endswith('_fault') or event['kind'].endswith('_fault_sample') or event['kind'] == 'FAIL':
                failures.append(dict(kind=event['kind'], sim=event['sim']))
    physics = json.loads((EVIDENCE / 'cycle_audit.json').read_text())
    grip = json.loads((EVIDENCE / 'independent_dual_grip.json').read_text())
    result = measured_duration(first['sim'] if first else None, final['sim'] if final else None,
                               physics['status'] == 'PASS' and not failures and returns == {'left', 'right'},
                               grip['status'] == 'PASS')
    result.update(scope='first_nonempty_actual_sim_to_complete_actual_PASS', milestones=milestones,
                  first_sim=first['sim'] if first else None, final_sim=final['sim'] if final else None,
                  duration_wall_s=final['wall'] - first['wall'] if final and first else None,
                  independent_physics_required=True, two_real_holder_audit_required=True,
                  planning_budget_or_video_playback_not_used=True, failures=failures)
    with (EVIDENCE / 'complete_speed_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    assert result['status'] == 'PASS'


if __name__ == '__main__':
    main()
