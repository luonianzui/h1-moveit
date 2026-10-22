import gzip
import json


def validate_preclock_events(rows):
    timed = ready = moved = False
    early = []
    for number, row in enumerate(rows, 1):
        if row.get('sim') is not None:
            timed = True
        if row['kind'] == 'fresh_startup_window_pass':
            ready = True
        if row['kind'] in ('command', 'button_arm_command', 'gripper_goal_accepted'):
            assert ready and timed, 'Motion before fresh clock'
            moved = True
        if row['kind'] == 'localization_delivery' and row['payload']['clock_sim'] is None:
            assert not timed and not ready and not moved, 'Missing clock after initialized state'
            assert row['phase'] == 'machine_preparation' and row['sim'] is None
            early.append(dict(row=number, phase=row['phase'], payload=row['payload']))
    assert timed and ready and moved
    return early


def audit_trace_clock(root):
    with gzip.open(root / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        return validate_preclock_events(json.loads(line) for line in stream)
