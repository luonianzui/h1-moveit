import collections
import gzip
import json
from pathlib import Path
import numpy as np

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r351'
commands = []
joints = collections.deque(maxlen=10)
samples = collections.deque(maxlen=10)
events = []
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'button_arm_command':
            commands.append(row)
        elif row['kind'] == 'actual_full_joint_frame':
            joints.append(row)
        elif row['kind'] == 'fast_button_tracking_sample':
            samples.append(row)
        elif row['kind'] == 'speed_button_tracking_fault':
            events.append(dict(fault=row, commands=list(commands), joints=list(joints), samples=list(samples)))
assert len(events) == 1
event = events[0]
command = event['commands'][-1]['payload']
fault = event['fault']
result = dict(status='ACTUAL_BUTTON_TRACKING_FAILURE_NOT_PLANNER_OR_FLOOR_FIX_VALIDATION', evidence=event,
    fault_sim=fault['sim'], fault_joint_sim=fault['payload']['joint_sim'], fault_wall=fault['wall'],
    command_start_sim=command['start'], command_duration_sim=command['duration'],
    final_command_points=len(command['points']), errors=fault['payload']['errors_rad'],
    planner_requests=json.loads((ROOT / 'run/planner_lifecycle.json').read_text())['requests'],
    duplicate_clock_after_failure_pause_sim=44.8645,
    duplicate_clock_is_after_primary_tracking_failure=True)
with (REVIEW / 'opening_tracking_diagnostic.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps({key:value for key,value in result.items() if key != 'evidence'}, indent=2))
print('COMMANDS', [(row['phase'],row['sim'],row['payload']['start'],row['payload']['duration']) for row in commands])
print('JOINT_TIMES', [(row['sim'],row['payload']['source_sim']) for row in joints])
