from collections import deque
import hashlib
import json
from pathlib import Path
import numpy as np
from trace_io import open_trace
from execute_loading import pose_matrix

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r350'
assert hashlib.sha256((RUN / 'identity.json').read_bytes()).hexdigest() == '7d93071508f913138f12ffa599fdbb13821ea5b188f9843936bb9df8c6d69123'
recent = deque(maxlen=12)
claim = joints = goal = fault = None
after = []
with open_trace(RUN) as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'sequential_preload_claim':
            claim = row
        elif row['kind'] == 'actual_full_joint_frame':
            joints = row
        elif row['kind'] == 'gripper_goal_accepted':
            goal = row
        elif row['kind'] == 'contact' and row['payload']['side'] == 'right':
            if fault is None:
                recent.append(row)
            else:
                after.append(row)
                if row['payload']['stamp'] > fault['sim'] + .003:
                    break
        elif row['kind'] == 'sequential_remaining_basket_fault':
            fault = row
            captured = dict(fault=fault, claim=claim, joints=joints, goal=goal, contacts_before=list(recent))
assert fault is not None and after
captured.update(contacts_after=after, source=str(RUN),
               after_fault_is_stop_and_hold_not_uninterrupted_success=True)
with (ROOT / 'r350_preload_fixture.json').open('x') as stream:
    json.dump(captured, stream, indent=2)
print('ACTUAL_FAULT_FIXTURE', len(recent), len(after), flush=True)
print('CLAIM', claim['payload'], 'GOAL', goal['payload'], 'JOINT', joints['payload']['source_sim'])
inverse = np.linalg.inv(pose_matrix(fault['payload']['poses']['box48_right']))
for pair in fault['payload']['contacts']:
    local = np.asarray(pair['positions']) @ inverse[:3,:3].T + inverse[:3,3]
    print('LOCAL', dict(minimum=local.min(axis=0).tolist(), maximum=local.max(axis=0).tolist(),
        mean=local.mean(axis=0).tolist(), force=pair['force']))
for event in list(recent)[-3:] + after[:8]:
    print(event['payload']['stamp'], [(pair['force'], 'bucket_' in pair['first'] + pair['second'])
          for pair in event['payload']['pairs']], flush=True)
