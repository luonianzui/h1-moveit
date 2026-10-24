import copy
import hashlib
import itertools
import json
import math
import time

import numpy as np

from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner


PARENT = EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291'
source = PARENT / 'run/1_load_pickup_request.json'
request = json.loads(source.read_text())
request.update(sequential_contract=CONTRACT, hand='left', payload='left', original_runtime_holder_references={})
initial = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
request['positions'].update(initial)
for hand in ('left', 'right'):
    for number in (1, 2):
        request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
planner = SinglePlanner(request)
first = {name: request['positions'][name] for name in planner.names}
records, selected = [], None
began = time.monotonic()
for pitch, roll, elbow in itertools.product((.4, .8, 1.2), (.35, .65, .95), (.3, .8, 1.3)):
    target = dict(first, torso_joint=0.)
    for hand in ('left', 'right'):
        target.update({hand + '_shoulder_pitch_joint': pitch,
                       hand + '_shoulder_roll_joint': roll if hand == 'left' else -roll,
                       hand + '_shoulder_yaw_joint': 0., hand + '_elbow_joint': elbow,
                       hand + '_wrist_roll_joint': 0., hand + '_wrist_pitch_joint': 0., hand + '_wrist_yaw_joint': 0.})
    row = dict(pitch=pitch, roll=roll, elbow=elbow, joints=target, checked=0, valid=True)
    for torso in np.linspace(-math.radians(44), math.radians(44), 45):
        candidate = dict(target, torso_joint=float(torso))
        row['checked'] += 1
        if planner.planner.check(candidate)['collision']:
            row.update(valid=False, failed_torso_rad=float(torso))
            break
    if row['valid']:
        try:
            connector = planner.empty_connector(first, target)
            row.update(connector=connector, original_dense_checks=planner.checked)
            selected = row
        except (ValueError, RuntimeError) as error:
            row.update(valid=False, connector_error=str(error))
    records.append(row)
    print('PARK_SEARCH', pitch, roll, elbow, row['valid'], row['checked'], flush=True)
    if selected:
        break
with (EVIDENCE / 'safe_parking_probe.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_ONLY' if selected else 'NO_VERIFIED_PARK', selected=selected,
                   records=records, wall_s=time.monotonic() - began, request=request,
                   source_snapshot_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   original_request_not_fresh_current_scene=True), stream, indent=2)
assert selected, 'No parking profile yet qualifies'
