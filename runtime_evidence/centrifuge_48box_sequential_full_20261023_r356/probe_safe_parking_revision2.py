import copy
import itertools
import json
import time

import numpy as np

from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner


def main():
    source = EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291/run/1_load_pickup_request.json'
    request = json.loads(source.read_text())
    request.update(sequential_contract=CONTRACT, hand='left', payload='left', original_runtime_holder_references={})
    neutral = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
    request['positions'].update({name: neutral[name] for name in neutral if name == 'torso_joint' or 'shoulder' in name or 'elbow' in name or 'wrist' in name})
    for hand in ('left', 'right'):
        for number in (1, 2):
            request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
    planner = SinglePlanner(request)
    first = {name: request['positions'][name] for name in planner.names}
    assert not planner.planner.check(first)['collision']
    records, selected = [], None
    began = time.monotonic()
    for pitch, roll, elbow in itertools.product((0., .4, .8, 1.2), (.25, .65, .95), (.3, .6, 1.3)):
        target = dict(first, torso_joint=0.)
        for hand in ('left', 'right'):
            target.update({hand + '_shoulder_pitch_joint': pitch,
                           hand + '_shoulder_roll_joint': roll if hand == 'left' else -roll,
                           hand + '_shoulder_yaw_joint': 0., hand + '_elbow_joint': elbow,
                           hand + '_wrist_roll_joint': 0., hand + '_wrist_pitch_joint': 0., hand + '_wrist_yaw_joint': 0.})
        row = dict(pitch=pitch, roll=roll, elbow=elbow, joints=target, checked=0, valid=True)
        for torso in np.linspace(-.65, .65, 33):
            candidate = dict(target, torso_joint=float(torso))
            row['checked'] += 1
            if planner.planner.check(candidate)['collision']:
                row.update(valid=False, failed_torso_rad=float(torso))
                break
        if row['valid']:
            try:
                planner.check_edge(dict(target, torso_joint=-.65), dict(target, torso_joint=.65), 'park_torso_sweep')
                row.update(connector=planner.empty_connector(first, target), original_dense_checks=planner.checked)
                selected = copy.deepcopy(row)
            except (ValueError, RuntimeError) as error:
                row.update(valid=False, connector_error=str(error))
        records.append(row)
        print('PARK_REVISION2', pitch, roll, elbow, row['valid'], row['checked'], flush=True)
        if selected:
            break
    result = dict(status='PASS_OFFLINE_ONLY' if selected else 'NO_VERIFIED_PARK', selected=selected,
                  records=records, wall_s=time.monotonic() - began, request=request,
                  selected_torso_window_rad=[-.65, .65], original_torso_limit_not_relaxed=True,
                  actual_execution=False, original_request_not_fresh_current_scene=True)
    with (EVIDENCE / 'safe_parking_probe_revision2.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    assert selected, 'No parking profile yet qualifies'


if __name__ == '__main__':
    main()
