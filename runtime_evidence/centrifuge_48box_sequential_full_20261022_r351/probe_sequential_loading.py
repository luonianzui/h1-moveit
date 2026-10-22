import copy
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner, pose_matrix


def pose_dictionary(matrix):
    return dict(position=dict(zip('xyz', matrix[:3, 3].tolist())),
                orientation=dict(zip('xyzw', Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist())))


def probe(row, parking, table_request):
    hand = row['hand']
    inactive = 'right' if hand == 'left' else 'left'
    request = copy.deepcopy(table_request)
    request.update(sequential_contract=CONTRACT, hand=hand, payload=hand, original_runtime_holder_references={})
    request['positions'].update(row['joints'])
    request['positions'].update({name: value for name, value in parking.items() if name.startswith(inactive + '_')})
    for current in ('left', 'right'):
        for number in (1, 2):
            request['positions'][current + f'_dex1_finger_{number}_joint'] = -.01901 if current == hand else -.0199
    reference = np.array(row['request']['planning_references'][hand])
    planner = SinglePlanner(request, reference)
    first = {name: request['positions'][name] for name in planner.names}
    body = planner.wrist(first) @ reference
    request['poses']['box48_' + hand] = pose_dictionary(body)
    planner = SinglePlanner(request, reference)
    planner.attach(body, first)
    planner.check_edge(first, first, 'basket_release_endpoint')
    stages, seed = [], first
    targets = []
    overhead = body.copy()
    overhead[2, 3] = 1.15975
    targets.append(('reverse_lift_clear', overhead))
    outboard = overhead.copy()
    outboard[:2, 3] = [.44, .38 if hand == 'left' else -.38]
    outboard[:3, :3] = np.eye(3)
    targets.append(('reverse_outboard', outboard))
    table = pose_matrix(table_request['poses']['box48_' + hand])
    targets.append(('reverse_table_descent', table))
    for phase, target in targets:
        stage = planner.stage(body, target, seed, phase, table_end=phase == 'reverse_table_descent')
        stages.append(stage)
        body, seed = target, stage['samples'][-1]
        print('SINGLE_LOAD_STAGE', hand, phase, planner.checked, planner.elbow_metrics(seed)['from_down_deg'], flush=True)
    return dict(status='PASS_REVERSE_SINGLE_LOAD_GEOMETRY_ONLY', hand=hand, request=request,
                reference=reference.tolist(), stages=stages, checked=planner.checked,
                max_position_error_m=planner.maximum_position_error, max_angle_error_deg=planner.maximum_angle_error,
                actual_execution=False, basket_elbow=planner.elbow_metrics(first), table_elbow=planner.elbow_metrics(seed))


def main():
    parking = json.loads((EVIDENCE / 'safe_parking_probe_revision2.json').read_text())['selected']['joints']
    prior = EVIDENCE.parent / 'centrifuge_48box_redline_probe_20261017_r1'
    rows = [row for row in json.loads((prior / 'sequential_endpoints.json').read_text()) if row['case'] == '4_load_alignment']
    table_request = json.loads((EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291/run/1_load_pickup_request.json').read_text())
    results = []
    for row in rows:
        began = time.monotonic()
        try:
            result = probe(row, parking, table_request)
        except (ValueError, RuntimeError, AssertionError) as error:
            result = dict(status='REJECTED_OFFLINE_SINGLE_LOAD', hand=row['hand'], error=str(error))
        result['wall_s'] = time.monotonic() - began
        results.append(result)
        with (EVIDENCE / ('sequential_loading_' + row['hand'] + '.json')).open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print('SINGLE_LOAD_RESULT', row['hand'], result['status'], result.get('error'), flush=True)
    with (EVIDENCE / 'sequential_loading_summary.json').open('x') as stream:
        json.dump(results, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
