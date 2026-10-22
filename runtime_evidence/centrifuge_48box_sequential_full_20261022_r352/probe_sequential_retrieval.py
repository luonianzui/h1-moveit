import copy
import itertools
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation

from extraction_clearance import bucket_rim_bounds
from sequential_geometry import CONTRACT, EVIDENCE, SIZE, SinglePlanner, pose_matrix


def probe(row, parking):
    hand = row['hand']
    inactive = 'right' if hand == 'left' else 'left'
    request = copy.deepcopy(row['request'])
    payload = request['assignment'][hand]
    request.update(sequential_contract=CONTRACT, hand=hand, payload=payload, original_runtime_holder_references={})
    request['positions'].update(row['joints'])
    request['positions'].update({name: value for name, value in parking.items() if name.startswith(inactive + '_')})
    for current in ('left', 'right'):
        for number in (1, 2):
            request['positions'][current + f'_dex1_finger_{number}_joint'] = -.01901 if current == hand else -.0199
    reference = np.array(request['planning_references'][hand])
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = planner.wrist(seed) @ reference
    planner.attach(body, seed)
    planner.check_edge(seed, seed, 'retrieval_pilot_endpoint', initial_support='basket')
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + payload])
    rim = bucket_rim_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')[payload]
    vertices = np.array(list(itertools.product((-1., 1.), repeat=3))) * np.array(SIZE) / 2
    local = np.linalg.inv(bucket) @ body
    bottom = (vertices @ local[:3, :3].T + local[:3, 3])[:, 2].min()
    distance = max(0., float(rim + .006 - bottom))
    axis_exit = body.copy()
    axis_exit[:3, 3] += -bucket[:3, 0] * local[0, 3] - bucket[:3, 1] * local[1, 3] + bucket[:3, 2] * distance
    high = body.copy()
    high[2, 3] = 1.15975
    outboard = high.copy()
    outboard[1, 3] = .32 if hand == 'left' else -.32
    table = outboard.copy()
    yaw = np.pi if abs(np.arctan2(body[1, 0], body[0, 0])) > np.pi / 2 else 0.
    table[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    table[:3, 3] = [.44, .38 if hand == 'left' else -.38, 1.14475]
    down = table.copy()
    down[2, 3] = .84475
    stages = []
    for phase, target, count in [('axis_exit', axis_exit, 41), ('lift', high, 41), ('outboard', outboard, 81),
                                 ('transfer', table, 121), ('setdown', down, 121)]:
        stage = planner.stage(body, target, seed, phase, count=count, table_end=phase == 'setdown',
                              support_start='basket' if phase == 'axis_exit' else None)
        stages.append(stage)
        body, seed = target, stage['samples'][-1]
        print('SINGLE_RETRIEVAL_STAGE', hand, phase, planner.checked, planner.elbow_metrics(seed)['from_down_deg'], flush=True)
    return dict(status='PASS_OFFLINE_SINGLE_RETRIEVAL_ONLY', hand=hand, payload=payload, request=request, stages=stages,
                checked=planner.checked, max_position_error_m=planner.maximum_position_error,
                max_angle_error_deg=planner.maximum_angle_error, table_yaw_rad=float(yaw), actual_execution=False)


def main():
    parking = json.loads((EVIDENCE / 'safe_parking_probe_revision2.json').read_text())['selected']['joints']
    rows = json.loads((EVIDENCE.parent / 'centrifuge_48box_redline_probe_20261017_r1/sequential_endpoints.json').read_text())
    results = []
    for row in rows:
        if row['case'] != '10_retrieval_transport' or row['endpoint'] != 0:
            continue
        began = time.monotonic()
        try:
            result = probe(row, parking)
        except (RuntimeError, ValueError, AssertionError) as error:
            result = dict(hand=row['hand'], status='REJECTED_OFFLINE_SINGLE_RETRIEVAL', error=str(error))
        result['wall_s'] = time.monotonic() - began
        results.append(result)
        with (EVIDENCE / ('sequential_retrieval_' + row['hand'] + '.json')).open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print('SINGLE_RETRIEVAL_RESULT', row['hand'], result['status'], result.get('error'), flush=True)
    with (EVIDENCE / 'sequential_retrieval_summary.json').open('x') as stream:
        json.dump(results, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
