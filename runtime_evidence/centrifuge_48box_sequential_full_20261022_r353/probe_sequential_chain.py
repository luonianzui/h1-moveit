import copy
import json
import sys
import time

import numpy as np

from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner, pose_matrix
from sequential_planning import park, pickup, pilot, load_transport, retrieve_transport, retreat
from probe_sequential_loading import pose_dictionary


PARENT = EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291'


def advance(request, planned, moved=False):
    for stage in planned['stages']:
        request['positions'].update(stage['samples'][-1])
        if moved and 'last_body' in stage:
            request['poses']['box48_' + request['payload']] = pose_dictionary(np.array(stage['last_body']))


def closed_fingers(request, value):
    for number in (1, 2):
        request['positions'][request['hand'] + f'_dex1_finger_{number}_joint'] = value


def main():
    request = json.loads((PARENT / 'run/1_load_pickup_request.json').read_text())
    request.update(sequential_contract=CONTRACT, hand='left', payload='left', original_runtime_holder_references={})
    drop_fixture = json.loads((PARENT / 'run/6_empty_gripper_after_retreat_request.json').read_text())
    for hand in ('left', 'right'):
        for number in (1, 2):
            request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
    began = time.monotonic()
    output = dict(status='REJECTED_OFFLINE_SEQUENTIAL_CHAIN', stages=[],
                  actual_execution=False, real_grip_drop_machine_spin_not_simulated=True,
                  machine_pose_handover='ORIGINAL_R291_POST_SPIN_FIXTURE_NOT_NEW_MACHINE_RESULT')

    def accept(name, planned):
        output['stages'].append(dict(name=name, result=planned))
        print('SEQUENTIAL_CHAIN_STAGE', name, planned['status'], planned['checked_samples'], flush=True)

    try:
        planned = park(request)
        accept('initial_park', planned)
        advance(request, planned)
        for hand in ('left', 'right'):
            request.update(hand=hand, payload=hand)
            planned = pickup(request)
            accept(hand + '_load_pickup', planned)
            advance(request, planned)
            reference = planned['planning_reference']
            closed_fingers(request, -.01901)
            planned = pilot(request, reference)
            accept(hand + '_load_pilot', planned)
            advance(request, planned, moved=True)
            planned = load_transport(request, reference)
            accept(hand + '_load_transport', planned)
            advance(request, planned, moved=True)
            closed_fingers(request, -.0199)
            request['poses']['box48_' + hand] = copy.deepcopy(drop_fixture['poses']['box48_' + hand])
            planned = retreat(request)
            accept(hand + '_load_retreat', planned)
            advance(request, planned)
        post_spin = json.loads((PARENT / 'run/8_retrieval_pickup_request.json').read_text())
        request['poses'] = copy.deepcopy(post_spin['poses'])
        payloads = sorted(('left', 'right'), key=lambda payload: pose_matrix(request['poses']['box48_' + payload])[1, 3])
        assignment = dict(left=payloads[1], right=payloads[0])
        output['post_spin_assignment'] = assignment
        for hand in ('left', 'right'):
            request.update(hand=hand, payload=assignment[hand])
            planned = pickup(request, retrieval=True)
            accept(hand + '_retrieval_pickup', planned)
            advance(request, planned)
            reference = planned['planning_reference']
            closed_fingers(request, -.01901)
            planned = pilot(request, reference, retrieval=True)
            accept(hand + '_retrieval_pilot', planned)
            advance(request, planned, moved=True)
            planned = retrieve_transport(request, reference)
            accept(hand + '_retrieval_transport', planned)
            advance(request, planned, moved=True)
            closed_fingers(request, -.0199)
            planned = retreat(request)
            accept(hand + '_retrieval_retreat', planned)
            advance(request, planned)
        output.update(status='PASS_OFFLINE_SEQUENTIAL_CHAIN_GEOMETRY_ONLY', final_predicted_request=request,
                      total_dense_checks=sum(item['result']['checked_samples'] for item in output['stages']))
    except (RuntimeError, ValueError, AssertionError) as error:
        output.update(error=str(error), failed_request=copy.deepcopy(request))
        print('SEQUENTIAL_CHAIN_REJECTED', str(error), flush=True)
    output['wall_s'] = time.monotonic() - began
    destination = EVIDENCE / (sys.argv[1] if len(sys.argv) == 2 else 'sequential_chain.json')
    if destination.parent != EVIDENCE or destination.suffix != '.json':
        raise ValueError('Exclusive local evidence JSON path required')
    with destination.open('x') as stream:
        json.dump(output, stream, indent=2, allow_nan=False)
    print('SEQUENTIAL_CHAIN_RESULT', output['status'], output.get('total_dense_checks'), flush=True)


if __name__ == '__main__':
    main()
