import copy
import json
import time

import numpy as np

from sequential_geometry import CONTRACT, EVIDENCE, SinglePlanner
from probe_sequential_loading import pose_dictionary


def main():
    parent = EVIDENCE.parent / 'centrifuge_48box_elbow_full_20261016_r291/run'
    request = json.loads((parent / '1_load_pickup_request.json').read_text())
    drop = json.loads((parent / '6_empty_gripper_after_retreat_request.json').read_text())
    spun = json.loads((parent / '8_retrieval_pickup_request.json').read_text())
    chain = json.loads((EVIDENCE / 'sequential_chain_revision2.json').read_text())
    assert chain['status'] == 'PASS_OFFLINE_SEQUENTIAL_CHAIN_GEOMETRY_ONLY'
    request.update(sequential_contract=CONTRACT, hand='left', payload='left', original_runtime_holder_references={})
    for hand in ('left', 'right'):
        for number in (1, 2):
            request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
    began = time.monotonic()
    records = []
    for row in chain['stages']:
        name, planned = row['name'], row['result']
        if name != 'initial_park':
            hand = name.split('_')[0]
            request.update(hand=hand, payload=chain['post_spin_assignment'][hand] if '_retrieval_' in name else hand)
        if name == 'left_retrieval_pickup':
            request['poses'] = copy.deepcopy(spun['poses'])
        moved = name.endswith('_pilot') or name.endswith('_transport')
        if name.endswith('_pilot'):
            for number in (1, 2):
                request['positions'][request['hand'] + f'_dex1_finger_{number}_joint'] = -.01901
        if name.endswith('_retreat'):
            for number in (1, 2):
                request['positions'][request['hand'] + f'_dex1_finger_{number}_joint'] = -.0199
            if '_load_' in name:
                request['poses']['box48_' + request['payload']] = copy.deepcopy(drop['poses']['box48_' + request['payload']])
        planner = SinglePlanner(request, planned['planning_reference'])
        first = {key: request['positions'][key] for key in planner.names}
        if moved:
            planner.attach(np.array(planned['stages'][0]['first_body']), first)
        for stage_index, stage in enumerate(planned['stages']):
            support = None
            if stage_index == 0:
                if name.endswith('_load_pilot') or name.endswith('_load_transport'):
                    support = 'table'
                elif name.endswith('_retrieval_pilot') or name.endswith('_retrieval_transport'):
                    support = 'basket'
            for index, target in enumerate(stage['samples'][1:]):
                permitted = support if index <= 1 else None
                if permitted == 'table' and index == 1:
                    permitted = 'table_start_only'
                planner.check_edge(first, target, 'independent_final_' + stage['phase'],
                    table_endpoint=stage['phase'] == 'retrieve_setdown' and index == len(stage['samples']) - 2,
                    initial_support=permitted)
                first = target
            if moved:
                request['poses']['box48_' + request['payload']] = pose_dictionary(np.array(stage['last_body']))
            request['positions'].update(first)
        records.append(dict(name=name, final_path_dense_samples=planner.checked, original_step_rad=.002))
        print('FINAL_SEQUENTIAL_DENSE', name, planner.checked, flush=True)
    result = dict(status='PASS_OFFLINE_FINAL_COMMAND_PATH_FCL_NOT_PHYSICAL', stages=records,
                  final_path_dense_samples=sum(row['final_path_dense_samples'] for row in records),
                  elapsed_wall_s=time.monotonic() - began, all_final_paths_zero_collision=True,
                  predictive_contact_and_old_post_spin_fixture_not_new_physical_evidence=True)
    with (EVIDENCE / 'sequential_final_path_review.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('FINAL_SEQUENTIAL_DENSE_PASS', result['final_path_dense_samples'], flush=True)


if __name__ == '__main__':
    main()
