import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import dual_worker as worker
from elbow_posture import ELBOW_CONTRACT, posture_metrics
from minimum_turn import CONTRACT
from post_retreat_contract import expected_retreat_phases
from gripper_return_geometry import plan_post_retreat_gripper


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_minimum_turn_full_20261015_r285'


def update_predicted_poses(request, planner, joints, references):
    planner.planner.state.joint_positions = dict(request['positions'], **joints)
    planner.planner.state.update()
    request = copy.deepcopy(request)
    request['positions'].update(joints)
    for hand, payload in request['assignment'].items():
        body = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link') @ references[hand]
        request['poses']['box48_' + payload]['position'] = dict(zip('xyz', body[:3, 3].tolist()))
        request['poses']['box48_' + payload]['orientation'] = dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist()))
    return request


def probe(prefix):
    source = json.loads((PARENT / 'run' / (prefix + '_request.json')).read_text())
    selected = json.loads((ROOT / (prefix + '_elbow_revision2.json')).read_text())['result']
    assert selected['status'] == 'PASS'
    assert selected['elbow_posture_selection']['basket_max_from_down_deg'] < 45
    source.update(minimum_turn_contract=CONTRACT, elbow_posture_contract=ELBOW_CONTRACT,
                  prediction_only_not_current_actual_snapshot=True)
    joints = selected['stages'][-1]['samples'][-1]
    references = {}
    nominal = worker.nominal_references()
    for hand, flip in zip(('left', 'right'), selected['flips_deg']):
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
        references[hand] = nominal[hand] @ rotation
    planner = worker.PairedPlanner(source, references)
    planner.planner.state.joint_positions = dict(source['positions'], **joints)
    planner.planner.state.update()
    actual_body_predictions = {hand: worker.pose_matrix(source['poses']['box48_' + payload])
                               for hand, payload in source['assignment'].items()}
    references = {hand: np.linalg.inv(planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link')) @ body
                  for hand, body in actual_body_predictions.items()}
    source['positions'].update(joints)
    source['planning_references'] = {hand: body.tolist() for hand, body in references.items()}
    source['mode'] = 'pilot'
    pilot = worker.plan_transport(source)
    source = update_predicted_poses(source, planner, pilot['stages'][-1]['samples'][-1], references)
    source['mode'] = 'retrieval_transport' if source['retrieval'] else 'load_transport'
    carry_request = copy.deepcopy(source)
    carry = worker.plan_transport(source)
    selected_stage = carry['stages'][0 if source['retrieval'] else -1]
    metrics = {hand: max(posture_metrics(planner, point)[hand]['from_down_deg'] for point in selected_stage['samples'])
               for hand in ('left', 'right')}
    assert max(metrics.values()) < 45
    record = dict(status='PASS_PREDICTED_GEOMETRY_ONLY', mode=source['mode'], pilot=pilot, carry_request=carry_request,
                  carry=carry, basket_posture_max_from_down_deg=metrics, no_physical_reference_frozen=True)
    if not source['retrieval']:
        released = update_predicted_poses(source, planner, carry['stages'][-1]['samples'][-1], references)
        for payload in ('left', 'right'):
            released['poses']['box48_' + payload]['position']['z'] -= .045
        for hand in ('left', 'right'):
            for number in (1, 2):
                released['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
        released.update(mode='empty_return', original_runtime_holder_references={},
                        current_run_executed_loading_guides=carry['stages'], current_run_guide_identity=ROOT.name)
        retreat = worker.plan_empty_return(released)
        record['predicted_empty_retreat'] = retreat
        released['positions'].update(retreat['stages'][-2]['samples'][-1])
        released.update(mode='empty_gripper_after_retreat', accepted_gripper_efforts=dict(left=0., right=0.),
                        completed_empty_retreat_stages=list(expected_retreat_phases(carry['stages'])))
        neutral = json.loads((ROOT / 'inputs/plan.json').read_text())['initial_positions']
        record['predicted_gripper_stroke'] = plan_post_retreat_gripper(worker.PairedPlanner, released, neutral, ROOT.name)
        for hand in ('left', 'right'):
            for number in (1, 2):
                released['positions'][hand + f'_dex1_finger_{number}_joint'] = -.019
        released['mode'] = 'empty_return_after_gripper'
        record['predicted_final_empty_return'] = plan_post_retreat_gripper(worker.PairedPlanner, released, neutral, ROOT.name)
    return record


def main():
    records = []
    for prefix in ('1_load_pickup', '7_retrieval_pickup'):
        try:
            record = probe(prefix)
        except (ValueError, RuntimeError, AssertionError) as error:
            record = dict(status='FAIL_PREDICTED_GEOMETRY', error=str(error), prefix=prefix)
        with (ROOT / (prefix + '_elbow_chain.json')).open('x') as stream:
            json.dump(record, stream, indent=2)
        records.append(record)
        print(prefix, record['status'], record.get('basket_posture_max_from_down_deg'), record.get('error'), flush=True)
    with (ROOT / 'elbow_chain_audit.json').open('x') as stream:
        json.dump(dict(status='PASS_OFFLINE_NOT_PHYSICAL' if all(row['status'].startswith('PASS') for row in records) else 'FAIL',
                       no_actual_current_scene_or_grasp_or_full_execution_certificate=True,
                       modes=[dict(mode=row.get('mode'), status=row['status']) for row in records]), stream, indent=2)


if __name__ == '__main__':
    main()
