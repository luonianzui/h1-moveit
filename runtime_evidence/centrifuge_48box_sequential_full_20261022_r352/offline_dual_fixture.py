import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

import dual_worker
from alignment_scene import pose_matrix


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def main():
    folder = EVIDENCE / 'paired_fixtures'
    folder.mkdir()
    original = ROOT / 'runtime_evidence/centrifuge_48box_dual_sync_20261012_r227/dual_pickup_request.json'
    request = json.loads(original.read_text())
    request['poses']['centrifuge']['position']['x'] -= .08
    request.update(mode='load_pickup', assignment=dict(left='left', right='right'), retrieval=False)
    request['positions'] = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
    requests = [('load_pickup', request)]
    actual = ROOT / 'runtime_evidence/centrifuge_48box_alignment_cap_20261013_r236/run/right_retrieval_pickup_request.json'
    if not actual.is_file():
        matches = list((actual.parent).glob('*retrieval*pickup*request*'))
        assert len(matches) == 1, matches
        actual = matches[0]
    stopped = json.loads(actual.read_text())
    stopped['poses']['centrifuge']['position']['x'] -= .08
    for payload in ('left', 'right'):
        stopped['poses']['box48_' + payload]['position']['x'] -= .08
    order = sorted(('left', 'right'), key=lambda payload: stopped['poses']['box48_' + payload]['position'].get('y', 0))
    stopped.update(mode='retrieval_pickup', assignment=dict(right=order[0], left=order[1]), retrieval=True)
    requests.append(('retrieval_pickup', stopped))
    results = []
    state = RobotState(RobotModel(str(EVIDENCE / 'inputs/robot_dual.urdf'), str(ROOT / 'src/h1_2_moveit_config/config/h1_2.srdf')))
    for label, request in requests:
        with (folder / (label + '_request.json')).open('x') as stream:
            json.dump(request, stream, indent=2)
        try:
            pickup = dual_worker.plan_pickup(request)
            with (folder / (label + '_plan.json')).open('x') as stream:
                json.dump(pickup, stream, indent=2)
            positions = dict(request['positions'], **pickup['stages'][-1]['samples'][-1])
            state.joint_positions = positions
            state.update()
            references = {hand: (np.linalg.inv(state.get_global_link_transform(hand + '_wrist_yaw_link')) @
                          pose_matrix(request['poses']['box48_' + payload])).tolist()
                          for hand, payload in request['assignment'].items()}
            pilot_request = dict(request, mode='pilot', positions=positions, planning_references=references)
            pilot = dual_worker.plan_transport(pilot_request)
            with (folder / (label + '_pilot.json')).open('x') as stream:
                json.dump(pilot, stream, indent=2)
            poses = copy.deepcopy(request['poses'])
            positions.update(pilot['stages'][-1]['samples'][-1])
            state.joint_positions = positions
            state.update()
            for hand, payload in request['assignment'].items():
                body = state.get_global_link_transform(hand + '_wrist_yaw_link') @ np.array(references[hand])
                from scipy.spatial.transform import Rotation
                poses['box48_' + payload]['position'] = dict(zip('xyz', body[:3, 3].tolist()))
                poses['box48_' + payload]['orientation'] = dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist()))
            carry_request = dict(request, mode='retrieval_transport' if request['retrieval'] else 'load_transport',
                                 positions=positions, poses=poses, planning_references=references)
            carry = dual_worker.plan_transport(carry_request)
            with (folder / (label + '_carry.json')).open('x') as stream:
                json.dump(carry, stream, indent=2)
            results.append(dict(mode=label, status='GEOMETRY_PASS_NOT_LIVE', pickup_checks=pickup['checked_samples'],
                                pilot_checks=pilot['checked_samples'], carry_checks=carry['checked_samples']))
        except (RuntimeError, ValueError) as error:
            results.append(dict(mode=label, status='REJECTED', error=str(error)))
        print(results[-1], flush=True)
    with (EVIDENCE / 'paired_fixture_audit.json').open('x') as stream:
        json.dump(dict(status='PASS' if all(row['status'] == 'GEOMETRY_PASS_NOT_LIVE' for row in results) else 'FAIL',
                       records=results, past_stopped_source_sha256=hashlib.sha256(actual.read_bytes()).hexdigest(),
                       source_prospective_layout_transformed_in_memory_only=True,
                       no_gazebo_robot_model_pose_or_guard_changed=True,
                       predicted_lift_after_planning_not_actual_measured_transport=True,
                       actual_new_grip_snapshot_FCL_tracking_and_full_cycle_still_required=True), stream, indent=2)


if __name__ == '__main__':
    main()
