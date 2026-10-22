import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from bounded_release_alignment import correction_target
from sequential_geometry import CONTRACT, SinglePlanner, pose_matrix
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_review_20261021_r3/release_failure_snapshot.json'
diagnosis = json.loads(SOURCE.read_text())
measured = diagnosis['latest']['synchronized_grip']['payload']
holder = diagnosis['latest']['dual_holder_begin']['payload']
assert measured['hand'] == holder['hand'] == 'right'
request = dict(mode='load_release_align', hand='right', payload='right', sequential_contract=CONTRACT,
    positions=copy.deepcopy(measured['actual_joint_bracket']['positions']),
    poses=copy.deepcopy(measured['paired_poses']), source_sim=measured['coherent_pose_sim'],
    original_runtime_holder_references={'right':copy.deepcopy(holder['frozen'])}, runtime_reference_not_rebased=True)
model = SinglePlanner(request)
joints = {name:request['positions'][name] for name in model.names}
request['planning_reference'] = (np.linalg.inv(model.wrist(joints)) @ pose_matrix(request['poses']['box48_right'])).tolist()
original = copy.deepcopy(request)
records = []
for index in range(2):
    before, target, step = correction_target(request['poses'], 'right')
    planned = plan(request)
    assert planned['maximum_joint_step_rad'] == .002
    assert planned['maximum_ik_position_error_m'] <= .00002
    assert planned['maximum_ik_angle_error_deg'] <= .01
    assert planned['maximum_local_joint_change_rad'] <= .045
    assert planned['original_runtime_holder_references'] == original['original_runtime_holder_references']
    independent = SinglePlanner(request, request['planning_reference'])
    joints = {name:request['positions'][name] for name in independent.names}
    independent.attach(pose_matrix(request['poses']['box48_right']), joints)
    stage = planned['stages'][0]
    for first, last in zip(stage['samples'], stage['samples'][1:]):
        independent.check_edge(first, last, 'bounded_release_independent')
    records.append(dict(before=before, step_deg=step, plan=planned, independent_checks=independent.checked))
    request['positions'].update(stage['samples'][-1])
    body = np.array(stage['last_body'])
    request['poses']['box48_right'] = dict(position=dict(zip('xyz', body[:3,3])),
        orientation=dict(zip('xyzw', Rotation.from_matrix(body[:3,:3]).as_quat())))
assert correction_target(request['poses'], 'right')[0]['within_alignment_goal']
with (ROOT / 'bounded_release_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_R335_LOCAL_STEPS_NOT_PHYSICAL', source=str(SOURCE),
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(), request=original,
        first_step_actual_snapshot=True, second_step_predicted_endpoint_not_actual_feedback=True,
        original_holder_anchors_unchanged=True, records=records), stream, indent=2)
print('BOUNDED_RELEASE_QUALIFIED_NOT_PHYSICAL',[(row['step_deg'],row['independent_checks']) for row in records])
