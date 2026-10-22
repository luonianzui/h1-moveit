import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from release_alignment import release_error
from sequential_geometry import CONTRACT, SinglePlanner, pose_matrix
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_review_20261019_r8/r305_release_diagnosis.json'
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == 'd325c8b833affba286f751dd1205f2f086b0ce92e76d9f4862cfae5e8f5816d1'
diagnosis = json.loads(SOURCE.read_text())
measured = diagnosis['latest']['synchronized_grip']['payload']
holder = diagnosis['latest']['dual_holder_begin']['payload']
assert measured['hand'] == holder['hand'] == 'left'
request = dict(mode='load_release_align', hand='left', payload='left', sequential_contract=CONTRACT,
               positions=copy.deepcopy(measured['actual_joint_bracket']['positions']),
               poses=copy.deepcopy(measured['paired_poses']), source_sim=measured['coherent_pose_sim'],
               original_runtime_holder_references={'left': copy.deepcopy(holder['frozen'])},
               runtime_reference_not_rebased=True)
model = SinglePlanner(request)
joints = {name:request['positions'][name] for name in model.names}
request['planning_reference'] = (np.linalg.inv(model.wrist(joints)) @ pose_matrix(request['poses']['box48_left'])).tolist()
original = copy.deepcopy(request)
result = plan(request)
assert request == original
assert result['status'] == 'PASS_GEOMETRY_ONLY' and result['maximum_joint_step_rad'] == .002
assert result['maximum_ik_position_error_m'] <= .00002 and result['maximum_ik_angle_error_deg'] <= .01
assert result['original_runtime_holder_references'] == original['original_runtime_holder_references']
independent = SinglePlanner(request, request['planning_reference'])
independent.attach(pose_matrix(request['poses']['box48_left']), joints)
stage = result['stages'][0]
assert stage['samples'][0] == joints
for first, last in zip(stage['samples'], stage['samples'][1:]):
    independent.check_edge(first, last, 'independent_release_alignment')
target = np.array(stage['last_body'])
actual_fk_goal = independent.wrist(stage['samples'][-1]) @ np.array(request['planning_reference'])
assert np.linalg.norm(target[:3, 3] - actual_fk_goal[:3, 3]) <= .00002
fixture = dict(source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
               actual_coherent_grip_snapshot_not_new_physical_run=True, request=request)
with (ROOT / 'inputs/actual_release_snapshot.json').open('x') as stream:
    json.dump(fixture, stream, indent=2, allow_nan=False)
with (ROOT / 'release_alignment_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_SNAPSHOT_RELEASE_GEOMETRY_NOT_PHYSICAL',
                   before=release_error(request['poses'], 'left'),
                   independent_dense_checks=independent.checked,
                   original_holder_anchors_unchanged=True, plan=result), stream, indent=2, allow_nan=False)
print('RELEASE_ALIGNMENT_QUALIFIED', independent.checked, result['maximum_local_joint_change_rad'],
      result['maximum_ik_position_error_m'], result['maximum_ik_angle_error_deg'])
