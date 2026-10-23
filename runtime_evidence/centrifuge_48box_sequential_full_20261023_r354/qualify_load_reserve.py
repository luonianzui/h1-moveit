import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
source = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r308/run/4_load_transport_left_request.json'
checksum = '44d834e28a1856d396a6c6b7954ffd263378463289c8dc0e2954236ccc2d9d76'
assert hashlib.sha256(source.read_bytes()).hexdigest() == checksum
request = json.loads(source.read_text())
original = copy.deepcopy(request)
planned = plan(request)
assert request == original
independent = SinglePlanner(request, planned['planning_reference'])
initial = {name:request['positions'][name] for name in independent.names}
independent.attach(pose_matrix(request['poses']['box48_' + request['payload']]), initial)
for stage_index, stage in enumerate(planned['stages']):
    for index, (first, last) in enumerate(zip(stage['samples'], stage['samples'][1:])):
        support = 'table' if stage_index == 0 and index == 0 else 'table_start_only' if stage_index == 0 and index == 1 else None
        independent.check_edge(first, last, 'independent_interior_load', initial_support=support)
final = planned['stages'][-1]['samples'][-1]
values = np.array([final[name] for name in independent.active])
margin = float(np.min(np.minimum(values-independent.lower[independent.indices], independent.upper[independent.indices]-values)))
assert margin >= .001 - 1e-10
perturbed = dict(final)
perturbed['left_shoulder_yaw_joint'] -= 1.0284892937306722e-9
independent.check_edge(perturbed, final, 'synthetic_old_boundary_roundoff_not_measured_runtime')
with (ROOT / 'load_reserve_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_LOAD_SNAPSHOT_INTERIOR_IK_NOT_PHYSICAL', source=str(source),
        source_sha256=checksum, request=request, plan=planned,
        independent_original_dense_checks=independent.checked,
        minimum_final_active_margin_rad=margin, synthetic_roundoff_probe_not_measured_runtime=True,
        original_joint_guard_and_grasp_anchors_unchanged=True, no_physical_action=True), stream, indent=2, allow_nan=False)
print('INTERIOR_LOAD_QUALIFIED_NOT_PHYSICAL', independent.checked, margin, flush=True)
