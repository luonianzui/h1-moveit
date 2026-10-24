import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from audit_speed_paths import review_path
from sequential_geometry import CONTRACT, SinglePlanner, pose_matrix
from speed_planning import plan
from speed_policy import configure_simulation_profile
from table_alignment import MODE


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r341'
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_review_20261021_r9/table_gate_context.json'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == '795ab164f6ec6c5b266a5f60b2eb04d05b0493b67e429faa9e45669b17fd4c91'
assert json.loads((PARENT / 'run/result.json').read_text())['status'] == 'FAIL'
configure_simulation_profile()
context = json.loads(SOURCE.read_text())
records = []
for index in (-30, -15, -1):
    row = context['preceding_grip_rows'][index]
    data = row['original_grip']['payload']
    assert row['table_pairs'] and any(pair['force'] > 0 for pair in row['table_pairs'])
    request = dict(mode=MODE, hand='right', payload='right', sequential_contract=CONTRACT,
        positions=data['actual_joint_bracket']['positions'], poses=data['paired_poses'],
        source_sim=data['coherent_pose_sim'],
        original_runtime_holder_references=json.loads((PARENT / 'run/result.json').read_text())['actual_holder_registry'],
        runtime_reference_not_rebased=True)
    probe = SinglePlanner(request)
    request['planning_reference'] = (np.linalg.inv(probe.wrist(request['positions'])) @
                                    pose_matrix(request['poses']['box48_right'])).tolist()
    original = copy.deepcopy(request)
    planned = plan(request)
    review = review_path(request, planned)
    assert request == original
    records.append(dict(source_index=index, request=request, plan=planned, independent_dense=review))
with (ROOT / 'table_alignment_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_FAILED_TABLE_SNAPSHOT_GEOMETRY_ONLY_NOT_PHYSICAL',
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(), records=records,
        original_runtime_grasp_anchors_not_changed=True), stream, indent=2)
print('TABLE_ALIGNMENT_ACTUAL_SNAPSHOTS', len(records), flush=True)
