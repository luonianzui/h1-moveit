import copy
import gzip
import json
import math
from pathlib import Path

import numpy as np

from alignment_scene import make_alignment_scene
from audit_coherent import verify_bracket
from planner_resources import make_resource_scene
import sequential_geometry
from sequential_geometry import pose_matrix
from speed_planning import plan
from speed_policy import configure_simulation_profile


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r348'
configure_simulation_profile()
original = json.loads((ROOT / 'run/17_retrieval_transport_left_request.json').read_text())
settled = json.loads((REVIEW / 'prefetch_failure_diagnostic.json').read_text())['timeline'][-1]['sim']
frames = {}
selected = None
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        prefix, separator, unused = line.partition(', "payload": ')
        header = json.loads(prefix + '}') if separator else json.loads(line)
        if header['sim'] is None or header['sim'] < settled - .3:
            continue
        if header['kind'] == 'actual_full_joint_frame':
            data = json.loads(line)['payload']
            frames[data['source_sim']] = data['positions']
        elif header['kind'] == 'retrieval_grip':
            data = json.loads(line)['payload']
            if header['sim'] >= settled - .01 and data['coherent_pose_sim'] >= settled - .05:
                positions = verify_bracket(data['actual_joint_bracket'], data['coherent_pose_sim'], frames)
                selected = data
                break
assert selected is not None
poses = selected['paired_poses']
planner = make_alignment_scene(positions, poses, ROOT / 'inputs/robot_dual.urdf')
reference = np.linalg.inv(planner.state.get_global_link_transform('left_wrist_yaw_link')) @ pose_matrix(poses['box48_left'])
request = dict(original, positions=positions, poses=poses,
    source_sim=selected['coherent_pose_sim'], planning_reference=reference.tolist())
outcomes = []
for label, factory in (('original', make_alignment_scene), ('cached', make_resource_scene)):
    sequential_geometry.make_alignment_scene = factory
    try:
        result = plan(copy.deepcopy(request))
        outcomes.append(dict(factory=label, status='PASS_GEOMETRY',
            maximum_tilt_deg=result['maximum_planned_payload_tilt_deg'],
            checked_samples=result['checked_samples']))
    except (RuntimeError, ValueError) as error:
        outcomes.append(dict(factory=label, status='GEOMETRY_REJECTED', error=str(error)))
body = pose_matrix(poses['box48_left'])
with (REVIEW / 'settled_snapshot_qualification.json').open('x') as stream:
    json.dump(dict(status='RECORDED_SETTLED_SNAPSHOT_OFFLINE_NOT_FRESH_PHYSICAL',
        settle_pass_sim=settled, selected_coherent_sim=selected['coherent_pose_sim'],
        source_tilt_deg=math.degrees(math.acos(float(np.clip(body[2, 2], -1, 1)))),
        original_holder_reference_preserved=True, request=request, compared=outcomes), stream, indent=2)
print(json.dumps(outcomes), flush=True)
