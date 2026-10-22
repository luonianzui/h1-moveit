import hashlib
import json
import math
from pathlib import Path

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
records = []
for identity, prefix in [('r317', '14_retrieval_pilot_left'), ('r316', '14_retrieval_pilot_left'),
                         ('r316', '18_retrieval_pilot_right'), ('r315', '18_retrieval_pilot_right')]:
    source = ROOT.parent / ('centrifuge_48box_sequential_full_20261020_' + identity) / 'run' / (prefix + '_request.json')
    request = json.loads(source.read_text())
    planned = plan(request)
    assert planned['status'] == 'PASS_GEOMETRY_ONLY'
    assert planned['original_runtime_holder_references'] == request['original_runtime_holder_references']
    stage = planned['stages'][0]
    first, last = np.array(stage['first_body']), np.array(stage['last_body'])
    assert first[2, 2] > math.cos(math.radians(2))
    assert np.allclose(last[:3, 2], [0., 0., 1.], atol=1e-12, rtol=0)
    assert abs(math.atan2(first[1, 0], first[0, 0]) - math.atan2(last[1, 0], last[0, 0])) < 1e-12
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    local_first, local_last = np.linalg.inv(bucket) @ first, np.linalg.inv(bucket) @ last
    assert np.allclose(local_last[:2, 3], [0., 0.], atol=1e-12, rtol=0)
    assert abs(local_last[2, 3] - local_first[2, 3] - .015) < 1e-12
    planner = SinglePlanner(request, planned['planning_reference'])
    initial = {name: request['positions'][name] for name in planner.names}
    planner.attach(first, initial)
    for index, (begin, end) in enumerate(zip(stage['samples'], stage['samples'][1:])):
        planner.check_edge(begin, end, 'independent_actual_leveled_pilot', initial_support='basket' if index <= 1 else None)
    name = identity + '_' + prefix + '_level_result.json'
    with (ROOT / name).open('x') as output:
        json.dump(planned, output, indent=2, allow_nan=False)
    records.append(dict(actual_request=str(source), request_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        result=name, independent_dense_checks=planner.checked,
        initial_tilt_deg=math.degrees(math.acos(float(first[2, 2]))), final_tilt_deg=0.,
        original_runtime_holder_reference_unchanged=True))
with (ROOT / 'level_pilot_qualification.json').open('x') as output:
    json.dump(dict(status='PASS_ACTUAL_LEVEL_PILOT_DENSE_GEOMETRY_NOT_PHYSICAL', records=records), output, indent=2)
print('PASS_ACTUAL_LEVEL_PILOT_DENSE_GEOMETRY_NOT_PHYSICAL', records, flush=True)
