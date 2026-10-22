import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
configure_simulation_profile()
records = []
for identity in ('r337', 'r339', 'r340'):
    parent = ROOT.parent / ('centrifuge_48box_sequential_full_20261021_' + identity)
    for source in sorted((parent / 'run').glob('*retrieval_pilot*request.json')):
        request = json.loads(source.read_text())
        body = pose_matrix(request['poses']['box48_' + request['payload']])
        bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
        local = np.linalg.inv(bucket) @ body
        for axes in ('x', 'y', 'xy'):
            target = body.copy()
            target[:3, 3] += .015 * bucket[:3, 2]
            for axis in axes:
                index = 'xy'.index(axis)
                target[:3, 3] -= bucket[:3, index] * local[index, 3]
            try:
                reference = loaded_reference(request)
                planner = SinglePlanner(request, reference)
                seed = {name: request['positions'][name] for name in planner.names}
                planner.attach(body, seed)
                result = planner.stage(body, target, seed, 'single_pilot', count=21, support_start='basket', refine=False)
                row = dict(status='PASS_GEOMETRY_ONLY', checks=planner.checked)
            except Exception as error:
                row = dict(status='FAIL_GEOMETRY', error=str(error))
            row.update(identity=identity, source=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                       centered_axes=axes, leveling=False, local_start=local.tolist())
            records.append(row)
            print(identity, source.name, axes, row['status'], flush=True)
with (ROOT / 'pilot_geometry_probe.json').open('x') as stream:
    json.dump(dict(records=records, no_execution=True), stream, indent=2)
