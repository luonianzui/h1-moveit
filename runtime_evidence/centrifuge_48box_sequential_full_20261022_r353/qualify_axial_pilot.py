import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from sequential_geometry import SinglePlanner, pose_matrix
from speed_planning import plan
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent


def main():
    configure_simulation_profile()
    records = []
    for identity in ('r337', 'r339', 'r340'):
        parent = ROOT.parent / ('centrifuge_48box_sequential_full_20261021_' + identity)
        for source in sorted((parent / 'run').glob('*retrieval_pilot*request.json')):
            content = source.read_bytes()
            manifest = {name: checksum for checksum, name in
                        (line.split(maxsplit=1) for line in (parent / 'SHA256SUMS').read_text().splitlines())}
            assert hashlib.sha256(content).hexdigest() == manifest['run/' + source.name]
            request = json.loads(content)
            original = copy.deepcopy(request)
            planned = plan(request)
            first, last = map(np.array, (planned['stages'][0]['first_body'], planned['stages'][0]['last_body']))
            bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
            difference = (np.linalg.inv(bucket) @ last - np.linalg.inv(bucket) @ first)[:3, 3]
            local = np.linalg.inv(bucket) @ first
            assert np.allclose(difference, [-local[0, 3], 0., .015], atol=1e-12, rtol=0)
            assert np.array_equal(first[:3, :3], last[:3, :3]) and request == original
            independent = SinglePlanner(request, planned['planning_reference'])
            seed = {name: request['positions'][name] for name in independent.names}
            independent.attach(first, seed)
            for index, (start, end) in enumerate(zip(planned['stages'][0]['samples'], planned['stages'][0]['samples'][1:])):
                independent.check_edge(start, end, 'independent_axial_pilot', initial_support='basket' if index <= 1 else None)
            records.append(dict(source=str(source), source_sha256=hashlib.sha256(content).hexdigest(),
                original_holders_unchanged=request['original_runtime_holder_references'] == planned['original_runtime_holder_references'],
                local_delta_m=difference.tolist(), independent_checks=independent.checked, plan=planned))
            print('X_RELIEF_Y_PRESERVED_PILOT_PASS', source.name, independent.checked, flush=True)
    result = dict(status='PASS_ACTUAL_SNAPSHOTS_AXIAL_PILOT_GEOMETRY_ONLY', records=records,
                  guard_rpm=.1, brake_torque_limit_nm=2., no_new_physical_certificate=True)
    with (ROOT / 'axial_pilot_qualification.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(result['status'], [(row['source'], row['independent_checks']) for row in records], flush=True)


if __name__ == '__main__':
    main()
