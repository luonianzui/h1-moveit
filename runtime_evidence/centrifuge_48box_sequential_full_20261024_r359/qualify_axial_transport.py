import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from axial_transport import retrieve_transport
from sequential_geometry import SinglePlanner, pose_matrix
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
configure_simulation_profile()
records = []
for source in json.loads((ROOT / 'axial_pilot_qualification.json').read_text())['records']:
    request = json.loads(Path(source['source']).read_text())
    planned = source['plan']
    future = copy.deepcopy(request)
    future['mode'] = 'retrieval_transport'
    future['positions'].update(planned['stages'][0]['samples'][-1])
    body = np.array(planned['stages'][0]['last_body'])
    future['poses']['box48_' + future['payload']] = dict(position=dict(zip('xyz', body[:3, 3].tolist())),
        orientation=dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist())))
    result = retrieve_transport(future, planned['planning_reference'])
    exit_body = np.array(result['stages'][0]['last_body'])
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    assert np.allclose((np.linalg.inv(bucket) @ exit_body)[:2, 3],
                       (np.linalg.inv(bucket) @ body)[:2, 3], atol=1e-12, rtol=0)
    actual_input_body = pose_matrix(future['poses']['box48_' + future['payload']])
    assert np.array_equal(exit_body[:3, :3], actual_input_body[:3, :3])
    assert result['world_level_after_axis_exit_with_original_actual_yaw']
    records.append(dict(source=source['source'], source_sha256=source['source_sha256'],
        predicted_geometry_only_not_actual_future_snapshot=True, result=result))
    print('PREDICTED_AXIAL_TRANSPORT_PASS', source['source'], result['checked_samples'], flush=True)
with (ROOT / 'axial_transport_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_PREDICTED_AXIAL_TRANSPORT_NOT_PHYSICAL', records=records), stream, indent=2)
