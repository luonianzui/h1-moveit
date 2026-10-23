import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import axial_transport
from sequential_geometry import pose_matrix, SIZE
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358/run'
source = RUN / '16_retrieval_transport_left_request.json'
request = json.loads(source.read_text())
original_request = copy.deepcopy(request)
configure_simulation_profile()
records = []
original = axial_transport.SpeedRetrievalPlanner.check_edge
bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_left'])
body = pose_matrix(request['poses']['box48_left'])

def observe(self, first, last, phase, table_endpoint=False, initial_support=None):
    record = dict(phase=phase, initial_support=initial_support, table_endpoint=table_endpoint,
                  maximum_joint_delta_rad=max(abs(last[name] - first[name]) for name in first))
    try:
        original(self, first, last, phase, table_endpoint, initial_support)
        record['status'] = 'PASS_ORIGINAL_GATE'
    except RuntimeError as error:
        record.update(status='REPRODUCED_ORIGINAL_COLLISION_REJECTION', error=str(error))
        self.update(last)
        predicted = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link') @ self.reference
        local = np.linalg.inv(bucket) @ predicted
        record.update(actual_initial_bucket_local=(np.linalg.inv(bucket) @ body).tolist(),
                      rejected_endpoint_bucket_local=local.tolist(),
                      body_motion_world_m=(predicted[:3, 3] - body[:3, 3]).tolist(),
                      original_start_joint_positions=first, rejected_endpoint_joint_positions=last)
        raise
    finally:
        records.append(record)

axial_transport.SpeedRetrievalPlanner.check_edge = observe
try:
    axial_transport.retrieve_transport(request, loaded_reference(request))
    raise AssertionError('Expected actual source rejection')
except RuntimeError as error:
    assert 'Single-holder dense full-scene collision:' in str(error)
finally:
    axial_transport.SpeedRetrievalPlanner.check_edge = original
assert request == original_request
assert records[-1]['status'] == 'REPRODUCED_ORIGINAL_COLLISION_REJECTION'
result = dict(status='PASS_REPRODUCED_R358_GEOMETRIC_FAILURE_NOT_FIXED',
    source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    original_protection_and_runtime_holder_anchors_unchanged=True, no_commands_or_physics=True,
    checked_edges=records, task_pass=False)
with (ROOT / 'axis_exit_failure_probe.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result))
