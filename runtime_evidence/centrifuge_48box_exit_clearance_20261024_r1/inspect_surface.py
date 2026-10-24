import ctypes
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.spatial.transform import Rotation

from axial_transport import SpeedRetrievalPlanner
from sequential_geometry import pose_matrix, SIZE
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358'
request_path = PARENT / 'run/16_retrieval_transport_left_request.json'
request = json.loads(request_path.read_text())
failure = json.loads((ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r5/axis_exit_failure_probe.json').read_text())
model = WORKSPACE / 'src/h1_2_description/models/centrifuge/model.sdf'
collision = element_tree.parse(model).find("./model/link[@name='bucket_left']/collision")
mesh_path = WORKSPACE / 'src/h1_2_description' / collision.findtext('geometry/mesh/uri').split('h1_2_description/', 1)[1]
facets = np.frombuffer(mesh_path.read_bytes(), offset=84, dtype=np.dtype([
    ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
scale = np.array([float(value) for value in collision.findtext('geometry/mesh/scale').split()])
offset = np.array([float(value) for value in collision.findtext('pose').split()])
triangles = np.ascontiguousarray((facets['vertices'] * scale) @ Rotation.from_euler('xyz', offset[3:]).as_matrix().T + offset[:3])
native = ctypes.CDLL(str(ROOT / 'mesh_probe.so'))
double_array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
native.create_surface.argtypes = [double_array, ctypes.c_size_t]
native.create_surface.restype = ctypes.c_void_p
native.query_surface.argtypes = [ctypes.c_void_p, double_array, double_array, double_array, ctypes.c_int]
native.query_surface.restype = ctypes.c_int
native.delete_surface.argtypes = [ctypes.c_void_p]
surface = native.create_surface(triangles, len(triangles))


def contacts(matrix):
    records = np.empty((200, 8))
    count = native.query_surface(surface, np.ascontiguousarray(matrix), np.array(SIZE), records, len(records))
    return records[:count].tolist()


configure_simulation_profile()
planner = SpeedRetrievalPlanner(request, loaded_reference(request))
seed = {name: request['positions'][name] for name in planner.names}
body = pose_matrix(request['poses']['box48_left'])
bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_left'])
start_wrist = planner.wrist(seed)
attached_reference = np.linalg.inv(start_wrist) @ body
rows = []
for label, joints in [('actual_start', seed), ('rejected_endpoint', failure['checked_edges'][-1]['rejected_endpoint_joint_positions'])]:
    local = np.linalg.inv(bucket) @ planner.wrist(joints) @ attached_reference
    rows.append(dict(label=label, body_bucket_local=local.tolist(), contacts_depth_xyz_normal_triangle=contacts(local)))
local = np.array(rows[0]['body_bucket_local'])
sweep = []
for lift in (0., .001, .0023146, .005, .010, .020, .040, .080):
    for offset_x in (-.001, -.0005, 0., .0005, .001):
        pose = local.copy()
        pose[:3, 3] += [offset_x, 0., lift]
        found = contacts(pose)
        sweep.append(dict(lift_m=lift, x_delta_m=offset_x, contacts=len(found), maximum_depth_m=max((row[0] for row in found), default=0.)))
native.delete_surface(surface)
result = dict(source=str(request_path), source_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest(),
    mesh_path=str(mesh_path), mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
    no_commands_or_physics=True, original_holder_reference_unchanged=True,
    mesh_bounds_m=[triangles.reshape(-1, 3).min(axis=0).tolist(), triangles.reshape(-1, 3).max(axis=0).tolist()],
    rows=rows, sweep=sweep)
with (ROOT / 'surface_probe.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
