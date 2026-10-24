import hashlib
import json
from pathlib import Path

import numpy as np
from moveit.core.collision_detection import CollisionResult

from axial_transport import SpeedRetrievalPlanner
from sequential_geometry import pose_matrix
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358'
request_path = PARENT / 'run/16_retrieval_transport_left_request.json'
request = json.loads(request_path.read_text())
failure = json.loads((ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r5/axis_exit_failure_probe.json').read_text())
configure_simulation_profile()
planner = SpeedRetrievalPlanner(request, loaded_reference(request))
seed = {name: request['positions'][name] for name in planner.names}
body = pose_matrix(request['poses']['box48_left'])
bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_left'])
planner.attach(body, seed)
planner.planner.scene.allowed_collision_matrix.set_entry('box48_left', 'centrifuge_bucket_left_bucket_left_collision', False)
planner.planner.request.contacts = True
planner.planner.request.max_contacts = 1000
planner.planner.request.max_contacts_per_pair = 100
rows = []
for label, joints in [('actual_start', seed), ('rejected_endpoint', failure['checked_edges'][-1]['rejected_endpoint_joint_positions'])]:
    planner.update(joints)
    collision = CollisionResult()
    planner.planner.scene.check_collision(planner.planner.request, collision, planner.planner.state)
    print(label, [name for name in dir(collision) if not name.startswith('_')], flush=True)
    print('CONTACTS', collision.contacts, flush=True)
    records = []
    for pair, contacts in collision.contacts.items():
        for contact in contacts:
            print('CONTACT_FIELDS', [name for name in dir(contact) if not name.startswith('_')], flush=True)
            position = np.asarray(contact.pos)
            records.append(dict(pair=list(pair), depth_m=float(contact.depth), position_world_m=position.tolist(),
                position_bucket_local_m=(np.linalg.inv(bucket) @ np.r_[position, 1])[:3].tolist(), normal=np.asarray(contact.normal).tolist()))
    rows.append(dict(label=label, collision=collision.collision, contact_count=collision.contact_count, contacts=records))
result = dict(source=str(request_path), source_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest(),
    no_commands_or_physics=True, rows=rows)
with (ROOT / 'collision_contacts.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
