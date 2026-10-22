import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from dual_motion import rigid_reference
from grip_limits import grip_exceeds_limits
from sequential_geometry import EVIDENCE, SinglePlanner, pose_matrix, validate_request
from sequential_planning import park, neutral, pickup, pilot, load_transport, retrieve_transport, retreat
from bounded_release_alignment import plan_release_alignment


EMPTY_MODES = {'park', 'neutral', 'load_pickup', 'retrieval_pickup', 'empty_retreat'}
LOADED_MODES = {'load_pilot', 'load_transport', 'retrieval_pilot', 'retrieval_transport', 'load_release_align'}


def loaded_reference(request):
    holders = request.get('original_runtime_holder_references', {})
    if set(holders) != {request['hand']} or request.get('runtime_reference_not_rebased') is not True:
        raise ValueError('One original actual holder and explicit no-rebase contract are required')
    held = holders[request['hand']]
    if not math.isfinite(held.get('source_sim', float('nan'))) or held['source_sim'] < 0:
        raise ValueError('Finite original grasp source time required')
    native = rigid_reference(held['native_reference'])
    frozen_fk = rigid_reference(held['fk_reference'])
    supplied = rigid_reference(request['planning_reference'])
    planner = SinglePlanner(request)
    joints = {name: request['positions'][name] for name in planner.names}
    wrist = planner.wrist(joints)
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    current = np.linalg.inv(wrist) @ body
    if not np.allclose(supplied, current, rtol=0, atol=1e-10):
        raise ValueError('Planning transform must equal current actual FK/body snapshot, not a historical reference')
    native_wrist = pose_matrix(request['poses'][request['hand'] + '_wrist_yaw_link'])
    for transform in (wrist @ frozen_fk, native_wrist @ native):
        slip = float(np.linalg.norm(transform[:3, 3] - body[:3, 3]))
        angle = math.degrees(Rotation.from_matrix(transform[:3, :3].T @ body[:3, :3]).magnitude())
        if grip_exceeds_limits(slip, angle):
            raise ValueError('Original native/FK two-mm/five-degree holder limit exceeded')
    return supplied


def plan(request):
    validate_request(request)
    mode = request.get('mode')
    if mode not in EMPTY_MODES | LOADED_MODES:
        raise ValueError('Unknown sequential planning mode')
    original = copy.deepcopy(request)
    if mode in EMPTY_MODES:
        if request.get('original_runtime_holder_references'):
            raise ValueError('An empty-phase plan cannot ignore any actual holder')
        if mode == 'park':
            result = park(request)
        elif mode == 'neutral':
            result = neutral(request)
        elif mode == 'empty_retreat':
            result = retreat(request)
        else:
            result = pickup(request, retrieval=mode == 'retrieval_pickup')
            result['prospective_carry_geometry'] = dict(status='NOT_COMPUTED_NOT_A_CERTIFICATE',
                actual_fresh_post_grasp_pilot_and_carry_checks_still_required=True,
                consequence='Actual later plan can fail after grasp; stop-and-hold remains required')
    else:
        reference = loaded_reference(request)
        if mode == 'load_release_align':
            result = plan_release_alignment(request, reference)
        elif mode.endswith('_pilot'):
            result = pilot(request, reference, retrieval=mode == 'retrieval_pilot')
        elif mode == 'load_transport':
            result = load_transport(request, reference)
        else:
            result = retrieve_transport(request, reference)
    if request != original:
        raise RuntimeError('Planner mutated the supplied actual snapshot or original holder anchors')
    result.update(sequential_contract=request['sequential_contract'], mode=mode,
                  original_runtime_holder_references=copy.deepcopy(request.get('original_runtime_holder_references', {})),
                  execution_dispatch_not_qualified=True)
    return result


def main():
    if len(sys.argv) != 3:
        raise ValueError('Expected a snapshot JSON and a never-used output JSON path')
    source, destination = map(Path, sys.argv[1:])
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(str(destination))
    content = source.read_bytes()
    result = plan(json.loads(content))
    result.update(source_snapshot_sha256=hashlib.sha256(content).hexdigest(),
                  worker_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  geometric_hints_sha256=hashlib.sha256((EVIDENCE / 'inputs/sequential_hints.json').read_bytes()).hexdigest())
    with destination.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('SEQUENTIAL_GEOMETRY_ONLY_NOT_PHYSICAL_EXECUTION', result['mode'], result['checked_samples'], flush=True)


if __name__ == '__main__':
    main()
