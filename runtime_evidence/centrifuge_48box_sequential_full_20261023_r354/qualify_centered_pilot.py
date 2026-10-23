import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from sequential_geometry import SinglePlanner, pose_matrix
from sequential_planning import pilot
from sequential_worker import loaded_reference


ROOT = Path(__file__).resolve().parent
records = []
sources = [
    ('centrifuge_48box_sequential_full_20261017_r294', '13_retrieval_pilot_left_request.json', None),
    ('centrifuge_48box_sequential_full_20261017_r295', '13_retrieval_pilot_left_request.json', None),
    ('centrifuge_48box_sequential_full_20261019_r306', '14_retrieval_pilot_left_request.json', '551fd594ad82c592f643124f8509fb69a57b7f6c0bf26241a2dd48208259dbca'),
    ('centrifuge_48box_sequential_full_20261019_r307', '14_retrieval_pilot_left_request.json', '81e6aef49c52f4519a9dca799f58376eb7bbc1db61c309710b1362c55c5935f7')]
for identity, name, checksum in sources:
    source = ROOT.parent / identity / 'run' / name
    if checksum is None:
        manifest = {relative:expected for expected,relative in (line.split(maxsplit=1)
                    for line in (source.parents[1] / 'SHA256SUMS').read_text().splitlines())}
        checksum = manifest['run/' + name]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == checksum
    request = json.loads(source.read_text())
    original = copy.deepcopy(request)
    reference = loaded_reference(request)
    planned = pilot(request, reference, retrieval=True)
    stage = planned['stages'][0]
    first, last = map(np.array, (stage['first_body'], stage['last_body']))
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    initial_local, final_local = np.linalg.inv(bucket) @ first, np.linalg.inv(bucket) @ last
    assert np.max(np.abs(final_local[:2,3])) < 1e-12
    assert abs(final_local[2,3] - initial_local[2,3] - .015) < 1e-12
    assert np.allclose(last[:3,2], [0.,0.,1.], atol=1e-12, rtol=0) and request == original
    assert abs(np.arctan2(first[1,0],first[0,0])-np.arctan2(last[1,0],last[0,0])) < 1e-12
    independent = SinglePlanner(request, reference)
    initial = {name:request['positions'][name] for name in independent.names}
    independent.attach(first, initial)
    for index, (begin, end) in enumerate(zip(stage['samples'], stage['samples'][1:])):
        independent.check_edge(begin, end, 'independent_centered_pilot',
                               initial_support='basket' if index <= 1 else None)
    records.append(dict(parent=identity, request_sha256=checksum, request=request, result=planned,
                        independent_dense_checks=independent.checked,
                        initial_local_position=initial_local[:3,3].tolist(),
                        final_local_position=final_local[:3,3].tolist(),
                        old_runtime_anchor_preserved=True, no_physical_action=True))
    print('ACTUAL_SNAPSHOT_CENTERED_PILOT_PASS', identity, independent.checked, flush=True)
with (ROOT / 'centered_pilot_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_SNAPSHOTS_CENTERED_PILOT_NOT_PHYSICAL', records=records), stream, indent=2)
