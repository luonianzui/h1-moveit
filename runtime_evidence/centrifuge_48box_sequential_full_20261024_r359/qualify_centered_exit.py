import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from axial_transport import retrieve_transport
from audit_speed_paths import review_path
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
configure_simulation_profile()
sources = [ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358/run/16_retrieval_transport_left_request.json']
sources += sorted((ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356/run').glob('*_retrieval_transport_*_request.json'))
assert len(sources) == 3
records = []
for index, source in enumerate(sources):
    request = json.loads(source.read_text())
    original = copy.deepcopy(request)
    reference = loaded_reference(request)
    planned = retrieve_transport(request, reference)
    assert request == original and np.array_equal(planned['planning_reference'], reference)
    correction = planned['stages'][0].get('clearance_correction')
    if index == 0:
        assert correction['no_basket_collision_exemption']
        assert abs(np.linalg.norm(correction['translation_world_m']) - .0009063718403958809) < 1e-12
    else:
        assert correction is None
    for prior, following in zip(planned['stages'], planned['stages'][1:]):
        assert prior['last_body'] == following['first_body']
        assert prior['samples'][-1] == following['samples'][0]
    audit = review_path(request, planned)
    records.append(dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), plan=planned,
                        independent_full_scene_dense_review=audit))
    print('CURRENT_CANDIDATE_CLEARANCE_PASS', source.name, planned['checked_samples'], flush=True)
with (ROOT / 'centered_exit_qualification_revision2.json').open('x') as stream:
    json.dump(dict(status='PASS_R358_FAILED_AND_R356_BOTH_SUCCESS_SNAPSHOTS_NOT_PHYSICAL',
        original_holders_and_requests_unchanged=True, records=records), stream, indent=2)
