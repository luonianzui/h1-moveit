import copy
import json
from pathlib import Path
import sys

import numpy as np

from sequential_geometry import CONTRACT, SinglePlanner
from sequential_planning import pilot
from sequential_worker import loaded_reference


ROOT = Path(__file__).resolve().parent
records = []
for identity in ('centrifuge_48box_sequential_full_20261017_r294',
                 'centrifuge_48box_sequential_full_20261017_r295'):
    source = ROOT.parent / identity / 'run/13_retrieval_pilot_left_request.json'
    request = json.loads(source.read_text())
    original = copy.deepcopy(request)
    assert request['sequential_contract'] == CONTRACT
    reference = loaded_reference(request)
    result = pilot(request, reference, retrieval=True)
    stage = result['stages'][0]
    first, last = map(np.array, (stage['first_body'], stage['last_body']))
    assert np.array_equal(first[:2, 3], last[:2, 3])
    assert abs(last[2, 3] - first[2, 3] - .015) < 1e-12
    assert np.array_equal(first[:3, :3], last[:3, :3])
    assert request == original
    records.append(dict(parent=identity, request=request, result=result,
                        old_runtime_anchor_preserved=True, no_physical_action=True))
    print('ACTUAL_SNAPSHOT_VERTICAL_PILOT_GEOMETRY_PASS', identity, result['checked_samples'], flush=True)
destination = ROOT / (sys.argv[1] if len(sys.argv) == 2 else 'vertical_pilot_qualification.json')
assert destination.parent == ROOT and destination.suffix == '.json'
with destination.open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_VERTICAL_PILOT_NOT_PHYSICAL', records=records), stream, indent=2)
