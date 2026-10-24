import copy
import hashlib
import json
from pathlib import Path
import sys

import axial_transport
from centered_exit import CenteredExitPlanner
from sequential_worker import loaded_reference
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r358'
source_code = (PARENT / 'axial_transport.py').read_text()
assert source_code.count('planner = SpeedRetrievalPlanner(request, reference)') == 1
assert source_code.count('body, seed = target, stage[\'samples\'][-1]') == 1
source_code = source_code.replace('planner = SpeedRetrievalPlanner(request, reference)',
                                'planner = CenteredExitPlanner(request, reference)')
source_code = source_code.replace("body, seed = target, stage['samples'][-1]",
                                "body, seed = np.array(stage['last_body']), stage['samples'][-1]")
namespace = dict(__name__='isolated_exit_candidate', CenteredExitPlanner=CenteredExitPlanner)
exec(compile(source_code, str(ROOT / 'derived_axial_transport.py'), 'exec'), namespace)
configure_simulation_profile()
sources = [PARENT / 'run/16_retrieval_transport_left_request.json']
sources += sorted((ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356/run').glob('*_retrieval_transport_*_request.json'))
records = []
for source in sources:
    request = json.loads(source.read_text())
    original = copy.deepcopy(request)
    planned = namespace['retrieve_transport'](request, loaded_reference(request))
    assert original == request
    records.append(dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), plan=planned))
    print('OFFLINE_GEOMETRY_PASS', source.name, planned['checked_samples'],
          planned['stages'][0].get('clearance_correction'), flush=True)
with (ROOT / 'centered_exit_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_FAILED_AND_SUCCESS_SNAPSHOTS_GEOMETRY_ONLY',
        original_snapshots_and_holder_anchors_unchanged=True, no_commands_or_physics=True,
        records=records), stream, indent=2)
