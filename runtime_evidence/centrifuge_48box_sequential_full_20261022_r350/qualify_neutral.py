import json
from pathlib import Path

from sequential_geometry import CONTRACT
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
request = json.loads((ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r291/run/6_empty_gripper_after_retreat_request.json').read_text())
request.update(mode='neutral', hand='left', payload='left', sequential_contract=CONTRACT,
               original_runtime_holder_references={}, explicit_offline_fixture=True)
request['positions'].update(json.loads((ROOT / 'inputs/sequential_hints.json').read_text())['park'])
for hand in ('left', 'right'):
    for number in (1, 2):
        request['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
result = plan(request)
assert result['status'] == 'PASS_GEOMETRY_ONLY'
with (ROOT / 'neutral_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_EMPTY_NEUTRAL_NOT_PHYSICAL', request=request, result=result), stream, indent=2)
print('NEUTRAL_GEOMETRY_PASS', result['checked_samples'], flush=True)
