import copy
import hashlib
import json
from pathlib import Path
import time
import sys

from sequential_geometry import CONTRACT
from sequential_worker import plan


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r291/run'
parking = json.loads((ROOT / 'inputs/sequential_hints.json').read_text())['park']
records = []
prefix = sys.argv[1] if len(sys.argv) == 2 else 'sequential_dispatch'
assert prefix.replace('_', '').isalnum()
for mode, fixture in [('load_pickup', '1_load_pickup_request.json'), ('retrieval_pickup', '8_retrieval_pickup_request.json')]:
    base = json.loads((PARENT / fixture).read_text())
    base['positions'].update(parking)
    for hand in ('left', 'right'):
        for number in (1, 2):
            base['positions'][hand + f'_dex1_finger_{number}_joint'] = -.0199
    assignment = dict(left='left', right='right') if mode == 'load_pickup' else dict(left='right', right='left')
    for hand in ('left', 'right'):
        request = copy.deepcopy(base)
        request.update(mode=mode, hand=hand, payload=assignment[hand], sequential_contract=CONTRACT,
                       original_runtime_holder_references={}, runtime_reference_not_rebased=True,
                       explicit_offline_pose_fixture_not_fresh_current_robot=True)
        began = time.monotonic()
        try:
            result = plan(request)
        except (ValueError, RuntimeError, AssertionError) as error:
            with (ROOT / f'{prefix}_{mode}_{hand}_failed.json').open('x') as stream:
                json.dump(dict(status='FAIL_OFFLINE_NO_ROBOT_COMMAND', mode=mode, hand=hand,
                               request=request, error=str(error)), stream, indent=2)
            raise
        assert result['status'] == 'PASS_GEOMETRY_ONLY' and result['prospective_carry_geometry']['status'] == 'NOT_COMPUTED_NOT_A_CERTIFICATE'
        row = dict(mode=mode, hand=hand, request=request, result=result, wall_s=time.monotonic() - began,
                   fixture_sha256=hashlib.sha256((PARENT / fixture).read_bytes()).hexdigest())
        assert row['wall_s'] < 110
        records.append(row)
        with (ROOT / f'{prefix}_{mode}_{hand}.json').open('x') as stream:
            json.dump(row, stream, indent=2)
        print('SEQUENTIAL_DISPATCH_QUALIFIED_GEOMETRY_ONLY', mode, hand, row['wall_s'], flush=True)
with (ROOT / (prefix + '_qualification.json')).open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_EXACT_DISPATCH_NOT_PHYSICAL', records=records,
                   no_actual_holder_frozen_or_robot_command=True), stream, indent=2)
