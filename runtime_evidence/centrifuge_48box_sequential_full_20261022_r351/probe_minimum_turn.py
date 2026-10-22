import hashlib
import json
from pathlib import Path
import time

import dual_worker
from minimum_turn import CONTRACT


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_front_camera_full_20261015_r284'
manifest = {relative: checksum for checksum, relative in
            (line.split('  ', 1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
records = []
for prefix in ('1_load_pickup', '7_retrieval_pickup'):
    relative = 'run/' + prefix + '_request.json'
    content = (PARENT / relative).read_bytes()
    assert hashlib.sha256(content).hexdigest() == manifest[relative]
    request = json.loads(content)
    request['minimum_turn_contract'] = CONTRACT
    started = time.monotonic()
    result = dual_worker.plan_pickup(request)
    selection = result['minimum_turn_selection']
    record = dict(status='PASS_OFFLINE_NOT_PHYSICAL', mode=request['mode'],
                  request_sha256=manifest[relative], selection=selection,
                  elapsed_wall_s=time.monotonic() - started, result=result)
    records.append(record)
    with (ROOT / (prefix + '_minimum_turn_probe_revision4.json')).open('x') as stream:
        json.dump(record, stream, indent=2)
    print('MINIMUM_TURN_OFFLINE', request['mode'], selection['improved'],
          selection['baseline']['score'], selection['selected']['score'], record['elapsed_wall_s'], flush=True)
with (ROOT / 'minimum_turn_offline_audit_revision4.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_NOT_PHYSICAL', records=[dict(mode=record['mode'],
        improved=record['selection']['improved'], baseline=record['selection']['baseline'],
        selected=record['selection']['selected']) for record in records],
        all_old_hold_force_collision_and_actual_fresh_replan_gates_still_required=True), stream, indent=2)
