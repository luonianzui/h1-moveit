import hashlib
import json
from pathlib import Path

import numpy as np

from bounded_release_alignment import correction_target


ROOT = Path(__file__).resolve().parent
result = json.loads((ROOT / 'run/result.json').read_text())
assert result['status'] == 'PASS'
counts, records = {}, []
for source in sorted((ROOT / 'run').glob('*_load_release_align_*_request.json')):
    content = source.read_bytes()
    request = json.loads(content)
    output = source.with_name(source.name.replace('_request.json', '_result.json'))
    planned = json.loads(output.read_text())
    assert planned['source_snapshot_sha256'] == hashlib.sha256(content).hexdigest()
    assert planned['original_runtime_holder_references'] == request['original_runtime_holder_references']
    metrics, target, step = correction_target(request['poses'], request['payload'])
    assert step <= 1. + 1e-10 and metrics['position_m'] <= .001 and metrics['angle_deg'] <= 2.
    assert planned['maximum_local_joint_change_rad'] <= .045
    assert planned['maximum_ik_position_error_m'] <= .00002 and planned['maximum_ik_angle_error_deg'] <= .01
    assert np.allclose(np.array(planned['stages'][0]['last_body']), target, atol=1e-12, rtol=0.)
    hand = request['hand']
    counts[hand] = counts.get(hand, 0) + 1
    assert counts[hand] <= 2
    records.append(dict(request=str(source), source_sha256=hashlib.sha256(content).hexdigest(),
        hand=hand, before=metrics, commanded_step_deg=step))
with (ROOT / 'bounded_release_audit.json').open('x') as stream:
    json.dump(dict(status='PASS', actual_requests=records, counts=counts,
        above_one_degree_branch_observed=any(row['before']['angle_deg'] > 1. for row in records),
        original_final_release_gate_and_grip_audit_required_separately=True), stream, indent=2)
print('BOUNDED_RELEASE_ACTUAL_PLANS_PASS', counts)
