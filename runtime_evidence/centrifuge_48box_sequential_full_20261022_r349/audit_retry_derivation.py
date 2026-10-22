import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r289'
assert not (ROOT / 'identity.json').exists()
for name in ('endpoint_overlap_cycle.py', 'execute_cycle.py', 'execute_dual.py', 'elbow_transport_rate.py',
             'elbow_posture.py', 'execute_loading.py', 'retrieval_grip_cycle.py'):
    expected = (PARENT / name).read_text().replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name)
    expected = expected.replace('centrifuge_48box_elbow_full_r289', 'centrifuge_48box_sequential_full_r349')
    assert (ROOT / name).read_text() == expected, name
path = ROOT / 'derivation.json'
with (ROOT / 'derivation_before_review_pause.json').open('xb') as stream:
    stream.write(path.read_bytes())
document = json.loads(path.read_text())
for row in document['records']:
    row['identity_rebound_pre_review_pause_sha256'] = row['final_sha256']
    row['final_sha256'] = hashlib.sha256((ROOT / row['relative']).read_bytes()).hexdigest()
document.update(current_runtime_delta='No action algorithm change from r289; pause before first-frame review and resume immediately into original dispatch; original 25sim freshness guard preserved',
                prior_retrieval_speed_0_3_not_yet_physically_qualified=True)
path.write_text(json.dumps(document, indent=2))
print('RETRY_SOURCE_IDENTITY_ACTION_ALGORITHM_UNCHANGED', len(document['records']), flush=True)
