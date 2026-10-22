import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r290'
assert not (ROOT / 'identity.json').exists()
names = ('endpoint_overlap_cycle.py', 'execute_cycle.py', 'execute_dual.py', 'elbow_transport_rate.py',
         'elbow_posture.py', 'execute_loading.py', 'retrieval_grip_cycle.py', 'inputs/robot_joint_state_relay_05ms.py')
for name in names:
    expected = (PARENT / name).read_text().replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name)
    expected = expected.replace('centrifuge_48box_elbow_full_r290', 'centrifuge_48box_sequential_full_r348')
    assert (ROOT / name).read_text() == expected, name
path = ROOT / 'derivation.json'
with (ROOT / 'derivation_before_server_review_freeze.json').open('xb') as stream:
    stream.write(path.read_bytes())
document = json.loads(path.read_text())
for row in document['records']:
    row['identity_rebound_pre_server_review_freeze_sha256'] = row['final_sha256']
    row['final_sha256'] = hashlib.sha256((ROOT / row['relative']).read_bytes()).hexdigest()
document.update(current_runtime_delta='No action or strict relay change from r290; owned never-dispatched startup server SIGSTOP/SIGCONT for actual first-frame review instead of failed WorldControl pause',
                unchanged_original_fresh_window_sim_s=25, prior_retrieval_speed_0_3_not_yet_physically_qualified=True)
path.write_text(json.dumps(document, indent=2))
print('ACTION_AND_STRICT_RELAY_BYTE_IDENTITY_PASS', len(names), len(document['records']), flush=True)
