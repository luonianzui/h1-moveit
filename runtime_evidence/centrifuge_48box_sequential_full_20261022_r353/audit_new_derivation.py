import hashlib
import json
from pathlib import Path

from elbow_transport_source import restore_elbow_transport


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r288'
assert not (ROOT / 'identity.json').exists()
path = ROOT / 'derivation.json'
document = json.loads(path.read_text())
original = path.read_bytes()
with (ROOT / 'derivation_before_runtime_patch.json').open('xb') as stream:
    stream.write(original)
for row in document['records']:
    source = ROOT / row['relative']
    row['exclusive_identity_rebound_sha256'] = row['final_sha256']
    row['final_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
restored = restore_elbow_transport('endpoint_overlap_cycle.py', (ROOT / 'endpoint_overlap_cycle.py').read_text())
assert restored == (PARENT / 'endpoint_overlap_cycle.py').read_text()
document.update(current_runtime_delta='Only loaded retrieval outboard/transfer rate 0.6 to 0.3; all parent force, model, path and guards unchanged',
                exact_endpoint_parent_inverse_sha256=hashlib.sha256(restored.encode()).hexdigest(),
                old_derivation_before_current_patch_sha256=hashlib.sha256(original).hexdigest())
path.write_text(json.dumps(document, indent=2))
print('CURRENT_RUNTIME_DERIVATION_EXACT_PARENT_INVERSE_PASS', len(document['records']), flush=True)
