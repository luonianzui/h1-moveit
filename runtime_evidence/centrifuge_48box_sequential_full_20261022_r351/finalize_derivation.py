import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
path = ROOT / 'derivation.json'
assert not (ROOT / 'identity.json').exists()
with (ROOT / 'derivation_before_seal_wrapper_fix.json').open('xb') as stream:
    stream.write(path.read_bytes())
document = json.loads(path.read_text())
for row in document['records']:
    row['preseal_wrapper_patch_previous_final_sha256'] = row['final_sha256']
    row['final_sha256'] = hashlib.sha256((ROOT / row['relative']).read_bytes()).hexdigest()
document['seal_attempt1_failed_before_identity_or_launch'] = 'Direct low-level sealer lacked the scene dependency certificate; retained seal.log. Proper preflight wrapper generates it before the sole actual identity seal.'
path.write_text(json.dumps(document, indent=2))
print('FINAL_DERIVATION_CURRENT_SOURCES_RECORDED', len(document['records']), flush=True)
