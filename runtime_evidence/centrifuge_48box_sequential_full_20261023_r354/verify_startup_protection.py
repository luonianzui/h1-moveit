import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_startup_20261017_r1'
phase = sys.argv[1]
assert phase in ('before', 'after')
manifest = SOURCE / 'SHA256SUMS'
assert hashlib.sha256(manifest.read_bytes()).hexdigest() == '78e146db0f7fb026dc2f71fbb85bdecaf0cefc570dfb3c15a96ea60bdd614095'
rows = manifest.read_text().splitlines()
assert len(rows) == 10
for row in rows:
    expected, relative = row.split(maxsplit=1)
    path = SOURCE / relative.lstrip('*')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected
    assert path.stat().st_mode & 0o777 == 0o444
result = dict(status='PASS', members=10, manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
if phase == 'after':
    assert result == json.loads((ROOT / 'startup_protected_before.json').read_text())
with (ROOT / ('startup_protected_' + phase + '.json')).open('x') as stream:
    json.dump(result, stream, indent=2)
print('STARTUP_SIDECAR_PROTECTION_PASS', phase, flush=True)
