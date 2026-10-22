import hashlib
import json
from pathlib import Path
import sys

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
phase = sys.argv[1]
assert phase in ('before', 'after')
baseline = json.loads(Path('/home/liu/h1_moveit/backups/backup_1009_release_20261009/archive_payload_sha256.json').read_text())
failures = {}
checked = 0
for relative, expected in baseline.items():
    if relative.startswith('src/'):
        checked += 1
        path = ROOT / relative
        actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else 'MISSING'
        if actual != expected:
            failures[relative] = dict(expected=expected, actual=actual)
result = dict(status='PASS' if not failures else 'FAIL', official_source_files=checked,
              baseline='1009 verified archive payload inventory', failures=failures)
with (EVIDENCE / ('source_1009_' + phase + '.json')).open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result))
assert not failures and checked == 785

