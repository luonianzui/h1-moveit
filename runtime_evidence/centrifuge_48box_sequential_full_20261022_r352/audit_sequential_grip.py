import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
path = ROOT / 'audit_dual_grip.py'
identity = json.loads((ROOT / 'identity.json').read_text())
assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256'][path.name]
source = path.read_text()
before = 'owners.begins == owners.ends == 4 and min(checks.values()) > 100 and simultaneous_checks > 100 and'
after = 'owners.begins == owners.ends == 4 and min(checks.values()) > 100 and simultaneous_checks == 0 and'
assert source.count(before) == 1
source = source.replace(before, after)
before = "                owners.begin(data['hand'], data['payload'], data['native_reference'], data['fk_reference'])"
assert source.count(before) == 1
source = source.replace(before, before + "\n                assert len(owners.rows) == 1, 'Sequential actual holders overlap'")
assert source.count('independent_dual_grip.json') == 1
source = source.replace('independent_dual_grip.json', 'independent_sequential_grip.json')
exec(compile(source, str(path), 'exec'), dict(__name__='__main__', __file__=str(path)))
