import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parent
manifest = ROOT / 'SHA256SUMS'
assert not manifest.exists()
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and path != manifest)
with manifest.open('x') as stream:
    for path in members:
        expected = hashlib.sha256(path.read_bytes()).hexdigest()
        stream.write(expected + '  ' + str(path.relative_to(ROOT)) + '\n')
for path in members + [manifest]:
    path.chmod(0o444)
for line in manifest.read_text().splitlines():
    expected, relative = line.split(maxsplit=1)
    path = ROOT / relative
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected
    assert path.stat().st_mode & 0o777 == 0o444
print('SEALED_PASS', len(members))
print('REPORT_SHA256', hashlib.sha256((ROOT / 'REPORT.md').read_bytes()).hexdigest())
print('MANIFEST_SHA256', hashlib.sha256(manifest.read_bytes()).hexdigest())
