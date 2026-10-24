import json
from pathlib import Path
import sys
from verify_r354_protection import digest

ROOT = Path(__file__).resolve().parent
EXPECTED = {
    'centrifuge_48box_sequential_full_20261023_r357': '00d784f300aa1b672871b06123c4b2ec9c0838a3e0ba6d01165061d27887f088',
    'centrifuge_48box_dart_lookup_20261023_r1': '0a5c0666db835368e534ea5999e83a117ebd6d3a3f04457b9a96f25cfd0fd6dc',
}
phase = sys.argv[1]
assert phase in ('before', 'after')
records = {}
for name, expected in EXPECTED.items():
    folder = ROOT.parent / name
    manifest = folder / 'SHA256SUMS'
    assert digest(manifest) == expected
    records[str(manifest)] = expected
    for row in manifest.read_text().splitlines():
        checksum, relative = row.split('  ', 1)
        path = folder / relative
        assert digest(path) == checksum and path.stat().st_mode & 0o777 == 0o444
        records[str(path)] = checksum
result = dict(status='PASS', sha256=records)
if phase == 'after':
    assert result == json.loads((ROOT / 'r357_protection_before.json').read_text())
with (ROOT / f'r357_protection_{phase}.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print('R357_FAILED_EVIDENCE_AND_LOOKUP_PRESERVED', phase, len(records))
