import json
from pathlib import Path
import sys

from verify_r354_protection import digest


ROOT = Path(__file__).resolve().parent
EXPECTED = {
    'centrifuge_48box_sequential_full_20261023_r358': '84d7cb92d80bacea4282f7d9a21a927198b279a2b7c59bdacdc2b4d3f3c94da3',
    'centrifuge_48box_sequential_review_20261023_r4': 'fb04b97165ea28655d25768c64f61bf5ee2524fcef064b2cfeeb7b953530887e',
    'centrifuge_48box_sequential_review_20261023_r5': '5094763741898ed3eb376a8e0bc9b437e3e63041c3f49c184a36e5274d2ef0e6',
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
    assert result == json.loads((ROOT / 'r358_protection_before.json').read_text())
with (ROOT / f'r358_protection_{phase}.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print('R358_AND_FAILED_REVIEWS_PRESERVED', phase, len(records), flush=True)
