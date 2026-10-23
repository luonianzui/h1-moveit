import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
EXPECTED = {
    'centrifuge_48box_sequential_full_20261023_r355': '75cdc76881244ba7c579f1fbb6772ea357690e360f3d117558e70b3a0c98d965',
    'centrifuge_48box_sequential_review_20261023_r2': '926b27eeaaab598d5d2f5e6d6dd221db745b502096a3df41fd045646d578fa19',
    'centrifuge_48box_sequential_full_20261023_r354': 'b26e2f0e94230ddd26465fee3e6ac9da68ba4d71145647c7b1830d3741cedf5d',
    'centrifuge_48box_sequential_review_20261023_r1': '3de197735f99538d3f2e7ea869b92d5c5334234bb9eb27511fe91ac06e3bd0d9',
    'centrifuge_48box_dart_queries_20261023_r1': 'a86b0b724fc0f1b93fe86bc911a0638f993e086d4ec19dcadf8a0366474d9870',
}

def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def main():
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
            assert digest(path) == checksum, path
            assert path.stat().st_mode & 0o777 == 0o444, path
            records[str(path)] = checksum
    result = dict(status='PASS', sha256=records)
    if phase == 'after':
        assert result == json.loads((ROOT / 'r354_protection_before.json').read_text())
    with (ROOT / f'r354_protection_{phase}.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print('R354_PRESERVED', phase, len(records))

if __name__ == '__main__':
    main()
