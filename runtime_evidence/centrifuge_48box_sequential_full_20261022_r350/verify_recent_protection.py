import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent


def main():
    phase = sys.argv[1]
    assert phase in ('before', 'after')
    counts = {}
    failures = []
    for name in ('centrifuge_48box_horizontal_rate_20261012_r225',
                 'centrifuge_48box_three_minute_budget_20261012_r226'):
        root = EVIDENCE.parent / name
        lines = (root / 'SHA256SUMS').read_text().splitlines()
        counts[name] = len(lines)
        for line in lines:
            expected, relative = line.split(maxsplit=1)
            path = root / relative.lstrip('*')
            hasher = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    hasher.update(chunk)
            if hasher.hexdigest() != expected:
                failures.append(str(path))
    with (EVIDENCE / ('recent_protected_' + phase + '.json')).open('x') as stream:
        json.dump(dict(status='FAIL' if failures else 'PASS', counts=counts, failures=failures), stream, indent=2)
    assert not failures, failures
    print('RECENT_PROTECTION_PASS', phase, counts)


if __name__ == '__main__':
    main()
