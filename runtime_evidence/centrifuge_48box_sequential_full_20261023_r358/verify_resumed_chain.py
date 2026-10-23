import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent
NAMES = ('centrifuge_48box_control_resample_20261012_r232',
         'centrifuge_48box_storage_resume_20261013_r235',
         'centrifuge_48box_shared_grasp_probe_20261012_r233',
         'centrifuge_48box_compound_handover_20261012_r234',
         'installer_archive_cleanup_20261013_r1')


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def main():
    phase = sys.argv[1]
    assert phase in ('before', 'after')
    counts, manifests, failures = {}, {}, []
    for name in NAMES:
        root = EVIDENCE.parent / name
        manifest = root / 'SHA256SUMS'
        manifests[name] = digest(manifest)
        rows = manifest.read_text().splitlines()
        counts[name] = len(rows)
        for line in rows:
            expected, relative = line.split(maxsplit=1)
            if digest(root / relative.lstrip('*')) != expected:
                failures.append(name + '/' + relative)
    result = dict(status='PASS' if not failures else 'FAIL', counts=counts,
                  manifests=manifests, failures=failures)
    if phase == 'after':
        assert result == json.loads((EVIDENCE / 'resumed_protected_before.json').read_text())
    with (EVIDENCE / ('resumed_protected_' + phase + '.json')).open('x') as stream:
        json.dump(result, stream, indent=2)
    assert not failures, failures
    print('RESUMED_CHAIN_PASS', phase, counts, flush=True)


if __name__ == '__main__':
    main()
