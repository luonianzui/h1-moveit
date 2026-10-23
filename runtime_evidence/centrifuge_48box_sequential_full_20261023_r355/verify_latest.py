import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent
PARENTS = ('centrifuge_48box_dual_sync_20261012_r227',
           'centrifuge_48box_servo_rate_20261012_r228',
           'centrifuge_48box_servo_fast_20261012_r229',
           'centrifuge_48box_coherent_feedback_20261012_r230',
           'centrifuge_48box_lower_release_20261012_r231')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    phase = sys.argv[1]
    assert phase in ('before', 'after')
    counts, manifests, failures = {}, {}, []
    for name in PARENTS:
        root = EVIDENCE.parent / name
        manifests[name] = digest(root / 'SHA256SUMS')
        lines = (root / 'SHA256SUMS').read_text().splitlines()
        counts[name] = len(lines)
        for line in lines:
            expected, relative = line.split(maxsplit=1)
            if digest(root / relative.lstrip('*')) != expected:
                failures.append(name + '/' + relative)
    if phase == 'after':
        assert manifests == json.loads((EVIDENCE / 'latest_protected_before.json').read_text())['manifests']
    with (EVIDENCE / f'latest_protected_{phase}.json').open('x') as stream:
        json.dump(dict(status='FAIL' if failures else 'PASS', counts=counts,
                       manifests=manifests, failures=failures), stream, indent=2)
    assert not failures
    if phase == 'before':
        parent = EVIDENCE.parent / PARENTS[1]
        with (EVIDENCE / 'parent_servo_calibration_audit.json').open('x') as stream:
            json.dump(dict(status='PASS', parent=str(parent),
                source_calibration_audit_sha256=digest(parent / 'calibration_audit.json'),
                physical_certificate=json.loads((parent / 'calibration_audit.json').read_text()),
                only_small_empty_arm_ramp_certified=True, loaded_motion_not_certified=True), stream, indent=2)
    print('LATEST_PROTECTION_PASS', phase, counts, flush=True)


if __name__ == '__main__':
    main()
