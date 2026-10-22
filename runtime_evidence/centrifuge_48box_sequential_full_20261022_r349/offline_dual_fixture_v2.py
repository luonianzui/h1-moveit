import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent


def main():
    original = EVIDENCE / 'offline_dual_fixture.py'
    source = original.read_text()
    label = sys.argv[1] if len(sys.argv) == 2 else 'v2'
    assert label in ('v2', 'v3', 'v4')
    changes = {'paired_fixtures': 'paired_fixtures_' + label,
               'right_retrieval_pickup_request.json': 'left_right_pickup_request.json',
               'paired_fixture_audit.json': 'paired_fixture_audit_' + label + '.json'}
    for before, after in changes.items():
        assert source.count(before) == 1
        source = source.replace(before, after)
    with (EVIDENCE / ('fixture_setup_correction_' + label + '.json')).open('x') as stream:
        json.dump(dict(status='SETUP_FILENAME_CORRECTED_ORIGINAL_FAILURE_KEPT',
                       original_source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                       executed_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                       real_previous_request='r236/run/left_right_pickup_request.json',
                       no_geometry_limits_guards_or_runtime_changed=True), stream, indent=2)
    exec(compile(source, str(original), 'exec'), dict(__name__='__main__', __file__=str(original)))


if __name__ == '__main__':
    main()
