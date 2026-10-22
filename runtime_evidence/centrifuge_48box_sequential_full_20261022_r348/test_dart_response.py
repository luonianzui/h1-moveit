import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def test_exact_response_component_identity():
    report = json.loads((ROOT / 'dart_response_dependencies.json').read_text())
    assert report['status'] == 'PASS_EXACT_RESPONSE_COMPONENT_NOT_FULL_ROBOT'
    assert len(report['records']) == 6
    for row in report['records']:
        assert row['world_steps'] == row['contact_steps'] == 2400
        assert row['response_cases'] == 96
        assert row['component_speedup'] > 1.
    for path, expected in report['sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected


def test_response_scope_is_only_new_gazebo_environment():
    contents = (ROOT / 'inputs/gz_solver.launch.py').read_text()
    assert "str(root / 'inputs/libDartContactResponse.so') + ':'" in contents
    assert "if environ.get('LD_PRELOAD'):" in contents
    assert "SetEnvironmentVariable(" not in contents and "os.environ[" not in contents
    assert "h1_seq_full_r348" in contents and "'156'" in contents
