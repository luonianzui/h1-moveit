import hashlib
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
FOLDERS = (
    'centrifuge_48box_sequential_full_20261022_r347',
    'centrifuge_48box_sequential_full_20261022_r348',
    'centrifuge_48box_sequential_full_20261022_r349',
    'centrifuge_48box_sequential_full_20261022_r350',
    'centrifuge_48box_sequential_full_20261022_r351',
    'centrifuge_48box_dart_response_20261022_r1',
    'build_cache_cleanup_20261022_r1',
)
records = []
for name in FOLDERS:
    manifest = WORKSPACE / 'runtime_evidence' / name / 'SHA256SUMS'
    output = ROOT / (name + '_manifest_check.json')
    process = subprocess.run([
        str(WORKSPACE / 'tools/trace_archive/run'), '--', '/usr/bin/python3',
        str(WORKSPACE / 'tools/trace_archive/archive_cli.py'), 'verify-manifest',
        '--output', str(output), str(manifest)], capture_output=True, text=True, check=False)
    assert process.returncode == 0, (name, process.stderr, process.stdout[-1500:])
    result = json.loads(process.stdout)
    assert result['status'] == 'PASS'
    record = dict(folder=name, status='PASS', checked_files=result['checked_files'],
        manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
    records.append(record)
    print(json.dumps(record), flush=True)
with (ROOT / 'round_manifest_verification.json').open('x') as stream:
    json.dump(dict(status='PASS', records=records, archived_trace_view_used=True,
        baseline_r347_not_modified=True), stream, indent=2)
