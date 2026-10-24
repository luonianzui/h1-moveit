import hashlib
import json
from pathlib import Path

from audit_speed_paths import review_path
from sequential_worker import loaded_reference


ROOT = Path(__file__).resolve().parent
records = []
for path in sorted((ROOT / 'run').glob('*_retrieval_transport_*_request.json')):
    output = path.with_name(path.name.replace('_request.json', '_result.json'))
    if not output.is_file():
        continue
    request = json.loads(path.read_text())
    planned = json.loads(output.read_text())
    reference = loaded_reference(request)
    assert planned['planning_reference'] == reference.tolist()
    assert planned['source_snapshot_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    stage = planned['stages'][0]
    assert stage['phase'] == 'retrieve_axis_exit'
    checks = review_path(request, planned)
    records.append(dict(request=path.name, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        hand=request['hand'], correction=stage.get('clearance_correction'), original_dense_audit=checks))
task = json.loads((ROOT / 'run/result.json').read_text())
complete = task['status'] == 'PASS' and {row['hand'] for row in records} == {'left', 'right'}
with (ROOT / 'centered_exit_actual_audit.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_RETRIEVAL_PATH_REVIEW' if complete else 'PARTIAL_NOT_COMPLETE',
        complete_task=complete, records=records,
        actual_correction_count=sum(row['correction'] is not None for row in records),
        corrected_branch_physical_execution_proven=complete and any(row['correction'] is not None for row in records),
        no_action_commands=True), stream, indent=2)
print('ACTUAL_CENTERED_EXIT_REVIEW', complete, len(records), flush=True)
