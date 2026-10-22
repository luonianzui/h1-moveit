import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r349'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r8'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == 'd53c7d220013ccfbd25d646160211ab92ee623e96c903edbbd9ddc8cf62c41e4'
assert hashlib.sha256((PARENT / 'SHA256SUMS').read_bytes()).hexdigest() == 'ffd0b00076dc795ff0caf68ee026a23c65490475f0f6d0f4714f6b6c5747ee0b'
assert json.loads((PARENT / 'run/retirement.json').read_text())['remaining'] == []
identity = json.loads((PARENT / 'identity.json').read_text())
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json'}
records = []
for relative, checksum in identity['sha256'].items():
    source = PARENT / relative
    data = source.read_bytes()
    assert hashlib.sha256(data).hexdigest() == checksum and source.stat().st_mode & 0o777 == 0o444
    if not (relative.startswith('inputs/') or source.suffix in ('.py', '.sh') or source.name in fixtures):
        continue
    if source.name.startswith(('derive_', 'freeze_')) or source.name in ('actual_release_snapshot.json', 'prepare_resume.sh', 'prepare_interval_resume.sh'):
        continue
    if source.suffix in ('.py', '.sh', '.json', '.yaml', '.sdf', '.urdf', '.config') and source.name != 'offline_profile_revision1_source_snapshot.json':
        content = data.decode()
        if source.name != 'verify_current_protection.py':
            for previous, current in ((PARENT.name, ROOT.name), ('h1_seq_full_r349', 'h1_seq_full_r350'),
                ('ROS_DOMAIN_ID=157', 'ROS_DOMAIN_ID=158'), ("'157'", "'158'")):
                content = content.replace(previous, current)
        if source.name == 'seal_sequential.py':
            content = content.replace("immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r347'", "immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r349'")
            content = content.replace('immediate_parent_failed_not_success=False', 'immediate_parent_failed_not_success=True')
        data = content.encode()
    destination = ROOT / relative
    if not destination.parent.exists():
        destination.parent.mkdir()
    with destination.open('xb') as stream:
        stream.write(data)
    records.append(dict(relative=relative, parent_sha256=checksum, derived_sha256=hashlib.sha256(data).hexdigest()))
with (ROOT / 'runtime_derivation.json').open('x') as stream:
    json.dump(dict(status='DERIVED_NOT_EXECUTED', parent=str(PARENT), last_success='r347',
        no_old_claim_results_or_videos_copied=True, records=records), stream, indent=2)
old_review = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r7'
for name in ('document_validation.py', 'observe_runtime.py', 'observe_server_threads.py', 'observe_world_stats.py',
             'capture_native_identity.py', 'analyze_native_profile.py', 'audit_native_symbols.py',
             'audit_native_impulses.py', 'audit_dart_samples.py', 'summarize_world.py',
             'summarize_performance.py', 'phase_performance.py', 'run_continuous.sh', 'freeze_results.py'):
    content = (old_review / name).read_text().replace(PARENT.name, ROOT.name).replace(old_review.name, REVIEW.name)
    content = content.replace('h1_seq_full_r349', 'h1_seq_full_r350').replace('ROS_DOMAIN_ID=157', 'ROS_DOMAIN_ID=158').replace("'157'", "'158'")
    if name == 'document_validation.py':
        content = content.replace("'r349' in contents", "'r350' in contents")
    with (REVIEW / name).open('x') as stream:
        stream.write(content)
print('R350_DERIVED', len(records), flush=True)
