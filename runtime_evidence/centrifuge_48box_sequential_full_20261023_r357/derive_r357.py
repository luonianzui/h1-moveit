import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r4'
OLD_REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r3'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == 'ce7ff0e0463f286fcd4bfeacd3721c3830e2afa3b75c56389ccf57a370018fbb'
assert json.loads((PARENT / 'run/retirement.json').read_text())['remaining'] == []
identity = json.loads((PARENT / 'identity.json').read_text())
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json'}
records = []


def rebind(content):
    for previous, current in ((PARENT.name, ROOT.name),
            ('install_lab_scene_48box_sequential_full_20261023_r356', 'install_lab_scene_48box_sequential_full_20261023_r357'),
            ('h1_seq_full_r356', 'h1_seq_full_r357'), ('ROS_DOMAIN_ID=164', 'ROS_DOMAIN_ID=165'), ("'164'", "'165'")):
        content = content.replace(previous, current)
    return content


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
        if not source.name.startswith('verify_') or source.name in ('verify_dart_runtime.py', 'verify_telemetry.py'):
            content = rebind(content)
        if source.name == 'seal_sequential.py':
            content = content.replace("immediate_parent_identity='centrifuge_48box_sequential_full_20261023_r355'", "immediate_parent_identity='centrifuge_48box_sequential_full_20261023_r356'")
            content = content.replace('immediate_parent_failed_not_success=True', 'immediate_parent_failed_not_success=False')
            content = content.replace("success_identity='centrifuge_48box_sequential_full_20261023_r354'", "success_identity='centrifuge_48box_sequential_full_20261023_r356'")
        if source.name == 'test_unilateral_observation.py':
            content = content.replace("PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r357'", "PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r355'")
        data = content.encode()
    destination = ROOT / relative
    if not destination.parent.exists():
        destination.parent.mkdir()
    with destination.open('xb') as stream:
        stream.write(data)
    records.append(dict(relative=relative, parent_sha256=checksum, derived_sha256=hashlib.sha256(data).hexdigest()))
with (ROOT / 'runtime_derivation.json').open('x') as stream:
    json.dump(dict(status='DERIVED_NOT_EXECUTED', parent=str(PARENT), last_success='r356',
                  no_old_claim_results_or_videos_copied=True, records=records), stream, indent=2)
review_records = []
for name in ('document_validation.py', 'observe_runtime.py', 'observe_server_threads.py', 'observe_world_stats.py',
             'capture_native_identity.py', 'analyze_native_profile.py', 'audit_native_symbols.py',
             'audit_native_impulses.py', 'audit_dart_samples.py', 'summarize_world.py',
             'summarize_performance.py', 'phase_performance.py', 'run_continuous.sh', 'freeze_results.py', 'collect_metrics.sh', 'test_sealing_receipt.py', 'seal_component.py', 'audit_inertia_effect.py'):
    data = (OLD_REVIEW / name).read_bytes()
    content = rebind(data.decode()).replace(OLD_REVIEW.name, REVIEW.name)
    if name == 'document_validation.py':
        content = content.replace("'r356' in contents", "'r357' in contents")
    with (REVIEW / name).open('x') as stream:
        stream.write(content)
    review_records.append(dict(name=name, parent_sha256=hashlib.sha256(data).hexdigest()))
with (REVIEW / 'helper_derivation.json').open('x') as stream:
    json.dump(review_records, stream, indent=2)
print('R357_DERIVED', len(records), flush=True)

