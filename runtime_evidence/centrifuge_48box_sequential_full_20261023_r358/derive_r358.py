import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r357'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r5'
OLD_REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r4'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == '2ef78c0eada9cabdcd2aeac600035359a99d455a912bc6879583168d5d2cb57a'
assert json.loads((PARENT / 'run/retirement.json').read_text())['remaining'] == []
identity = json.loads((PARENT / 'identity.json').read_text())
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json', 'r355_preload_fixture.json'}
records = []


def rebind(content):
    for previous, current in ((PARENT.name, ROOT.name),
            ('install_lab_scene_48box_sequential_full_20261023_r357', 'install_lab_scene_48box_sequential_full_20261023_r358'),
            ('h1_seq_full_r357', 'h1_seq_full_r358'), ('ROS_DOMAIN_ID=165', 'ROS_DOMAIN_ID=166'), ("'165'", "'166'")):
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
            content = content.replace("immediate_parent_identity='centrifuge_48box_sequential_full_20261023_r356'", "immediate_parent_identity='centrifuge_48box_sequential_full_20261023_r357'")
            content = content.replace('immediate_parent_failed_not_success=False', 'immediate_parent_failed_not_success=True')
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
             'summarize_performance.py', 'phase_performance.py', 'run_continuous.sh', 'freeze_results.py', 'collect_metrics.sh', 'test_sealing_receipt.py', 'seal_component.py', 'audit_inertia_effect.py', 'extract_inspection.py'):
    data = (OLD_REVIEW / name).read_bytes()
    content = rebind(data.decode()).replace(OLD_REVIEW.name, REVIEW.name)
    if name == 'document_validation.py':
        content = content.replace("'r357' in contents", "'r358' in contents")
    with (REVIEW / name).open('x') as stream:
        stream.write(content)
    review_records.append(dict(name=name, parent_sha256=hashlib.sha256(data).hexdigest()))
with (REVIEW / 'helper_derivation.json').open('x') as stream:
    json.dump(review_records, stream, indent=2)
print('R358_DERIVED', len(records), flush=True)
