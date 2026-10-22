import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r352'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r11'
OLD_REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r10'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == 'da699c92a9e03d64acd5141197eb1aed3908924effd268b6733b754c648a5070'
assert json.loads((PARENT / 'run/retirement.json').read_text())['remaining'] == []
identity = json.loads((PARENT / 'identity.json').read_text())
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json'}
records = []


def rebind(content):
    for previous, current in ((PARENT.name, ROOT.name),
            ('install_lab_scene_48box_sequential_full_20261022_r352', 'install_lab_scene_48box_sequential_full_20261022_r353'),
            ('h1_seq_full_r352', 'h1_seq_full_r353'), ('ROS_DOMAIN_ID=160', 'ROS_DOMAIN_ID=161'), ("'160'", "'161'")):
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
        if source.name != 'verify_current_protection.py':
            content = rebind(content)
        if source.name == 'seal_sequential.py':
            content = content.replace("immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r351'", "immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r352'")
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
review_records = []
for name in ('document_validation.py', 'observe_runtime.py', 'observe_server_threads.py', 'observe_world_stats.py',
             'capture_native_identity.py', 'analyze_native_profile.py', 'audit_native_symbols.py',
             'audit_native_impulses.py', 'audit_dart_samples.py', 'summarize_world.py',
             'summarize_performance.py', 'phase_performance.py', 'run_continuous.sh', 'freeze_results.py'):
    data = (OLD_REVIEW / name).read_bytes()
    content = rebind(data.decode()).replace(OLD_REVIEW.name, REVIEW.name)
    if name == 'document_validation.py':
        content = content.replace("'r352' in contents", "'r353' in contents")
    with (REVIEW / name).open('x') as stream:
        stream.write(content)
    review_records.append(dict(name=name, parent_sha256=hashlib.sha256(data).hexdigest()))
with (REVIEW / 'helper_derivation.json').open('x') as stream:
    json.dump(review_records, stream, indent=2)
print('R353_DERIVED', len(records), flush=True)
