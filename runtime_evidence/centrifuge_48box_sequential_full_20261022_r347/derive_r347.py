import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r346'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r5'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == '90e74c464b3ac119c90b799d4da66381a1a76f8e091ba586c514e2d9e597ff9b'
assert json.loads((PARENT / 'run/retirement.json').read_text())['remaining'] == []
assert json.loads((PARENT / 'run/result.json').read_text())['status'] == 'FAIL'
identity = json.loads((PARENT / 'identity.json').read_text())
assert len(identity['sha256']) == 655
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json'}
records = []
for relative, checksum in identity['sha256'].items():
    source = PARENT / relative
    data = source.read_bytes()
    assert hashlib.sha256(data).hexdigest() == checksum
    assert source.stat().st_mode & 0o777 == 0o444
    if not (relative.startswith('inputs/') or source.suffix in ('.py', '.sh') or source.name in fixtures):
        continue
    if source.name.startswith(('derive_', 'freeze_')) or source.name in ('actual_release_snapshot.json', 'prepare_resume.sh', 'prepare_interval_resume.sh'):
        continue
    if source.suffix in ('.py', '.sh', '.json', '.yaml', '.sdf', '.urdf', '.config') and source.name != 'offline_profile_revision1_source_snapshot.json':
        content = data.decode()
        if source.name != 'verify_current_protection.py':
            for previous, current in (
                (PARENT.name, ROOT.name),
                ('install_lab_scene_48box_sequential_full_20261022_r346', 'install_lab_scene_48box_sequential_full_20261022_r347'),
                ('h1_seq_full_r346', 'h1_seq_full_r347'),
                ('centrifuge_48box_sequential_full_r346', 'centrifuge_48box_sequential_full_r347'),
                ('ROS_DOMAIN_ID=154', 'ROS_DOMAIN_ID=155'),
                ("'154'", "'155'"),
            ):
                content = content.replace(previous, current)
        data = content.encode()
    destination = ROOT / relative
    if not destination.parent.exists():
        destination.parent.mkdir()
    with destination.open('xb') as stream:
        stream.write(data)
    records.append(dict(relative=relative, parent_sha256=checksum, derived_sha256=hashlib.sha256(data).hexdigest()))
with (ROOT / 'runtime_derivation.json').open('x') as stream:
    json.dump(dict(status='DERIVED_NOT_EXECUTED', parent=str(PARENT), parent_failed_not_success=True,
                   last_complete_success='centrifuge_48box_sequential_full_20261022_r345', records=records,
                   old_claims_results_and_video_not_copied=True), stream, indent=2)
for name in ('document_validation.py', 'observe_runtime.py', 'observe_server_threads.py',
             'observe_world_stats.py', 'capture_native_identity.py', 'analyze_native_profile.py',
             'audit_native_symbols.py', 'audit_native_impulses.py', 'audit_dart_samples.py',
             'summarize_world.py', 'summarize_performance.py', 'phase_performance.py'):
    source = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r4' / name
    text = source.read_text().replace(PARENT.name, ROOT.name).replace('h1_seq_full_r346', 'h1_seq_full_r347')
    text = text.replace("ROS_DOMAIN_ID='153'", "ROS_DOMAIN_ID='154'").replace("'153'", "'154'")
    if name == 'document_validation.py':
        text = text.replace("'r344' in contents", "'r347' in contents")
    with (REVIEW / name).open('x') as stream:
        stream.write(text)
print('R347_DERIVED', len(records), flush=True)


