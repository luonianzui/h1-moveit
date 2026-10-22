import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r347'
FAILED = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r348'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r7'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == '12127f531a60c98a0cc0075958c894d07ffb98df2940d4f225f19549b947c985'
assert hashlib.sha256((FAILED / 'SHA256SUMS').read_bytes()).hexdigest() == '3740ec2faa8ed2e0c550ba8bca1e99f54819c738f23faa3ca25352d39afe1495'
assert json.loads((FAILED / 'run/retirement.json').read_text())['remaining'] == []
identity = json.loads((PARENT / 'identity.json').read_text())
candidate = json.loads((FAILED / 'identity.json').read_text())
selected = {'planner_resources.py', 'prefetch_cycle.py', 'planner_client.py', 'persistent_worker.py',
            'table_alignment_cycle.py', 'test_planner_resources.py', 'qualify_planner_resources.py',
            'audit_planner_resources.py'}
fixtures = {'actual_table_clearance_native_inputs.json', 'release_descent_probe.json',
            'retrieval_grasp_regression_fixture.json', 'offline_profile_revision1_source_snapshot.json',
            'front_camera_contract.json', 'native_profiler_dependencies.json', 'dart_source_pack.json'}
records = []
for relative in sorted(set(identity['sha256']) | selected):
    source_root = FAILED if relative in selected else PARENT
    source = source_root / relative
    expected = (candidate if relative in selected else identity)['sha256'][relative]
    data = source.read_bytes()
    assert hashlib.sha256(data).hexdigest() == expected and source.stat().st_mode & 0o777 == 0o444
    if not (relative.startswith('inputs/') or source.suffix in ('.py', '.sh') or source.name in fixtures):
        continue
    if source.name.startswith(('derive_', 'freeze_')) or source.name in ('actual_release_snapshot.json', 'prepare_resume.sh', 'prepare_interval_resume.sh'):
        continue
    if source.suffix in ('.py', '.sh', '.json', '.yaml', '.sdf', '.urdf', '.config') and source.name != 'offline_profile_revision1_source_snapshot.json':
        content = data.decode()
        if source.name != 'verify_current_protection.py':
            number, domain = (348, 156) if relative in selected else (347, 155)
            for previous, current in (
                (source_root.name, ROOT.name),
                ('install_lab_scene_48box_sequential_full_20261022_r' + str(number), 'install_lab_scene_48box_sequential_full_20261022_r349'),
                ('h1_seq_full_r' + str(number), 'h1_seq_full_r349'),
                ('centrifuge_48box_sequential_full_r' + str(number), 'centrifuge_48box_sequential_full_r349'),
                ('ROS_DOMAIN_ID=' + str(domain), 'ROS_DOMAIN_ID=157'),
                ("'" + str(domain) + "'", "'157'")):
                content = content.replace(previous, current)
        if source.name == 'prefetch_cycle.py':
            content = content.replace("    if phase == 'seq_retrieval_single_pilot' and seconds == 1.:\n        return ('retrieval_transport', hand, payload)\n", '')
        if source.name == 'test_planner_resources.py':
            content = content.replace("assert next_stationary_plan('seq_retrieval_single_pilot', 1., held) == ('retrieval_transport', hand, hand)",
                "assert next_stationary_plan('seq_retrieval_single_pilot', 1., held) is None")
        if source.name == 'audit_planner_resources.py':
            content = content.replace('assert len(prefetched) == 4', 'assert len(prefetched) == 2')
            content = content.replace("for mode in ('load_transport', 'retrieval_transport')", "for mode in ('load_transport',)")
            content = content.replace('four_original_stationary_observations_preserved', 'two_loading_observations_and_original_postsettle_retrieval_preserved')
        if source.name == 'seal_sequential.py':
            content = content.replace("    from sequential_force import", "    assert read('planner_resources_qualification.json')['status'] == 'PASS_SAME_ACTUAL_SNAPSHOT_GEOMETRY_AND_FRESH_SCENES_NOT_PHYSICAL'\n    from sequential_force import", 1)
            content = content.replace("immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r346'", "immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r347'")
            content = content.replace('immediate_parent_failed_not_success=True', 'immediate_parent_failed_not_success=False')
            content = content.replace("last_complete_success_identity='centrifuge_48box_sequential_full_20261022_r345'", "last_complete_success_identity='centrifuge_48box_sequential_full_20261022_r347'")
            content = content.replace("fastest_complete_success_identity='centrifuge_48box_sequential_full_20261022_r345'", "fastest_complete_success_identity='centrifuge_48box_sequential_full_20261022_r347'")
            content = content.replace("planning_process_reuse='single_owned_process_fresh_scene_per_actual_snapshot_no_plan_cache',", "planning_process_reuse='single_owned_process_fresh_scene_per_actual_snapshot_no_plan_cache',\n        planning_static_resources='single_immutable_robot_model_fresh_scene_and_state',\n        planning_prefetch='two_loading_holds_only_retrieval_after_original_settle',\n        dart_response_candidate_not_loaded=True,")
        if source.name == 'post_speed.sh':
            content = content.replace('audit_trace audit_pipeline', 'audit_trace audit_pipeline audit_planner_resources')
        data = content.encode()
    destination = ROOT / relative
    if not destination.parent.exists():
        destination.parent.mkdir()
    with destination.open('xb') as stream:
        stream.write(data)
    records.append(dict(relative=relative, source=str(source), sha256=expected, derived_sha256=hashlib.sha256(data).hexdigest()))
with (ROOT / 'runtime_derivation.json').open('x') as stream:
    json.dump(dict(status='DERIVED_NOT_EXECUTED', success_parent=str(PARENT),
        selected_planning_sources=str(FAILED), discarded_combined_DART_and_retrieval_prefetch=True,
        no_old_claim_results_or_videos_copied=True, records=records), stream, indent=2)
old_review = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r6'
for name in ('document_validation.py', 'observe_runtime.py', 'observe_server_threads.py',
             'observe_world_stats.py', 'capture_native_identity.py', 'analyze_native_profile.py',
             'audit_native_symbols.py', 'audit_native_impulses.py', 'audit_dart_samples.py',
             'summarize_world.py', 'summarize_performance.py', 'phase_performance.py',
             'run_continuous.sh', 'freeze_results.py'):
    content = (old_review / name).read_text().replace(FAILED.name, ROOT.name).replace(old_review.name, REVIEW.name)
    content = content.replace('h1_seq_full_r348', 'h1_seq_full_r349').replace('ROS_DOMAIN_ID=156', 'ROS_DOMAIN_ID=157').replace("'156'", "'157'")
    if name == 'document_validation.py':
        content = content.replace("'r348' in contents", "'r349' in contents")
    if name == 'summarize_performance.py':
        content = content.replace("dart_response_component=read('dart_response_dependencies.json'),", "dart_response_component_not_loaded=True,")
    with (REVIEW / name).open('x') as stream:
        stream.write(content)
print('R349_DERIVED', len(records), flush=True)
