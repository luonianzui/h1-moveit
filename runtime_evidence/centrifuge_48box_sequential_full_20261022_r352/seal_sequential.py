import hashlib
import json
from pathlib import Path
import shutil

from scene_contract import read_contract


ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text())


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    assert read('initial_button_motion_qualification.json')['status'] == 'PASS_ACTUAL_R351_WITHDRAW_GEOMETRY_AND_BOUNDED_RAMP_NOT_PHYSICAL'
    assert read('planner_resources_qualification_revision2.json')['status'] == 'PASS_SAME_ACTUAL_SNAPSHOT_GEOMETRY_AND_FRESH_SCENES_NOT_PHYSICAL'
    from sequential_force import LOADING_FORCE_N, RETRIEVAL_FORCE_N, UNCHANGED_OUTPUT_GUARD_N
    assert read('interval_qualification.json')['status'] == 'PASS_SCHEDULING_AND_ENDPOINT_OVERLAP_NOT_PHYSICAL'
    assert (LOADING_FORCE_N, RETRIEVAL_FORCE_N, UNCHANGED_OUTPUT_GUARD_N) == (16., 16.5, 18.01)
    assert not (ROOT / 'identity.json').exists() and not (ROOT / 'run').exists()
    assert (ROOT / 'conda_six_before.txt').read_bytes() == (ROOT / 'conda_six_after.txt').read_bytes()
    assert read('initialization_compatibility.json')['status'] == 'PASS_ACTUAL_STATIONARY_LEG_COMPATIBILITY_NOT_LOADED_TASK'
    assert read('initialization_compatibility.json')['current_initialization_source_sha256'] == digest(ROOT / 'initialize_stationary_legs.py')
    assert read('speed_qualification.json')['status'] == 'PASS_ACTUAL_R318_PATH_RETIMING_ONLY_NOT_NEW_PHYSICS'
    assert read('speed_retrieval_qualification.json')['status'] == 'PASS_ACTUAL_FAILED_SNAPSHOT_DENSE_GEOMETRY_NOT_PHYSICAL'
    assert read('speed_retrieval_qualification.json')['plan']['world_level_after_axis_exit_with_original_actual_yaw']
    assert read('speed_retrieval_regression.json')['status'] == 'PASS_TWO_R318_ACTUAL_SNAPSHOTS_NOT_NEW_PHYSICS'
    assert len(read('speed_retrieval_regression.json')['records']) == 2
    assert read('runtime_performance_qualification.json')['status'] == 'PASS_EXACT_TIMELINE_AND_UNCHANGED_CONTRACT_NOT_PHYSICAL'
    assert read('batched_trace_qualification.json')['status'] == 'PASS_IDENTICAL_ACTUAL_TRACE_BYTES_NOT_TASK_SPEED'
    assert read('pose_cache_qualification.json')['status'] == 'PASS_ORIGINAL_MATRIX_BYTES_BOUNDED_CACHE_NOT_NEW_PHYSICS'
    assert read('preload_observation_qualification.json')['status'] == 'PASS_ACTUAL_FAULT_ELIGIBILITY_AND_BOUNDED_WAIT_NOT_NEW_PHYSICS'
    assert read('bucket_cache_qualification.json')['status'] == 'PASS_EXACT_BUCKET_CORNERS_ONLY_NOT_NEW_PHYSICS_OR_WHOLE_TASK_SPEED'
    assert read('localization_delivery_qualification.json')['status'] == 'PASS_REAL_DDS_LATEST_SAMPLE_COMPATIBILITY_NOT_GAZEBO_TASK'
    assert read('pipeline_qualification_revision2.json')['status'] == 'PASS_PERSISTENT_PROTOCOL_AND_JOINED_TIMING_NOT_PHYSICAL'
    assert read('right_transfer_qualification.json')['status'] == 'PASS_ACTUAL_R334_FAILED_SNAPSHOT_DENSE_GEOMETRY_NOT_PHYSICAL'
    assert read('bounded_release_qualification.json')['status'] == 'PASS_ACTUAL_R335_LOCAL_STEPS_NOT_PHYSICAL'
    assert '16 passed' in (ROOT / 'bounded_release_tests_revision2.log').read_text()
    for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
                   'additional_protected', 'dual_protected', 'current_protected', 'startup_protected'):
        assert read(prefix + '_before.json')['status'] == 'PASS', prefix
    for name in ('buttons_collision_audit.json', 'control_fork_audit.json', 'servo_fork_audit.json',
                 'calibrated_gain_audit.json', 'codec_audit.json', 'full_window_capture_audit.json', 'archive_runtime_dependencies.json'):
        assert read(name)['status'] == 'PASS', name
    assert read('calibrated_gain_audit.json')['position_proportional_gain'] == .6
    assert read('sequential_dispatch_revision3_qualification.json')['status'] == 'PASS_OFFLINE_EXACT_DISPATCH_NOT_PHYSICAL'
    assert len(read('sequential_dispatch_revision3_qualification.json')['records']) == 4
    assert read('neutral_qualification.json')['status'] == 'PASS_OFFLINE_EMPTY_NEUTRAL_NOT_PHYSICAL'
    assert read('sequential_chain_final.json')['status'] == 'PASS_OFFLINE_SEQUENTIAL_CHAIN_GEOMETRY_ONLY'
    assert len(read('sequential_chain_final.json')['stages']) == 17
    assert read('centered_pilot_qualification.json')['status'] == 'PASS_ACTUAL_SNAPSHOTS_CENTERED_PILOT_NOT_PHYSICAL'
    assert len(read('centered_pilot_qualification.json')['records']) == 4
    assert read('release_alignment_qualification.json')['status'] == 'PASS_ACTUAL_SNAPSHOT_RELEASE_GEOMETRY_NOT_PHYSICAL'
    assert read('axis_exit_qualification.json')['status'] == 'PASS_ACTUAL_FAILED_PATH_RETIMED_NOT_PHYSICAL'
    assert read('load_reserve_qualification.json')['status'] == 'PASS_ACTUAL_LOAD_SNAPSHOT_INTERIOR_IK_NOT_PHYSICAL'
    assert read('floor_frame_qualification.json')['status'] == 'PASS_ACTUAL_SINGLE_FRAME_CLASSIFICATION_NOT_GRIP_AND_NOT_TASK'
    assert read('record_dispatch_qualification.json')['status'] == 'PASS_ACTUAL_RECORD_BINDING_OFFLINE_NOT_TASK'
    assert read('pilot_settle_qualification.json')['status'] == 'PASS_ACTUAL_SETTLING_TREND_NOT_NEW_PHYSICAL_RESULT'
    assert read('single_setdown_qualification.json')['status'] == 'PASS_ACTUAL_SINGLE_SETDOWN_DISPATCH_OFFLINE_NOT_PHYSICAL'
    assert read('whole_single_transport_qualification.json')['status'] == 'PASS_WHOLE_SINGLE_TRANSPORT_DISPATCH_OFFLINE_NOT_PHYSICAL'
    assert read('right_setdown_qualification.json')['status'] == 'PASS_ACTUAL_RIGHT_PATH_RETIMED_OFFLINE_NOT_PHYSICAL'
    assert read('return_height_qualification.json')['status'] == 'PASS_ACTUAL_RETURN_HEIGHT_DENSE_GEOMETRY_NOT_PHYSICAL'
    assert read('level_pilot_qualification.json')['status'] == 'PASS_ACTUAL_LEVEL_PILOT_DENSE_GEOMETRY_NOT_PHYSICAL'
    for name in ('tests_speed_final.log', 'package_tests.log', 'loading_regression_tests_final.log'):
        contents = (ROOT / name).read_text()
        assert ' passed' in contents and ' failed' not in contents and 'ERROR' not in contents, name
    contract = read_contract(ROOT)
    assert read('axial_pilot_qualification.json')['status'] == 'PASS_ACTUAL_SNAPSHOTS_AXIAL_PILOT_GEOMETRY_ONLY'
    assert len(read('axial_pilot_qualification.json')['records']) == 4
    assert read('axial_transport_qualification.json')['status'] == 'PASS_PREDICTED_AXIAL_TRANSPORT_NOT_PHYSICAL'
    assert read('table_alignment_qualification.json')['status'] == 'PASS_ACTUAL_FAILED_TABLE_SNAPSHOT_GEOMETRY_ONLY_NOT_PHYSICAL'
    assert len(read('table_alignment_qualification.json')['records']) == 3
    for name, expected in read('native_profiler_dependencies.json')['sha256'].items():
        assert digest(Path(name)) == expected
    assert read('dart_cache_dependencies.json')['status'] == 'PASS_OFFLINE_EXACT_MATRIX_AND_CONTACT_WORLD_NOT_FULL_TASK'
    for name, expected in read('dart_cache_dependencies.json')['sha256'].items():
        assert digest(Path(name)) == expected
    native_pack = read('dart_source_pack.json')
    native_manifest = ROOT / native_pack['manifest']
    assert digest(native_manifest) == native_pack['sha256']
    assert len(native_manifest.read_text().splitlines()) == native_pack['members']
    for row in native_manifest.read_text().splitlines():
        expected, relative = row.split(maxsplit=1)
        assert digest(native_manifest.parent / relative) == expected
    assert len(read('axial_transport_qualification.json')['records']) == 4
    assert read('geometry_fastpath_qualification.json')['status'] == 'PASS_EXACT_POSE_GEOMETRY_COMPONENT_ONLY_NOT_REALTIME_PROOF'
    assert read('geometry_fastpath_qualification.json')['minimum_component_speedup'] > 1.
    assert read('right_setdown_stability_qualification.json')['status'] == 'PASS_ACTUAL_R338_PATH_RETIMING_ONLY_NOT_PHYSICAL'
    assert read('gentle_pilot_qualification.json')['status'] == 'PASS_R339_ACTUAL_FAILED_PILOT_RETIMING_ONLY_NOT_PHYSICAL'
    assert read('localizer_pipeline_qualification.json')['status'] == 'PASS_ORIGINAL_ALGORITHM_SYNTHETIC_CLOUDS_NOT_PHYSICAL'
    assert contract['full_hand_dependency_reviewed'] and contract['regression']['scene_dependencies_passed']
    assert contract['package_tests_passed'] == 6
    assert shutil.disk_usage(ROOT).free > 2096208487
    members = {}
    for path in sorted(ROOT.iterdir()):
        if path.is_file() and path.suffix in ('.py', '.sh', '.json', '.md'):
            members[path.name] = digest(path)
    for path in sorted((ROOT / 'inputs').iterdir()):
        if path.is_file():
            members[str(path.relative_to(ROOT))] = digest(path)
    from speed_policy import PROFILE
    identity = dict(identity=ROOT.name, scope='two_box_load_close_spin_open_retrieve',
        actual_runtime_subclass='SpeedPriorityCycle', parent_runtime_subclass='SequentialCycle',
        speed_profile=PROFILE, original_strict_certificate_not_claimed=True,
        sequential_contract='sequential_single_holder_elbow_up_v1', maximum_actual_holders=1,
        both_payloads_required=True, manager_update_rate_hz=500, position_proportional_gain=.6,
        retrieval_feedforward_N=16.5, loading_feedforward_N=16., parent_retrieval_feedforward_N=18.,
        immediate_parent_retrieval_feedforward_N=16.5,
        immediate_parent_identity='centrifuge_48box_sequential_full_20261022_r351',
        localization_rate_gate='exact_integer_nanoseconds_original_stamps_and_1s_guard',
        retrieval_pilot_stability_rate_rad_s=.05,
        immediate_parent_failed_not_success=True,
        future_start_offset_sim_s=.3,
        named_free_empty_future_start_offset_sim_s=.16,
        endpoint_observation_overlap_only_named_empty_free_phases=True,
        continuous_executor_no_ai_between_actions=True,
        noncontact_button_withdraw_rate_rad_s=.6,
        initial_button_withdraw_ramp_sim_s=.3,
        initial_button_withdraw_acceleration_rad_s2=2.,
        button_controller_reference_readonly_max_samples=10000,
        button_tracking_limit_unchanged_rad=.15,
        dart_optimization='same_order_per_group_constraint_cache_and_flat_symmetric_copy',
        runtime_speed_subclass_module='table_alignment_cycle',
        table_alignment='at_most_two_actual_supported_horizontal_1mm_steps_original_2mm_2deg_release_gate',
        native_task_profiler='owned_PreUpdate_thread_CPU_samples_only_no_physics_or_control_changes',
        right_coarse_setdown_rate_rad_s=.2,
        right_coarse_setdown_ramp_sim_s=.6,
        pose_key_fastpath='explicit_seven_IEEE_double_fields_no_rounding',
        bucket_transform_cache_max_entries=512,
        localization_producer_rate_hz=5.,
        localization_source_timestamps_and_quality_unchanged=True,
        localization_producer_tf_and_publication_trace=True,
        last_complete_success_identity='centrifuge_48box_sequential_full_20261022_r347',
        planning_process_reuse='single_owned_process_fresh_scene_per_actual_snapshot_no_plan_cache',
        planning_static_resources='single_immutable_robot_model_fresh_scene_and_state',
        planning_prefetch='two_loading_holds_only_retrieval_after_original_settle',
        dart_response_candidate_not_loaded=True,
        native_transport_finite_send_receive_hwm=20000,
        original_joint_source_step_ns=500000,
        owned_relay_and_server_wall_watchdog=True,
        original_floor_contact_age_ns=1000000,
        floor_age_comparison='integer_nanoseconds_same_one_millisecond_limit',
        joined_pairs=[['load_lift', 'load_transfer'], ['retrieve_outboard', 'retrieve_transfer']],
        fastest_complete_success_identity='centrifuge_48box_sequential_full_20261022_r347',
        localization_delivery='KEEP_LAST_DEPTH_1_BEST_EFFORT_WITH_UNCHANGED_1S_STAMP_GUARD',
        protected_original_success_identity='centrifuge_48box_sequential_full_20261020_r318',
        runtime_performance='exact_deque_bisect_joint_timeline_no_pose_or_guard_decimation',
        stack_sampler_wall_interval_s=.02,
        lossless_trace_batch_bytes=65536,
        retrieval_pilot_only_rate_rad_s=.1,
        empty_button_noncontact_rate_cap_rad_s=.8,
        pose_transform_cache_max_entries=512,
        exact_bucket_corner_cache_max_entries=512,
        preload_zero_force_observation_wait_wall_s=.020,
        preload_zero_force_observation_wait_sim_s=.001,
        original_support_and_strong_capture_required_after_wait=True,
        maximum_fresh_start_sim_s=300., original_fresh_start_sim_s=25.,
        startup_compatibility='pre_task_original_leg_targets_3s_smooth_and_5s_fresh_endpoint_hold',
        stationary_leg_endpoint_limit_rad=.005, original_stationary_leg_endpoint_limit_rad=.001,
        physics_fork='fixed_pelvis_two_original_stance_ground_trimmed_sole_OBB_collision_only',
        pickup_prospective_planning='NOT_COMPUTED_NOT_A_CERTIFICATE',
        immediate_parent_actual_post_pilot_tilt_deg=None,
        immediate_parent_duplicate_record_shadowed_history=False,
        immediate_parent_post_pilot_snapshot_drift_failed=False,
        immediate_parent_single_setdown_dispatched_to_paired_profile=False,
        immediate_parent_single_setdown_predecessor_phase_mismatch=False,
        immediate_parent_right_coarse_setdown_native_slip_failed=False,
        immediate_parent_right_transfer_IK_failed_before_dispatch=False,
        right_return_transfer_height_m=1.12475,
        single_setdown_coarse_rate_rad_s=.4, single_setdown_ramp_sim_s=.3,
        right_single_setdown_coarse_rate_rad_s=.4, right_single_setdown_ramp_sim_s=.3,
        guarded_retrieval_pilot_settle_sim_s=1.0,
        immediate_parent_interrupted_before_retrieval=False,
        immediate_parent_release_orientation_failed=False,
        immediate_parent_axis_exit_contact_force_failed=False,
        immediate_parent_measured_pilot_tilt_before_axis_dispatch_failed=False,
        immediate_parent_actual_initial_release_joint_bound_failed=False,
        immediate_parent_startup_stats_parse_failed_before_dispatch=False,
        immediate_parent_one_missing_basket_frame_during_early_preload_failed=False,
        early_preload_floor_history='one_0.5ms_native_contact_frame_only_with_5um_floor_geometry_and_current_accepted_closing_not_grip_certificate',
        startup_stats_policy='strict_complete_monotonic_json_stream_latest_record_explicit_300s_startup_window',
        load_transport_active_ik_additional_interior_reserve_rad=.001,
        original_joint_bound_dense_guard_unchanged=True,
        basket_axis_exit_rate_rad_s=.15, basket_axis_exit_tracking_limit_rad=.04,
        loading_release_alignment='at_most_two_measured_steps_each_lte_1deg_and_1mm_observed_angle_lte_2deg_final_goal_unchanged',
        retrieval_pilot_geometry='15mm_basket_axis_lift_and_local_x_relief_preserve_actual_local_y_and_orientation',
        retrieval_axis_exit_geometry='axis_only_until_original_rim_plus_6mm_then_world_level',
        loading_preload_support='accepted_current_goal_native_0.25mm_12ms_opposed_midband_3_to_20.5N_at_original_birth',
        formal_capture_and_observation_timing='speed_priority_1s_grip_0.5s_release_1s_terminal_30s_spin',
        loading_table_force_gap_policy='original_20ms_positive_force_history_with_actual_table_contact_and_fresh_native_closed_goal',
        retrieval_preload_support='accepted_current_16.5N_fresh_native_original_opposed_midband_loads_with_original_bucket_containment',
        retrieval_weak_preload_gap='original_20ms_basket_positive_history_with_fresh_native_actual_opposed_midband_positive_contacts_not_grip_certificate',
        gripper_pid_d=6., maximum_native_gripper_velocity_m_s=.6, original_model_gripper_velocity_m_s=.2, software_output_guard_N=18.01,
        original_hold_release_steady_spin_sim_s=[10, 3, 30], no_current_physical_success_claimed=True,
        old_paired_successes_and_models_not_overwritten=True, sha256=members)
    with (ROOT / 'identity.json').open('x') as stream:
        json.dump(identity, stream, indent=2)
    for relative, expected in members.items():
        path = ROOT / relative
        assert digest(path) == expected
        path.chmod(0o444)
    (ROOT / 'identity.json').chmod(0o444)
    print('SEQUENTIAL_ONCE_SEALED_NOT_LAUNCHED', len(members), len(contract['selected_resources']),
          digest(ROOT / 'identity.json'), flush=True)


if __name__ == '__main__':
    main()
