import hashlib
import json
from pathlib import Path
from identity_members import VALIDATION_FILES, is_identity_member
from scene_contract import read_contract

evidence = Path(__file__).resolve().parent
root = evidence.parents[1]
assert json.loads((evidence / 'codec_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'skip_return_offline_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'resize_capture_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'free_carry_offline_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'full_window_capture_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'servo_fork_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'parent_servo_calibration_audit.json').read_text())['status'] == 'PASS'
gain_calibration = json.loads((evidence / 'calibrated_gain_audit.json').read_text())
assert gain_calibration['status'] == 'PASS' and gain_calibration['position_proportional_gain'] == .6
assert gain_calibration['loaded_motion_not_preclaimed'] and gain_calibration['fresh_complete_physical_execution_required']
release_probes = json.loads((evidence / 'release_descent_probe.json').read_text())
assert all(any(row['side'] == side and row['distance_m'] == .045 and row['result']['status'] == 'PASS'
               for row in release_probes['records']) for side in ('left', 'right'))
baseline = root / 'runtime_evidence/centrifuge_48box_demo_complete_20261010_r214'
assert (baseline / 'SHA256SUMS').is_file()
assert json.loads((baseline / 'cycle_audit.json').read_text())['status'] == 'PASS'
for name in ('protected_before.json', 'source_1009_before.json', 'buttons_collision_audit.json', 'control_fork_audit.json'):
    assert json.loads((evidence / name).read_text())['status'] == 'PASS'
single = root / 'runtime_evidence/centrifuge_48box_single_extraction_20261010_r190'
assert (single / 'SHA256SUMS').is_file()
assert json.loads((single / 'single_audit.json').read_text())['status'] == 'PASS'
first = root / 'runtime_evidence/centrifuge_48box_soft_grasp_20261010_r194'
assert (first / 'SHA256SUMS').is_file()
assert json.loads((first / 'single_audit.json').read_text())['status'] == 'PASS'
dual = root / 'runtime_evidence/centrifuge_48box_setdown_braking_20261010_r205'
assert (dual / 'SHA256SUMS').is_file()
assert json.loads((dual / 'unloading_audit.json').read_text())['status'] == 'PASS'
local = root / 'runtime_evidence/centrifuge_48box_pilot_braking_20261010_r208'
assert (local / 'SHA256SUMS').is_file()
assert json.loads((local / 'unloading_audit.json').read_text())['status'] == 'PASS'
assert json.loads((local / 'pilot_contract_audit.json').read_text())['status'] == 'PASS'
for variant in ('baseline', 'brake'):
    assert json.loads((root / 'runtime_evidence/centrifuge_48box_position_brake_20261010_r189' / ('run_' + variant) / 'audit.json').read_text())['status'] == 'PASS'
assert (evidence / 'conda_six_before.txt').read_bytes() == (evidence / 'conda_six_after.txt').read_bytes()
contract = read_contract(evidence)
assert json.loads((evidence / 'dual_layout_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'paired_fixture_actual_opening_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'actual_guided_retreat_audit_checked.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'actual_paired_setdown_split_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'actual_table_clearance_probe.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'medium_retrieval_probe.json').read_text())['status'] == 'PASS'
handover = json.loads((evidence / 'stop_handover_fixture_audit.json').read_text())
assert handover['status'] == 'PASS' and handover['new_actual_worker_still_required']
assert json.loads((evidence / 'dual_protected_before.json').read_text())['status'] == 'PASS'
assert ' passed' in (evidence / 'dual_executor_tests.log').read_text()
dispatch_tests = (evidence / 'loaded_dispatch_tests.log').read_text()
assert '8 passed' in dispatch_tests and ' failed' not in dispatch_tests and 'ERROR' not in dispatch_tests
profile_tests = (evidence / 'complete_profile_dispatch_tests.log').read_text()
assert '129 passed' in profile_tests and ' failed' not in profile_tests and 'ERROR' not in profile_tests
lid_fork = json.loads((evidence / 'private_lid_rate_fork_audit.json').read_text())
assert lid_fork['status'] == 'PASS' and lid_fork['candidate_lid_velocity_rad_s'] == .8
assert lid_fork['unchanged_upper_angle_deg'] == 120. and lid_fork['unchanged_joint_effort_Nm'] == 30.
lid_tests = (evidence / 'private_lid_rate_tests.log').read_text()
assert '8 passed' in lid_tests and ' failed' not in lid_tests and 'ERROR' not in lid_tests
ramp_tests = (evidence / 'button_ramp_tests.log').read_text()
assert ' passed' in ramp_tests and ' failed' not in ramp_tests and 'ERROR' not in ramp_tests
restore_tests = (evidence / 'retrieval_rate_restore_tests.log').read_text()
assert ' passed' in restore_tests and ' failed' not in restore_tests and 'ERROR' not in restore_tests
schedule_tests = (evidence / 'schedule_gripper_tests.log').read_text()
assert ' passed' in schedule_tests and ' failed' not in schedule_tests and 'ERROR' not in schedule_tests
native_feedback_tests = (evidence / 'native_gripper_feedback_tests.log').read_text()
assert ' passed' in native_feedback_tests and ' failed' not in native_feedback_tests and 'ERROR' not in native_feedback_tests
dense_tests = (evidence / 'dense_shortcut_tests.log').read_text()
assert ' passed' in dense_tests and ' failed' not in dense_tests and 'ERROR' not in dense_tests
fixture_scope_tests = (evidence / 'dense_fixture_scope_tests.log').read_text()
assert '8 passed' in fixture_scope_tests and ' failed' not in fixture_scope_tests and 'ERROR' not in fixture_scope_tests
gentle_tests = (evidence / 'gentle_setdown_tests.log').read_text()
initial_tests = (evidence / 'initial_approach_tests.log').read_text()
startup_tests = (evidence / 'startup_ramp_tests.log').read_text()
assert '18 passed' in startup_tests and ' failed' not in startup_tests and 'ERROR' not in startup_tests
assert json.loads((evidence / 'startup_ramp_model_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'archive_runtime_dependencies.json').read_text())['status'] == 'PASS'
post_storage_tests = (evidence / 'post_storage_tests.log').read_text()
assert ' passed' in post_storage_tests and ' failed' not in post_storage_tests and 'ERROR' not in post_storage_tests
grip_tests = (evidence / 'retrieval_grip_tests.log').read_text()
assert ' passed' in grip_tests and ' failed' not in grip_tests and 'ERROR' not in grip_tests
pickup_tests = (evidence / 'pickup_approach_tests.log').read_text()
assert ' passed' in pickup_tests and ' failed' not in pickup_tests and 'ERROR' not in pickup_tests
assert '17 passed' in initial_tests and ' failed' not in initial_tests and 'ERROR' not in initial_tests
initial_probe = json.loads((evidence / 'initial_approach_probe_audit.json').read_text())
assert initial_probe['status'] == 'PASS' and initial_probe['candidate_rate_rad_s'] == .7
assert initial_probe['all_original_points_actual_seed_groups_and_floor_preserved']
assert '18 passed' in gentle_tests and ' failed' not in gentle_tests and 'ERROR' not in gentle_tests
gentle_probe = json.loads((evidence / 'gentle_setdown_probe_audit.json').read_text())
assert gentle_probe['status'] == 'PASS' and gentle_probe['candidate_coarse_rate_rad_s'] == .30
assert gentle_probe['all_original_points_and_actual_seed_preserved']
assert gentle_probe['original_native_grip_limits_not_relaxed']
dense_audit = json.loads((evidence / 'dense_retrieval_actual_probe_audit.json').read_text())
assert dense_audit['status'] == 'PASS' and dense_audit['only_shortcut_sampling_changed_rad'] == .002
assert dense_audit['original_final_full_state_sampling_unchanged_rad'] == .002
assert dense_audit['actual_fresh_online_scene_and_post_grasp_plan_still_required']
assert contract['regression']['scene_dependencies_passed']
assert contract['full_hand_dependency_reviewed']
for name in ('tests_revision3.log', 'package_tests.log', 'loading_regression_tests_final.log'):
    log = (evidence / name).read_text()
    assert ' passed' in log and ' failed' not in log and 'ERROR' not in log
assert not (evidence / 'run/launch_pid.json').exists()
retreat_tests = (evidence / 'post_retreat_gripper_tests.log').read_text()
assert ' passed' in retreat_tests and ' failed' not in retreat_tests and 'ERROR' not in retreat_tests
assert json.loads((evidence / 'post_retreat_gripper_probe.json').read_text())['status'] == 'PASS'
revision_tests = (evidence / 'coarse_ramp_tests.log').read_text()
assert ' passed' in revision_tests and ' failed' not in revision_tests and 'ERROR' not in revision_tests
assert json.loads((evidence / 'post_retreat_margin_audit.json').read_text())['status'] == 'PASS'
assert json.loads((evidence / 'inherited_inputs_runtime_scope_erratum.json').read_text())['status'] == 'CURRENT_RUNTIME_SCOPE_CORRECTION'
protocol_tests = (evidence / 'empty_protocol_tests.log').read_text()
assert '22 passed' in protocol_tests and ' failed' not in protocol_tests and 'ERROR' not in protocol_tests
new_tests = (evidence / 'dense_endpoint_tests.log').read_text()
assert '18 passed' in new_tests and ' failed' not in new_tests and 'ERROR' not in new_tests
provider_tests = (evidence / 'lift_provider_tests.log').read_text()
assert ' passed' in provider_tests and ' failed' not in provider_tests and 'ERROR' not in provider_tests
assert json.loads((evidence / 'current_bucket_exit_probe.json').read_text())['status'] == 'PASS_OFFLINE_CURRENT_PROVIDER_NOT_PHYSICAL'
new_tests = (evidence / 'optional_micro_tests.log').read_text()
assert '26 passed' in new_tests and ' failed' not in new_tests and 'ERROR' not in new_tests
assert json.loads((evidence / 'current_micro_alignment_probe.json').read_text())['status'] == 'PASS_OFFLINE_CURRENT_HELPER_NOT_PHYSICAL'
records = {}
for path in sorted(evidence.iterdir()):
    if path.is_file() and is_identity_member(path):
        records[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
assert all(name in records for name in VALIDATION_FILES)
assert 'seal.log' not in records and 'launch.log' not in records
for path in sorted((evidence / 'inputs').iterdir()):
    if path.is_file():
        records[str(path.relative_to(evidence))] = hashlib.sha256(path.read_bytes()).hexdigest()
with (evidence / 'identity.json').open('x') as stream:
    json.dump(dict(identity=evidence.name, scope='two_box_load_close_spin_open_retrieve',
        manager_update_rate_hz=500, position_proportional_gain=.6, actual_runtime_subclass='EndpointOverlapCycle',
        parent_runtime_subclass='EndpointOverlapCycle',
        only_new_runtime_change_is_qualified_coarse_setdown_arc_timing_ramp=False,
        coarse_setdown_timing_ramp_sim_s=.6, all_original_joint_points_polyline_and_floor_preserved=False,
        no_continuous_acceleration_or_complete_180_certificate_from_arc_parameter=True,
        original_zero_force_empty_gripper_after_three_actual_retreats_before_final_transport=True,
        actual_gripper_stroke_FCL_and_fresh_after_endpoint_path_replan_required=True,
        only_new_runtime_change_is_empty_retrieval_approach_rate=False,
        declared_runtime_changes=['paired elbow-up symmetric empty grasp selection', 'bounded fixed-grasp loaded IK posture preference', 'empty initial approach cap 0.7 to 0.5 rad/s', 'loaded retrieval outboard and transfer cap 0.6 to 0.3 rad/s'],
        elbow_retrieval_transport_cap_rad_s=.3,
        startup_first_frame_review_paused_before_any_executor_or_grasp=False,
        startup_first_frame_review_owned_server_sigstop_sigcont_before_any_executor_or_grasp=True,
        fresh_window_after_review_resume_still_sim_s=25,
        elbow_empty_approach_cap_rad_s=.5,
        elbow_posture_contract='paired_elbow_up_fixed_grasp_v1',
        elbow_ik_refinement_joint_bound_rad=.045,
        original_cartesian_targets_and_actual_grip_reference_freeze_preserved=True,
        old_joint_path_minimum_turn_and_speed_certificates_not_inherited=True,
        inherited_bucket_axis_strict_tree_endpoint_overlap_and_empty_protocol_preserved=True,
        initial_empty_dummy_strokes_omitted=True, original_initial_open_native_precision_m=.00025,
        empty_post_action_idle_sim_s=.1, original_loaded_and_release_post_action_idle_sim_s=.3,
        fresh_bilateral_native_stamp_after_shared_result_before_empty_idle_required=True,
        parent_empty_retrieval_approach_cap_rad_s=.4, empty_retrieval_approach_cap_rad_s=.35,
        original_empty_initial_approach_rate_rad_s=.8,
        parent_empty_initial_approach_rate_rad_s=.7, empty_initial_approach_rate_rad_s=.5,
        initial_empty_approach_retiming_only_new_change=False,
        only_new_physics_change_is_private_startup_target_ramp=False,
        parent_startup_target_acceleration_rad_s2=.8, startup_target_acceleration_rad_s2=.8,
        only_new_runtime_change_is_qualified_paired_retrieval_feedforward=False,
        parent_retrieval_feedforward_N=18., retrieval_feedforward_N=18.,
        unchanged_gripper_software_output_guard_N=18.01,
        loading_feedforward_N=16., original_gripper_output_clamp_N=32., original_finger_contact_limit_N=40.,
        retrieval_grip_candidate_not_complete_or_180_certificate=True,
        original_stop_target_acceleration_rad_s2=1.,
        tested_private_startup_plugin_sha256='cb030abe1b549ac266132da3ba4cb5f78e460d015598d7be8bde81500d9dc8f1',
        fresh_before_executor_startup_ramp_status_field_readback_required=True,
        inherited_gentle_coarse_setdown_not_yet_physically_qualified=True,
        parent_loaded_coarse_setdown_rate_rad_s=.35,
        qualified_loaded_coarse_setdown_rate_rad_s=.30,
        only_loaded_retrieval_coarse_descent_rate_changed=False,
        original_immediate_native_grip_limit_m=.002,
        original_immediate_native_grip_angle_limit_deg=5.,
        original_fresh_actual_hold_references_not_rebased=True,
        empty_retrieval_pickup_shortcut_collision_sampling_rad=.002,
        original_rrt_tree_edge_collision_sampling_rad=.035, qualified_empty_retrieval_tree_edge_sampling_rad=.002,
        original_endpoint_precision_rad=.0003, original_native_endpoint_stability_sim_s=.2,
        original_post_trajectory_completion_margin_sim_s=.15, native_endpoint_feedback_overlap_only=False,
        inherited_native_endpoint_feedback_overlap_preserved=True,
        only_new_runtime_geometry_is_actual_paired_bucket_axis_exit=False,
        inherited_actual_bucket_axis_exit_preserved=True,
        exact_current_run_optional_alignment_reverse_stage_required=True,
        both_inside_reverse_requested_rate_rad_s=.13,
        qualified_optional_alignment_maximum_body_delta_m=.005,
        qualified_optional_alignment_maximum_body_rotation_deg=1.,
        qualified_optional_alignment_cartesian_targets=11,
        original_optional_alignment_requested_rate_rad_s=.10,
        bucket_axis_intermediate_rim_clearance_m=.006, original_required_extraction_clearance_m=.003,
        original_lift_cartesian_targets=81, original_lift_joint_points=82,
        original_lift_final_pose_and_world_orientation_unchanged=True,
        no_new_collision_exemption_or_grip_reference_rebase=True,
        original_final_empty_full_state_collision_sampling_rad=.002,
        dense_shortcut_only_for_declared_empty_actual_retrieval_pickup=True,
        original_rng_bounds_iterations_deadline_ik_targets_models_acm_preserved=True,
        frozen_failed_scene_probe_is_not_actual_fresh_regrasp_or_180_certificate=True,
        qualified_successful_gripper_action_requires_actual_native_stamp_after_shared_result_clock=True,
        original_gripper_endpoint_precision_m=.00025,
        native_gripper_feedback_maximum_additional_wait_sim_s=1.,
        native_gripper_feedback_maximum_additional_wait_wall_s=30.,
        original_unqualified_gripper_endpoint_assertion_preserved=True,
        qualified_command_future_start_offset_sim_s=.16, original_default_future_start_offset_sim_s=.3,
        minimum_actual_future_start_margin_sim_s=.15, explicit_physical_stop_schedule_unchanged=True,
        final_left_open_original_home_return_and_retrieval_path_policy_preserved=True,
        qualified_noncontact_button_ramp_time_sim_s=.1,
        qualified_noncontact_button_peak_rate_rad_s=1.2,
        qualified_noncontact_button_maximum_acceleration_rad_s2=12.,
        button_original_joint_polyline_and_every_original_zero_velocity_vertex_preserved=True,
        current_actual_two_arm_spline_readback_required=True,
        failed_unsmoothed_high_rate_button_profile_not_dispatched=True,
        complete_profile_rates_rad_s={'free': .6, 'pilot': .08, 'table_endpoint': .45,
            'basket_lowering': .14, 'full_lift': .4, 'coarse': .30, 'fine': .09,
            'empty_retrieval_approach': .4, 'empty_neutral': .6, 'empty_near_contact': .26,
            'qualified_empty_button_noncontact': 1.2},
        original_tracking_endpoint_geometry_floor_force_velocity_and_hold_gates_preserved=True,
        actual_complete_profile_three_group_and_button_publisher_path_tests_required=True,
        complete_profile_fake_publisher_tests_not_physical_certification=True,
        private_lid_joint_velocity_rad_s=.8, parent_private_lid_joint_velocity_rad_s=.8,
        original_lid_angle_force_rotor_contact_interlock_and_all_model_geometry_preserved=True,
        private_lid_rate_fork_audit_required=True,
        parent_actual_loaded_fine_setdown_rate_rad_s=.06,
        parent_gripper_pid_derivative_gain_preserved=6.,
        zero_final_effort_short_stroke_last_time_sim_s=.25,
        original_qualified_zero_final_effort_short_stroke_last_time_sim_s=.5,
        only_current_qualification_and_original_one_mm_four_finger_admission_allow_quarter_second=True,
        original_zero_final_effort_short_stroke_last_time_sim_s=1.,
        only_declared_phases_and_all_four_actual_fingers_within_one_mm_retimed=True,
        loaded_fine_setdown_rate_rad_s=.09, original_loaded_fine_setdown_rate_rad_s=.03,
        gripper_pid_derivative_gain=6., original_gripper_pid_derivative_gain=3.,
        original_feedback_force_clamps_N=[-2., 2.], current_damping_native_approach_required_before_loaded_speed=True,
        original_complete_dual_cycle_inherited=True, stop_home_return_skipped_only_after_actual_withdrawal=True,
        actual_full_model_left_open_handover_audit_required=True, horizontal_free_transport_rate_rad_s=.6,
        phase_specific_loaded_dispatch_cap_hook=True, original_default_loaded_dispatch_rate_rad_s=.25,
        actual_base_dispatch_publisher_path_unit_test_required=True,
        gain_calibration_parent_manifest_sha256=gain_calibration['calibration_manifest_sha256'],
        full_cycle_execution_certified=False, full_cycle_execution_required=True,
        experimental_fixed_initial_pelvis_x_m=.12, experimental_device_x_m=.52,
        maximum_measured_fresh_start_sim_s=25,
        original_leg_posture_targets_gains_and_goal_tolerance_unchanged=True,
        stationary_leg_initialization_duration_sim_s=1,
        original_stationary_leg_initialization_duration_sim_s=5,
        original_leg_endpoint_position_limit_rad=.001,
        fresh_actual_leg_feedback_required_after_action_result=True,
        leg_endpoint_maximum_additional_wait_sim_s=3.,
        leg_endpoint_maximum_additional_wait_wall_s=30.,
        paired_setdown_extra_seed_preserved_with_121_cartesian_targets=True,
        original_setdown_span_m=.3, original_fine_setdown_height_m=.01,
        empty_retrieval_approach_rate_cap_rad_s=.35,
        failed_global_arc_ramp_not_inherited_or_dispatched=True,
        prior_complete_r261_rate_not_current_physics_or_180_certificate=True,
        empty_machine_neutral_return_rate_cap_rad_s=.6,
        empty_machine_neutral_return_tracking_limit_rad=.002,
        empty_retrieval_approach_tracking_limit_rad=.002,
        gripper_model_velocity_limit_m_s=.2, original_velocity_fault_guard_not_relaxed=True,
        terminal_camera_maximum_attempts=3,
        gui_camera_request_timeout_wall_s=3,
        gui_camera_subprocess_timeout_wall_s=5,
        gui_camera_shutdown_join_timeout_wall_s=16,
        terminal_inspection_failure_still_finishes_video_before_scene_retirement=True,
        scene_retirement_requires_recorder_exit_verified=True,
        table_only_clearance_from_both_original_collision_buckets_m=.003,
        vertical_full_extraction_clearance_m=.003, table_clearance_independently_recomputed=True,
        actual_two_holder_registry_required=True, common_arm_timestamps_required=True,
        empty_retreat_from_actual_released_scene_and_current_run_executed_guides=True,
        rejected_fixed_wrist_orientation_retreat_not_used=True,
        no_empty_hand_loose_payload_finger_collision_exemptions=True,
        table_payload_birth_pose_guard_m=.0003, table_payload_birth_angle_guard_deg=.2,
        near_contact_descent_rate_cap_rad_s=.26, near_contact_tracking_limit_rad=.002,
        finite_wall_budget_seconds_per_sim_second=50, finite_wall_budget_bounds_s=[60, 900],
        command_start_and_completion_margin_sim_s=.31, original_wall_budget_margin_sim_s=.45,
        real_clock_progress_guard_wall_s=2,
        original_wall_budget_allowance_preserved=True,
        simulation_motion_rate_changed_only_in_declared_complete_profile=True,
        actual_post_spin_paired_retrieval_plan_required=True,
        nominal_release_floor_gap_m=.023, release_descent_dense_FCL_and_actual_snapshot_required=True,
        coherent_actual_joint_bracket_no_extrapolation=True,
        immediate_latest_native_guard_unchanged=True,
        unchanged_hard_physical_guards=True, actual_loaded_speed_not_preclaimed=True,
        steady_rotation_sim_s=30, terminal_policy='hold_at_verified_last_release_no_final_park',
        phase_stop_after_at_least_30_sim_s_via_real_finger_required=True,
        old_fixed_30_3_stop_window_not_preclaimed=True,
        actual_stop_phase_and_grasp_tracking_not_proven_by_nominal_fixture=True,
        independent_fixed_pelvis_hand_contract=True, original_navigation_regression_passed=contract['regression']['original_regression_passed'],
        sha256=records, simulation_only=True, unchanged_historic_models=True), stream, indent=2)
print('SEALED', len(records))

