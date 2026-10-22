#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261022_r347"
cd "${workspace}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 PYTEST_ADDOPTS='-p no:cacheprovider'
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "${evidence}/identity.json"
test ! -e "${evidence}/run"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_lab_scene_48box_sequential_full_20261022_r347/local_setup.bash
export PYTHONPATH="${evidence}:${workspace}/src/h1_2_navigation:${workspace}/src/h1_2_moveit_config/scripts:${PYTHONPATH:-}"
/usr/bin/python3 "${evidence}/audit_inherited_inputs.py" > "${evidence}/prepare.log" 2>&1
/usr/bin/python3 "${evidence}/write_input_scope_erratum.py"
/usr/bin/python3 "${evidence}/gain_contract.py" > "${evidence}/gain_prerequisite.log" 2>&1
/usr/bin/python3 "${evidence}/bind_parent_calibration.py" > "${evidence}/parent_calibration_binding.log" 2>&1
/usr/bin/python3 "${evidence}/probe_current_bucket_exit.py" > "${evidence}/current_bucket_exit_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_current_micro_alignment.py" > "${evidence}/current_micro_alignment_probe.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}" > "${evidence}/tests_revision3.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_front_camera.py" > "${evidence}/front_camera_tests_final.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_minimum_turn.py" > "${evidence}/minimum_turn_tests_final.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_retrieval_grip.py" "${evidence}/test_software_force_guard.py" > "${evidence}/retrieval_grip_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_pickup_approach.py" > "${evidence}/pickup_approach_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_post_retreat_gripper.py" > "${evidence}/post_retreat_gripper_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_coarse_ramp.py" > "${evidence}/coarse_ramp_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_empty_protocol.py" > "${evidence}/empty_protocol_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_endpoint_overlap.py" "${evidence}/test_dense_tree.py" > "${evidence}/dense_endpoint_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_bucket_axis_exit.py" "${evidence}/test_lift_provider.py" > "${evidence}/lift_provider_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_optional_retreat.py" "${evidence}/test_micro_alignment.py" > "${evidence}/optional_micro_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_schedule_gripper.py" > "${evidence}/schedule_gripper_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_native_gripper_feedback.py" > "${evidence}/native_gripper_feedback_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_dense_shortcut.py" > "${evidence}/dense_shortcut_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_dense_fixture_scope.py" > "${evidence}/dense_fixture_scope_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_gentle_setdown.py" > "${evidence}/gentle_setdown_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_initial_approach.py" > "${evidence}/initial_approach_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_startup_ramp.py" > "${evidence}/startup_ramp_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_complete_rate_profile.py" > "${evidence}/complete_profile_dispatch_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_lid_rate_fork.py" > "${evidence}/private_lid_rate_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_button_ramp.py" > "${evidence}/button_ramp_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_retrieval_rate_restore.py" > "${evidence}/retrieval_rate_restore_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_loaded_dispatch.py" > "${evidence}/loaded_dispatch_tests.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}/test_dual_executor.py" "${evidence}/test_dual_motion.py" \
  "${evidence}/test_contact_phase_policy.py" "${evidence}/test_dual_speed.py" \
  "${evidence}/test_independent_dual_grip.py" "${evidence}/test_camera_retry.py" > "${evidence}/dual_executor_tests.log" 2>&1
/usr/bin/python3 -m pytest -q src/h1_2_moveit_config/test > "${evidence}/package_tests.log" 2>&1
/usr/bin/python3 -m pytest -q runtime_evidence/centrifuge_48box_full_cycle_20261009_r178 > "${evidence}/loading_regression_tests_final.log" 2>&1
set +e
/usr/bin/python3 -m pytest -q src/h1_2_navigation/test --junitxml="${evidence}/navigation_tests.xml" > "${evidence}/navigation_tests.log" 2>&1
navigation_status=$?
set -e
printf '%s\n' "${navigation_status}" > "${evidence}/navigation_exit_code.txt"
ctest --test-dir "${evidence}/build/unique_prefix/h1_2_moveit_config" --output-on-failure > "${evidence}/ctest.log" 2>&1
/usr/bin/python3 "${evidence}/archive_runtime_binding.py" > "${evidence}/archive_runtime_binding.log" 2>&1
/usr/bin/python3 -m pytest -q "${evidence}" > "${evidence}/post_storage_tests.log" 2>&1
/usr/bin/python3 "${evidence}/verify_dual_protection.py" before > "${evidence}/before_protection.log" 2>&1
/usr/bin/python3 "${evidence}/fixture_actual_opening.py" > "${evidence}/actual_opening_fixture.log" 2>&1
/usr/bin/python3 "${evidence}/audit_dual_buttons.py" > "${evidence}/buttons_audit.log" 2>&1
/usr/bin/python3 "${evidence}/codec_preflight.py" > "${evidence}/codec_preflight_stdout.log" 2>&1
/usr/bin/python3 "${evidence}/resize_capture_preflight.py" > "${evidence}/resize_preflight_stdout.log" 2>&1
/usr/bin/python3 "${evidence}/full_capture_preflight.py" > "${evidence}/full_capture_preflight_stdout.log" 2>&1
/usr/bin/python3 "${evidence}/audit_skip_return_offline.py" > "${evidence}/skip_return_offline.log" 2>&1
/usr/bin/python3 "${evidence}/audit_free_carry_offline.py" > "${evidence}/free_carry_offline.log" 2>&1
/usr/bin/python3 "${evidence}/probe_actual_guided_retreat_checked.py" > "${evidence}/actual_guided_retreat_checked.log" 2>&1
/usr/bin/python3 "${evidence}/probe_actual_paired_setdown.py" > "${evidence}/actual_paired_setdown_split.log" 2>&1
/usr/bin/python3 "${evidence}/probe_actual_table_clearance.py" > "${evidence}/actual_table_clearance_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_medium_retrieval.py" > "${evidence}/medium_retrieval_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_dense_retrieval.py" > "${evidence}/dense_retrieval_actual_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_dense_tree_actual.py" > "${evidence}/dense_tree_current_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_gentle_setdown.py" > "${evidence}/gentle_setdown_probe.log" 2>&1
/usr/bin/python3 "${evidence}/probe_initial_approach.py" > "${evidence}/initial_approach_probe.log" 2>&1
/usr/bin/python3 "${evidence}/audit_stop_handover.py" > "${evidence}/stop_handover_fixture_audit.log" 2>&1
/usr/bin/python3 "${evidence}/probe_post_retreat_gripper.py" > "${evidence}/post_retreat_gripper_probe.log" 2>&1
/usr/bin/python3 "${evidence}/audit_post_retreat_margins.py" > "${evidence}/post_retreat_margin_audit.log" 2>&1
printf 'MINIMUM_TURN_REGRESSIONS_PREPARED_NOT_SEALED_OR_RUNNING\n'
