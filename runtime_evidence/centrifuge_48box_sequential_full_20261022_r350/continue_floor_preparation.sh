#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261022_r350"
cd "${workspace}"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
run_dev() { bash "${evidence}/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "${evidence}/identity.json"
run_dev -m pytest -q "${evidence}" > "${evidence}/tests_initial_qualification_corrected.log" 2>&1
run_dev -m pytest -q src/h1_2_moveit_config/test > "${evidence}/package_tests.log" 2>&1
run_dev -m pytest -q runtime_evidence/centrifuge_48box_full_cycle_20261009_r178 > "${evidence}/loading_regression_tests_final.log" 2>&1
set +e
run_dev -m pytest -q src/h1_2_navigation/test --junitxml="${evidence}/navigation_tests.xml" > "${evidence}/navigation_tests.log" 2>&1
navigation_status=$?
set -e
printf '%s\n' "${navigation_status}" > "${evidence}/navigation_exit_code.txt"
ctest --test-dir "${evidence}/build/unique_prefix/h1_2_moveit_config" --output-on-failure > "${evidence}/ctest.log" 2>&1
run_dev "${evidence}/qualify_sequential_dispatch.py" sequential_dispatch_revision3 > "${evidence}/dispatch_revision3.log" 2>&1
run_dev "${evidence}/qualify_neutral.py" > "${evidence}/neutral.log" 2>&1
run_dev "${evidence}/probe_sequential_chain.py" sequential_chain_final.json > "${evidence}/chain_final.log" 2>&1
run_dev "${evidence}/qualify_load_reserve.py" > "${evidence}/load_reserve_qualification.log" 2>&1
run_dev "${evidence}/qualify_floor_frame.py" > "${evidence}/floor_frame_qualification.log" 2>&1
run_dev "${evidence}/qualify_centered_pilot.py" > "${evidence}/centered_pilot_qualification.log" 2>&1
run_dev "${evidence}/qualify_release_alignment.py" > "${evidence}/release_alignment_qualification.log" 2>&1
run_dev "${evidence}/qualify_axis_exit.py" > "${evidence}/axis_exit_qualification.log" 2>&1
run_dev "${evidence}/fixture_actual_opening.py" > "${evidence}/actual_opening_fixture.log" 2>&1
run_dev "${evidence}/audit_dual_buttons.py" > "${evidence}/buttons_audit.log" 2>&1
run_dev "${evidence}/codec_preflight.py" > "${evidence}/codec_preflight_stdout.log" 2>&1
run_dev "${evidence}/full_capture_preflight.py" > "${evidence}/full_capture_preflight.log" 2>&1
run_dev "${evidence}/archive_runtime_binding.py" > "${evidence}/archive_runtime_binding.log" 2>&1
run_dev "${evidence}/verify_dual_protection.py" before > "${evidence}/before_protection.log" 2>&1
run_dev "${evidence}/verify_current_protection.py" before > "${evidence}/current_protection_before.log" 2>&1
run_dev "${evidence}/verify_startup_protection.py" before > "${evidence}/startup_protection_before.log" 2>&1
run_dev -m pytest -q "${evidence}" > "${evidence}/tests_final_qualification.log" 2>&1
printf 'SEQUENTIAL_PREPARED_NOT_SEALED_OR_RUNNING\n'
