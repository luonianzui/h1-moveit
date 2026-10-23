#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261023_r356"
cd "${workspace}"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
run_dev() { bash "${evidence}/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "${evidence}/identity.json"
run_dev "${evidence}/qualify_record_dispatch.py" > "${evidence}/record_dispatch_qualification_revision2.log" 2>&1
run_dev "${evidence}/qualify_pilot_settle.py" > "${evidence}/pilot_settle_qualification.log" 2>&1
run_dev "${evidence}/qualify_single_setdown.py" > "${evidence}/single_setdown_qualification.log" 2>&1
run_dev "${evidence}/qualify_whole_single_transport.py" > "${evidence}/whole_single_transport_qualification.log" 2>&1
run_dev "${evidence}/qualify_right_setdown.py" > "${evidence}/right_setdown_qualification.log" 2>&1
run_dev "${evidence}/qualify_return_height.py" > "${evidence}/return_height_qualification.log" 2>&1
run_dev "${evidence}/qualify_level_pilot.py" > "${evidence}/level_pilot_qualification.log" 2>&1
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
