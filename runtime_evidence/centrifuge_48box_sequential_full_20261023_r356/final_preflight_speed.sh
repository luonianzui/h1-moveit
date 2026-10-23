#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261023_r356"
cd "${workspace}"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
run_dev() { bash "${evidence}/run_dev.sh" "$@"; }
deadline=$((SECONDS + 2400))
until grep -q SEQUENTIAL_PREPARED_NOT_SEALED_OR_RUNNING "${evidence}/prepare_sequential_stdout.log"; do
  test "${SECONDS}" -lt "${deadline}"
  sleep 10
done
run_dev -m pytest -q "${evidence}" > "${evidence}/tests_speed_final.log" 2>&1
run_dev "${evidence}/audit_scene_dependencies.py" audit > "${evidence}/scene_dependency_speed.log" 2>&1
run_dev "${evidence}/seal_sequential.py" > "${evidence}/seal_sequential.log" 2>&1
bash "${evidence}/start_front_recording.sh" > "${evidence}/startup_front_recording.log" 2>&1
printf 'SPEED_FIRST_FRAME_READY_FOR_ACTUAL_REVIEW\n'
