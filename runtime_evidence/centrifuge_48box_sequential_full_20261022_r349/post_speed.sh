#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r349"
cd "${workspace}"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
run_dev() { bash "${evidence}/run_dev.sh" "$@"; }
deadline=$((SECONDS + 7200))
until test -f "${evidence}/run/process_exit_status.json"; do
  test "${SECONDS}" -lt "${deadline}"
  sleep 10
done
for name in audit_trace audit_pipeline audit_planner_resources audit_intervals audit_bounded_release audit_preload_observation audit_bucket_cache audit_localization_delivery audit_localizer_pipeline audit_table_alignment audit_speed_cycle audit_speed_grip audit_speed_physical summarize_actual_phases; do
  set +e
  run_dev "${evidence}/${name}.py" > "${evidence}/post_${name}.log" 2>&1
  status=$?
  set -e
  printf '%s %s\n' "${name}" "${status}"
done
if run_dev -c 'import json,sys; from pathlib import Path; sys.exit(json.loads(Path(sys.argv[1]).read_text())["status"]!="PASS")' "${evidence}/run/result.json"; then
  identity_hash="$(sha256sum "${evidence}/identity.json")"
  identity_hash="${identity_hash%% *}"
  set +e
  run_dev "${evidence}/audit_speed_paths.py" "${evidence}" "${identity_hash}" > "${evidence}/post_audit_speed_paths.log" 2>&1
  status=$?
  set -e
  printf 'actual_paths %s\n' "${status}"
fi
run_dev "${evidence}/verify_dual_protection.py" after > "${evidence}/protection_after.log" 2>&1
run_dev "${evidence}/verify_current_protection.py" after > "${evidence}/current_protection_after.log" 2>&1
run_dev "${evidence}/verify_startup_protection.py" after > "${evidence}/startup_protection_after.log" 2>&1
printf 'SPEED_POST_AUDITS_AND_PROTECTION_COMPLETED\n'
