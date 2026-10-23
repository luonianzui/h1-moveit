#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "$root/identity.json"
export ROS_DOMAIN_ID=166 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=h1_seq_full_r358
for name in localization_delivery bucket_cache preload_observation pose_cache batched_trace runtime_performance initialization_compatibility speed_priority speed_retrieval speed_retrieval_regression; do
    run_dev "$root/qualify_${name}.py" > "$root/resumed_${name}_qualification.log" 2>&1
done
for name in geometry_fastpath right_setdown_stability gentle_pilot bounded_release localizer_pipeline axial_pilot axial_transport table_alignment intervals planner_resources initial_button_motion telemetry_transport; do
    run_dev "$root/qualify_${name}.py" > "$root/extra_${name}_qualification.log" 2>&1
done
run_dev "$root/verify_r353_protection.py" before > "$root/r353_protection_before.log" 2>&1
run_dev -m pytest -q "$root" > "$root/tests_speed_final.log" 2>&1
run_dev -m pytest -q "$root" > "$root/tests_final_qualification.log" 2>&1
run_dev "$root/audit_scene_dependencies.py" audit > "$root/dependency_audit.log" 2>&1
printf 'R356_ALL_PREPARED_NOT_SEALED_NOT_LAUNCHED\n'
