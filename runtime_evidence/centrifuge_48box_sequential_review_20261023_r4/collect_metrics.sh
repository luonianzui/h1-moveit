#!/usr/bin/env bash
set -euo pipefail
set -C
root="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261023_r357"
review="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_review_20261023_r4"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
bash "$root/run_dev.sh" "$review/observe_runtime.py" summarize > "$review/cpu_summary.log" 2>&1
for helper in analyze_native_profile audit_native_symbols audit_native_impulses audit_dart_samples summarize_world summarize_performance phase_performance audit_inertia_effect; do
  bash "$root/run_dev.sh" "$review/${helper}.py" > "$review/${helper}.log" 2>&1
done
printf 'ACTUAL_PERFORMANCE_COLLECTED\n'

