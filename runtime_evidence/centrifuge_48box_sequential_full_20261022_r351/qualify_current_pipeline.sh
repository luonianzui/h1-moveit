#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
export ROS_DOMAIN_ID=159 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=h1_seq_full_r351
run_dev -m pytest -q "$root/test_bounded_release_alignment.py" > "$root/bounded_release_tests_revision2.log" 2>&1
run_dev -m pytest -q "$root/test_planning_pipeline.py" > "$root/pipeline_test_revision2.log" 2>&1
for name in pipeline right_transfer localization_delivery bucket_cache preload_observation pose_cache batched_trace runtime_performance initialization_compatibility speed_priority speed_retrieval speed_retrieval_regression; do
    run_dev "$root/qualify_${name}.py" > "$root/current_${name}_qualification.log" 2>&1
done
printf 'CURRENT_PIPELINE_QUALIFIED_NOT_RUNNING\n'
