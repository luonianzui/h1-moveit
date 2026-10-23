#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
export ROS_DOMAIN_ID=166 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=h1_seq_full_r358
test ! -e "$root/identity.json"
bash "$root/qualify_current_pipeline.sh" > "$root/current_pipeline_stdout.log" 2>&1
for name in geometry_fastpath right_setdown_stability gentle_pilot bounded_release localizer_pipeline axial_pilot axial_transport table_alignment intervals planner_resources initial_button_motion telemetry_transport; do
    run_dev "$root/qualify_${name}.py" > "$root/extra_${name}_qualification.log" 2>&1
done
printf 'CURRENT_EXTRAS_READY_NOT_SEALED_OR_RUNNING\n'
