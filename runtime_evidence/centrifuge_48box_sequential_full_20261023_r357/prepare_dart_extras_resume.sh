#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
for name in bucket_cache preload_observation pose_cache batched_trace runtime_performance initialization_compatibility speed_priority speed_retrieval speed_retrieval_regression geometry_fastpath right_setdown_stability gentle_pilot bounded_release localizer_pipeline axial_pilot axial_transport table_alignment; do
    run_dev "$root/qualify_${name}.py" > "$root/dart_${name}_qualification_revision2.log" 2>&1
done
run_dev -m pytest -q "$root" > "$root/tests_speed_final.log" 2>&1
run_dev "$root/audit_scene_dependencies.py" audit > "$root/dependency_audit.log" 2>&1
printf 'DART_EXTRAS_QUALIFIED_NOT_SEALED_OR_RUNNING\n'
