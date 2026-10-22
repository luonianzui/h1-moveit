#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
bash "$root/qualify_current_pipeline.sh" > "$root/current_pipeline_stdout.log" 2>&1
for name in geometry_fastpath right_setdown_stability gentle_pilot bounded_release localizer_pipeline axial_pilot axial_transport table_alignment; do
    run_dev "$root/qualify_${name}.py" > "$root/dart_${name}_qualification.log" 2>&1
done
run_dev -m pytest -q "$root" > "$root/tests_speed_final.log" 2>&1
run_dev "$root/qualify_intervals.py" > "$root/interval_qualification.log" 2>&1
run_dev "$root/audit_scene_dependencies.py" audit > "$root/dependency_audit.log" 2>&1
printf 'DART_EXTRAS_QUALIFIED_NOT_SEALED_OR_RUNNING\n'
