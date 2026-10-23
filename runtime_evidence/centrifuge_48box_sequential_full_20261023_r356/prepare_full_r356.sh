#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
run_dev() { bash "$root/run_dev.sh" "$@"; }
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "$root/identity.json"
run_dev "$root/bind_dart_cache.py" > "$root/dart_binding.log" 2>&1
run_dev "$root/bind_inertia_headers.py" > "$root/inertia_headers.log" 2>&1
run_dev "$root/qualify_unilateral_observation.py" > "$root/unilateral_qualification.log" 2>&1
run_dev "$root/verify_r354_protection.py" before > "$root/r354_protection_before.log" 2>&1
bash "$root/prepare_sequential.sh" > "$root/prepare_stdout.log" 2>&1
bash "$root/prepare_current_extras.sh" > "$root/extras_stdout.log" 2>&1
run_dev "$root/verify_r353_protection.py" before > "$root/r353_protection_before.log" 2>&1
run_dev -m pytest -q "$root" > "$root/tests_speed_final.log" 2>&1
run_dev -m pytest -q "$root" > "$root/tests_final_qualification.log" 2>&1
run_dev "$root/audit_scene_dependencies.py" audit > "$root/dependency_audit.log" 2>&1
printf 'R356_ALL_PREPARED_NOT_SEALED_NOT_LAUNCHED\n'
