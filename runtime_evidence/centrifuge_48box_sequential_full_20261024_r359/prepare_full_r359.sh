#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test ! -e "$root/identity.json"
bash "$root/run_dev.sh" "$root/verify_r357_protection.py" before > "$root/r357_protection_before.log" 2>&1
bash "$root/run_dev.sh" "$root/verify_r358_protection.py" before > "$root/r358_protection_before.log" 2>&1
bash "$root/run_dev.sh" "$root/qualify_centered_exit.py" > "$root/centered_exit_qualification.log" 2>&1
bash "$root/prepare_full_r357.sh"
printf 'R359_PREPARED_WITH_CENTERED_EXIT_NOT_SEALED_NOT_LAUNCHED\n'
