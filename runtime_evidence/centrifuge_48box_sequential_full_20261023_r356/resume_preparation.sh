#!/usr/bin/env bash
set -euo pipefail
set -C
root="$(cd "$(dirname "$0")" && pwd)"
test ! -e "$root/identity.json"
bash "$root/run_dev.sh" -m pytest -q "$root" > "$root/tests_final_qualification_revision2.log" 2>&1
printf 'DECLARED_BOUNDED_IMPORT_EQUIVALENCE_TESTS_UPDATED; ORIGINAL_FAILED_LOG_PRESERVED\nSEQUENTIAL_PREPARED_NOT_SEALED_OR_RUNNING\n' >> "$root/prepare_sequential_stdout.log"
