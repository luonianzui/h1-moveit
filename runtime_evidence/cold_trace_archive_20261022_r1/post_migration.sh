#!/usr/bin/env bash
set -euo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
audit="${workspace}/runtime_evidence/cold_trace_archive_20261022_r1"
candidate="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261022_r351"
cd "${workspace}"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
/usr/bin/python3 "${audit}/verify_archive.py" > "${audit}/archive_verification_stdout.log" 2>&1
/usr/bin/python3 "${audit}/verify_runtime.py" after > "${audit}/protection_after.log" 2>&1
/usr/bin/python3 "${audit}/verify_bindings.py" > "${audit}/bindings_after.log" 2>&1
bash "${candidate}/run_dev.sh" -m pytest -q -p no:cacheprovider "${candidate}" > "${audit}/r351_regression.log" 2>&1
bash "${candidate}/run_dev.sh" -m pytest -q -p no:cacheprovider runtime_evidence/centrifuge_48box_sequential_full_20261022_r347 > "${audit}/r347_regression.log" 2>&1
bash "${candidate}/run_dev.sh" -m pytest -q -p no:cacheprovider src/h1_2_moveit_config/test > "${audit}/configuration_regression.log" 2>&1
bash "${candidate}/run_dev.sh" -m pytest -q -p no:cacheprovider runtime_evidence/centrifuge_48box_full_cycle_20261009_r178 > "${audit}/loading_regression.log" 2>&1
printf 'COLD_MIGRATION_AND_EXISTING_RUNTIME_REGRESSIONS_PASS\n'
