#!/usr/bin/env bash
set -eo pipefail
set -C
evidence="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261023_r358"
bash "${evidence}/run_dev.sh" -m pytest -q "${evidence}" > "${evidence}/tests_final_qualification_revision2.log" 2>&1
printf 'SEQUENTIAL_PREPARED_NOT_SEALED_OR_RUNNING\n' >> "${evidence}/prepare_stdout.log"
printf 'FINAL_TEST_REVISION2_COMPLETE_OLD_FAILED_TEST_RETAINED\n'
