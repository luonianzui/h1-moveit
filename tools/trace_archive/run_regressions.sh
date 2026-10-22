#!/usr/bin/env bash
# Offline tests only. Invoke through tools/trace_archive/run after migration.
set -eo pipefail
trace_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${trace_workspace}"
trace_audit="${trace_workspace}/runtime_evidence/trace_archive_20261014_r1"
trace_candidate="${trace_workspace}/runtime_evidence/centrifuge_48box_initial_approach_20261014_r273"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_lab_scene_48box_initial_approach_20261014_r273/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="${trace_candidate}:${trace_workspace}/src/h1_2_navigation:${trace_workspace}/src/h1_2_moveit_config/scripts:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q -p no:cacheprovider "${trace_candidate}" > "${trace_audit}/r273_regression.log" 2>&1
/usr/bin/python3 -m pytest -q -p no:cacheprovider runtime_evidence/centrifuge_48box_full_cycle_20261009_r178 > "${trace_audit}/r178_regression.log" 2>&1
/usr/bin/python3 -m pytest -q -p no:cacheprovider src/h1_2_moveit_config/test > "${trace_audit}/configuration_regression.log" 2>&1
printf '%s\n' OFFLINE_REGRESSIONS_PASS
