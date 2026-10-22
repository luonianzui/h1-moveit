#!/usr/bin/env bash
set -eo pipefail
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261022_r350"
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
archive_pythonpath="${PYTHONPATH:-}"
cd "${workspace}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_lab_scene_48box_sequential_full_20261022_r350/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 PYTEST_ADDOPTS='-p no:cacheprovider'
export PYTHONPATH="${evidence}:${workspace}/src/h1_2_navigation:${workspace}/src/h1_2_moveit_config/scripts:${archive_pythonpath}:${PYTHONPATH:-}"
/usr/bin/python3 -c 'from pathlib import Path; import os, sitecustomize; assert Path(sitecustomize.__file__).parent.name == "trace_archive"; assert Path(os.environ["H1_TRACE_ARCHIVE_REGISTRY"]).is_file()'
exec /usr/bin/python3 "$@"
