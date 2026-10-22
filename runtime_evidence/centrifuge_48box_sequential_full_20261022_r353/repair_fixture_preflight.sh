#!/usr/bin/env bash
set -eo pipefail
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r353"
cd "${workspace}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 PYTEST_ADDOPTS='-p no:cacheprovider'
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_lab_scene_48box_sequential_full_20261022_r353/local_setup.bash
export PYTHONPATH="${evidence}:${workspace}/src/h1_2_navigation:${workspace}/src/h1_2_moveit_config/scripts:${PYTHONPATH:-}"
test ! -e "${evidence}/tests_final.log"
test ! -e "${evidence}/paired_fixtures_actual_opening"
/usr/bin/python3 -m pytest -q "${evidence}" > "${evidence}/tests_final.log" 2>&1
/usr/bin/python3 "${evidence}/fixture_actual_opening.py" > "${evidence}/actual_opening_fixture.log" 2>&1
/usr/bin/python3 -c 'import json, pathlib; root=pathlib.Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r353"); assert json.loads((root/"paired_fixture_actual_opening_audit.json").read_text())["status"] == "PASS"'
printf 'FIXTURE_SCOPE_REPAIR_AND_FINAL_TESTS_PASS_NOT_FRESH_EXECUTION\n'

