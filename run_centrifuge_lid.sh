#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
source "${workspace}/install_lab_scene_48box_20260923_r122/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
exec ros2 run h1_2_moveit_config centrifuge_lid.py "$@"
