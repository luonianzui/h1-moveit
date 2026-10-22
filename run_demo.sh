#!/usr/bin/env bash
set -e

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"

exec ros2 launch h1_2_moveit_config demo.launch.py use_rviz:=true run_demo:=true
