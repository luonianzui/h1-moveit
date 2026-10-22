#!/bin/bash
# 将立方体从放置位置抓回原始位置的演示脚本

set -e

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"

exec ros2 launch h1_2_moveit_config pick_place_return.launch.py
