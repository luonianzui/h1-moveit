#!/usr/bin/env bash
set -e

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
scene="${1:-centrifuge}"
case "${scene}" in
  centrifuge|incubator) ;;
  *) printf 'Usage: %s [centrifuge|incubator]\n' "$0" >&2; exit 2 ;;
esac
demo=false
with_48box=false
if (( $# > 0 )); then shift; fi
for option in "$@"; do
  case "${scene}:${option}" in
    centrifuge:--open-lid-demo) demo=true ;;
    centrifuge:--48box) with_48box=true ;;
    *) printf 'Usage: %s [centrifuge|incubator] [--open-lid-demo] [--48box]\n' "$0" >&2; exit 2 ;;
  esac
done
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
source "${workspace}/install_lab_scene_48box_20260923_r122/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch h1_2_moveit_config lab_scene.launch.py "scene:=${scene}" "allow_open_lid_demo:=${demo}" "with_48box:=${with_48box}"
