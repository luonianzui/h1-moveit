#!/usr/bin/env bash
set -eo pipefail

workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
runtime_dir="/home/liu/h1_moveit/runtime_logs/incubator_sim_direct_fast40"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash"
set -u

mkdir -p "${runtime_dir}"
exec 9>"${runtime_dir}/active.lock"
if ! flock -n 9; then
  echo "[sim-direct-fast40] Another fast40 wrapper owns the run lock; no command sent." >&2
  exit 2
fi

if ! pgrep -f 'gz sim server' >/dev/null; then
  echo "[sim-direct-fast40] Gazebo server is absent; no process was started." >&2
  exit 2
fi
if pgrep -f '/h1_2_moveit_demo/incubator_door_open_demo([[:space:]]|$)' >/dev/null; then
  echo "[sim-direct-fast40] incubator_door_open_demo is active; no command sent." >&2
  exit 2
fi
if ! timeout 3 gz model -m incubator -p >/dev/null 2>&1; then
  echo "[sim-direct-fast40] Existing Gazebo has no healthy incubator model; no command sent." >&2
  exit 2
fi
if ! timeout 5 ros2 topic echo --once /incubator/joint_states >/dev/null 2>&1; then
  echo "[sim-direct-fast40] No fresh incubator joint feedback; no command sent." >&2
  exit 2
fi

echo "[sim-direct-fast40] EXPERIMENTAL SIMULATION ONLY."
echo "[sim-direct-fast40] Candidate 3 is a parameter gate; no robot IK/path or ROS controller trajectory is used."
echo "[sim-direct-fast40] The original handle uses a 92 deg compensated command (feedback near 90), then the original door moves to 40 deg."

exec ros2 run h1_2_moveit_demo incubator_sim_direct_fast40_demo --ros-args \
  -p use_sim_time:=true \
  -p door_handoff_only_candidate:=3 \
  -p door_handoff_simulation_direct_joint_fast40:=true
