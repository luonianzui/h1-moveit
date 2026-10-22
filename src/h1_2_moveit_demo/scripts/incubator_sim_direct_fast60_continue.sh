#!/usr/bin/env bash
set -eo pipefail

workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
runtime_dir="/home/liu/h1_moveit/runtime_logs/incubator_sim_direct_fast40"
command_topic="/h1_2/incubator/simulation_direct_joint_trajectory"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash"
set -u

mkdir -p "${runtime_dir}"
exec 9>"${runtime_dir}/active.lock"
if ! flock -n 9; then
  echo "[sim-direct-fast60] Another direct-joint wrapper owns the run lock; no command sent." >&2
  exit 2
fi

if ! pgrep -f 'gz sim server' >/dev/null; then
  echo "[sim-direct-fast60] Gazebo server is absent; no process was started." >&2
  exit 2
fi
if pgrep -f '/h1_2_moveit_demo/incubator_door_open_demo([[:space:]]|$)' >/dev/null; then
  echo "[sim-direct-fast60] incubator_door_open_demo is active; no command sent." >&2
  exit 2
fi
if pgrep -f '/h1_2_moveit_demo/incubator_sim_direct_fast40_demo([[:space:]]|$)' >/dev/null; then
  echo "[sim-direct-fast60] Another simulation-direct demo is active; no command sent." >&2
  exit 2
fi
if ! timeout 3 gz model -m incubator -p >/dev/null 2>&1; then
  echo "[sim-direct-fast60] Existing Gazebo has no healthy incubator model; no command sent." >&2
  exit 2
fi
if ! timeout 5 ros2 topic echo --once /incubator/joint_states >/dev/null 2>&1; then
  echo "[sim-direct-fast60] No fresh incubator joint feedback; no command sent." >&2
  exit 2
fi
if ! timeout 8 ros2 topic echo --once /realsense_camera/depth/points \
  --field header >/dev/null 2>&1
then
  echo "[sim-direct-fast60] No fresh depth point cloud; no command sent." >&2
  exit 2
fi
if ! DISPLAY="${DISPLAY:-:0}" xwininfo -root -tree 2>/dev/null | \
  grep -q '"Gazebo Sim"'
then
  echo "[sim-direct-fast60] Gazebo GUI is not visible; no command sent." >&2
  exit 2
fi

controllers="$(timeout 5 ros2 control list_controllers)" || {
  echo "[sim-direct-fast60] Could not read controller states; no command sent." >&2
  exit 2
}
for controller in \
  right_arm_controller left_arm_controller joint_state_broadcaster \
  left_gripper_controller right_gripper_controller
do
  if ! grep -Eq "^${controller}[[:space:]].*[[:space:]]active$" <<<"${controllers}"; then
    echo "[sim-direct-fast60] Controller ${controller} is not active; no command sent." >&2
    exit 2
  fi
done

for trajectory_topic in \
  /right_arm_controller/joint_trajectory \
  /left_arm_controller/joint_trajectory \
  /right_gripper_controller/joint_trajectory \
  /left_gripper_controller/joint_trajectory
do
  topic_info="$(timeout 5 ros2 topic info "${trajectory_topic}")" || {
    echo "[sim-direct-fast60] Could not inspect ${trajectory_topic}; no command sent." >&2
    exit 2
  }
  if ! grep -q '^Publisher count: 0$' <<<"${topic_info}"; then
    echo "[sim-direct-fast60] ${trajectory_topic} has a publisher; no command sent." >&2
    exit 2
  fi
done

gz_topic_info="$(timeout 5 gz topic -i -t "${command_topic}")" || {
  echo "[sim-direct-fast60] Could not inspect the Gazebo command topic; no command sent." >&2
  exit 2
}
if ! grep -q '^No publishers on topic' <<<"${gz_topic_info}" ||
  [[ "$(grep -c 'gz.msgs.JointTrajectory' <<<"${gz_topic_info}")" -ne 1 ]]
then
  echo "[sim-direct-fast60] Expected exactly one resident controller and no publisher; no command sent." >&2
  exit 2
fi

echo "[sim-direct-fast60] EXPERIMENTAL SIMULATION ONLY."
echo "[sim-direct-fast60] Reusing the resident Gazebo controller for a dense 40 -> 60 deg door continuation."
echo "[sim-direct-fast60] Candidate 3 remains only a parameter gate; no robot IK/path or ROS trajectory is used."

exec ros2 run h1_2_moveit_demo incubator_sim_direct_fast40_demo --ros-args \
  -p use_sim_time:=true \
  -p door_handoff_only_candidate:=3 \
  -p door_handoff_simulation_direct_joint_fast40:=false \
  -p door_handoff_simulation_direct_joint_fast60_continuation:=true
