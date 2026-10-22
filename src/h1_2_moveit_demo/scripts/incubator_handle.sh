#!/usr/bin/env bash
set -eo pipefail

mode="${1:-task}"
case "${mode}" in
  task|demo)
    ;;
  *)
    echo "usage: ros2 run h1_2_moveit_demo incubator_handle.sh [task|demo]" >&2
    echo "  task: fresh/down handle -> approach, clamp, 0..90 deg; completed state is read-only" >&2
    echo "  demo: verified clamped 90 deg state -> 90..0..90 deg live cycle" >&2
    exit 2
    ;;
esac

workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
runtime_dir="/home/liu/h1_moveit/runtime_logs/incubator_handle"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${workspace}/install/setup.bash"
set -u

if pgrep -f '/h1_2_moveit_demo/incubator_door_open_demo([[:space:]]|$)' >/dev/null; then
  echo "[incubator-handle] Another incubator_door_open_demo is active; no command sent." >&2
  exit 2
fi

gazebo_running=false
if pgrep -f 'gz sim server' >/dev/null; then
  gazebo_running=true
fi

if [[ "${gazebo_running}" == false ]]; then
  if [[ "${mode}" == "demo" ]]; then
    echo "[incubator-handle] demo requires the existing verified 90 deg clamped state; Gazebo is absent." >&2
    exit 2
  fi
  mkdir -p "${runtime_dir}"
  stamp="$(date +%Y%m%d_%H%M%S)"
  echo "[incubator-handle] Starting Gazebo, GUI, controllers and the incubator..."
  nohup ros2 launch h1_2_moveit_config gazebo.launch.py \
    spawn_incubator:=true incubator_x:=0.590 \
    >"${runtime_dir}/gazebo_${stamp}.log" 2>&1 </dev/null &
  gazebo_pid=$!
  disown "${gazebo_pid}" 2>/dev/null || true
  for _ in $(seq 1 120); do
    if pgrep -f 'gz sim server' >/dev/null &&
      timeout 2 gz model -m incubator -p >/dev/null 2>&1; then
      gazebo_running=true
      break
    fi
    sleep 1
  done
  if [[ "${gazebo_running}" == false ]]; then
    echo "[incubator-handle] Gazebo did not become healthy within 120 s; no demo command sent." >&2
    exit 2
  fi
elif ! timeout 3 gz model -m incubator -p >/dev/null 2>&1; then
  echo "[incubator-handle] Existing Gazebo has no healthy incubator model; refusing a duplicate launch/reset." >&2
  exit 2
fi

move_group_running=false
if pgrep -f '/moveit_ros_move_group/move_group([[:space:]]|$)' >/dev/null; then
  move_group_running=true
fi
if [[ "${move_group_running}" == false ]]; then
  mkdir -p "${runtime_dir}"
  stamp="$(date +%Y%m%d_%H%M%S)"
  echo "[incubator-handle] Starting move_group..."
  nohup ros2 launch h1_2_moveit_config move_group_only.launch.py \
    >"${runtime_dir}/move_group_${stamp}.log" 2>&1 </dev/null &
  move_group_pid=$!
  disown "${move_group_pid}" 2>/dev/null || true
  for _ in $(seq 1 90); do
    if ros2 service type /get_planning_scene 2>/dev/null |
      grep -q '^moveit_msgs/srv/GetPlanningScene$'; then
      move_group_running=true
      break
    fi
    sleep 1
  done
  if [[ "${move_group_running}" == false ]]; then
    echo "[incubator-handle] move_group did not become healthy within 90 s; no demo command sent." >&2
    exit 2
  fi
elif ! ros2 service type /get_planning_scene 2>/dev/null |
  grep -q '^moveit_msgs/srv/GetPlanningScene$'; then
  echo "[incubator-handle] Existing move_group process is unhealthy; refusing a duplicate launch." >&2
  exit 2
fi

controllers="$(ros2 control list_controllers 2>/dev/null || true)"
for controller in joint_state_broadcaster right_arm_controller \
  left_arm_controller right_gripper_controller left_gripper_controller; do
  if ! grep -Eq "^${controller}[[:space:]].*[[:space:]]active$" <<<"${controllers}"; then
    echo "[incubator-handle] Controller ${controller} is not active; no command sent." >&2
    exit 2
  fi
done

if ! timeout 8 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
  echo "[incubator-handle] No fresh /joint_states feedback; no command sent." >&2
  exit 2
fi
if ! timeout 8 ros2 topic echo --once /move_group/d435_filtered_cloud \
  >/dev/null 2>&1; then
  echo "[incubator-handle] No fresh filtered depth cloud; no command sent." >&2
  exit 2
fi

cycle_argument=()
if [[ "${mode}" == "demo" ]]; then
  cycle_argument=(-p handle_rotate_cycle_demo:=true)
fi

echo "[incubator-handle] Health PASS. Running ${mode} with full geometry gates, 0.21 s/deg and one continuous 90 deg live stage..."
ros2 run h1_2_moveit_demo incubator_door_open_demo --ros-args \
  -p use_sim_time:=true \
  -p preflight_only:=false \
  -p handle_rotate:=true \
  -p handle_rotate_fast_mode:=true \
  "${cycle_argument[@]}" \
  -p front_handle_pivot_tcp_arc:=true \
  -p front_diagonal_dual_pad_test:=true \
  -p front_reverse_seed_entry:=true \
  -p front_handle_arm:=left \
  -p front_handle_phase_degrees:=-159.0 \
  -p front_palm_down_tilt_degrees:=7.0 \
  -p front_handle_local_x_offset_mm:=0.0 \
  -p front_handle_local_z_mm:=90.0 \
  -p front_handle_outward_offset_mm:=16.0 \
  -p front_gripper_position_mm:=11.85 \
  -p front_gripper_asymmetry_mm:=-0.5 \
  -p front_diagonal_max_station_separation_mm:=26.0 \
  -p front_diagonal_minimum_body_clearance_mm:=0.6 \
  -p front_safe_ik_attempts:=100 \
  -p front_fixed_torso_yaw_degrees:=0.0
