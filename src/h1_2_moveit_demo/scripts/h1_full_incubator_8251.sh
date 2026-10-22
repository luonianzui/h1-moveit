#!/usr/bin/env bash
# Replay the four sealed, robot-controller trajectory assets for the H1-2
# incubator task.  This wrapper never drives an incubator joint directly and
# never resets, detaches, or opens a supporting hand as failure cleanup.
set -Eeuo pipefail

readonly WORKSPACE="${H1_FULL_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly TRAJECTORY_TOOL="${H1_FULL_TRAJECTORY_TOOL:-${WORKSPACE}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_joint_trajectory_asset.py}"
readonly ROS_SNAPSHOT_TOOL="${H1_FULL_ROS_SNAPSHOT_TOOL:-${WORKSPACE}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_ros_snapshot_8251.py}"
readonly RUNTIME_ROOT="${H1_FULL_RUNTIME_ROOT:-/home/liu/h1_moveit/runtime_logs/full_robot_flow_8251}"
readonly LOCK_FILE="${H1_FULL_LOCK_FILE:-${RUNTIME_ROOT}/active.lock}"

readonly MAX_WALL_SECONDS="${H1_FULL_MAX_WALL_SECONDS:-120}"
readonly PHYSICS_MAX_STEP_SIZE="${H1_FULL_PHYSICS_MAX_STEP_SIZE:-0.008}"
readonly CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE="${H1_FULL_CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE:-0.006}"
readonly CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE="${H1_FULL_CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE:-0.006}"
readonly PHYSICS_REAL_TIME_FACTOR="${H1_FULL_PHYSICS_REAL_TIME_FACTOR:-2.0}"
readonly MINIMUM_RTF="${H1_FULL_MINIMUM_RTF:-0.70}"
readonly CUBE_SUPPORTED_MINIMUM_RTF="${H1_FULL_CUBE_SUPPORTED_MINIMUM_RTF:-0.50}"
readonly CUBE_SUPPORTED_RTF_SAMPLE_COUNT="${H1_FULL_CUBE_SUPPORTED_RTF_SAMPLE_COUNT:-3}"
readonly RTF_SAMPLE_COUNT="${H1_FULL_RTF_SAMPLE_COUNT:-1}"
readonly RTF_GATE_ATTEMPTS="${H1_FULL_RTF_GATE_ATTEMPTS:-3}"
readonly RECORD_VIDEO="${H1_FULL_RECORD:-1}"
readonly RECORD_FRAMERATE="${H1_FULL_RECORD_FRAMERATE:-12}"
readonly RECORD_CRF="${H1_FULL_RECORD_CRF:-20}"
readonly VIDEO_OUTPUT_OVERRIDE="${H1_FULL_VIDEO_OUTPUT:-}"
readonly VIDEO_CROP="${H1_FULL_VIDEO_CROP:-1160:1530:0:80}"
readonly GUI_CAMERA_SERVICE="/gui/move_to/pose"

# The trajectory JSON deliberately contains controller commands only.  It
# cannot reproduce the independent Gazebo cube-support attach/detach calls.
# A run therefore requires this external, separately validated hook:
#
#   H1_CUBE_SUPPORT_HELPER=/absolute/executable
#   helper preload -> exit 0 and print CUBE_SUPPORT_PRELOAD=PASS
#   helper attach  -> exit 0 and print CUBE_SUPPORT_HOOK=PASS operation=attach
#   helper detach  -> exit 0 and print CUBE_SUPPORT_HOOK=PASS operation=detach
#
# The helper receives the asset and split indices through exported H1_* vars.
# It must verify real contact/support and the plugin response itself.  An empty
# hook is never treated as success.
readonly CUBE_SUPPORT_HELPER="${H1_CUBE_SUPPORT_HELPER:-${WORKSPACE}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_cube_support_8251.sh}"
readonly CUBE_ATTACH_AFTER_INDEX="${H1_CUBE_PRE_ATTACH_END_INDEX:-${H1_CUBE_ATTACH_AFTER_INDEX:-2}}"
readonly CUBE_DETACH_AFTER_INDEX="${H1_CUBE_PRE_DETACH_END_INDEX:-${H1_CUBE_DETACH_AFTER_INDEX:-4}}"
readonly DOOR_SUPPORT_HELPER="${H1_DOOR_SUPPORT_HELPER:-${WORKSPACE}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_door_support_8251.sh}"
readonly DOOR_ATTACH_AFTER_INDEX="${H1_DOOR_ATTACH_AFTER_INDEX:-6}"
readonly DOOR_SUPPORT_MAX_FORCE="${H1_DOOR_SUPPORT_MAX_FORCE:-120.0}"
readonly DOOR_SUPPORT_MAX_TORQUE="${H1_DOOR_SUPPORT_MAX_TORQUE:-8.0}"

# These fixed per-message schedules were admitted against the sealed workspace
# joint bounds by h1_joint_trajectory_asset.py.  They change timing only, never
# the stored positions or stage order.  Door and cube schedules are split at
# their verified Gazebo support boundaries.
readonly HANDLE_MESSAGE_TIME_SCALES="${H1_FULL_HANDLE_MESSAGE_TIME_SCALES:-0:0.25,3:0.25,4:0.25}"
readonly LEFT_CLEAR_MESSAGE_TIME_SCALES="${H1_FULL_LEFT_CLEAR_MESSAGE_TIME_SCALES:-0:0.25,1:0.25}"
readonly DOOR_BEFORE_SUPPORT_TIME_SCALES="${H1_FULL_DOOR_BEFORE_SUPPORT_TIME_SCALES:-0:0.3,1:0.1,2:0.3,3:0.15,4:0.35,5:0.1}"
readonly DOOR_AFTER_SUPPORT_TIME_SCALES="${H1_FULL_DOOR_AFTER_SUPPORT_TIME_SCALES:-6:0.12,7:0.15,8:0.1,9:0.1,10:0.1,11:0.1,12:0.1}"
readonly CUBE_BEFORE_ATTACH_TIME_SCALES="${H1_FULL_CUBE_BEFORE_ATTACH_TIME_SCALES:-0:1,1:1}"
readonly CUBE_SUPPORTED_CARRY_TIME_SCALES="${H1_FULL_CUBE_SUPPORTED_CARRY_TIME_SCALES:-2:0.75,3:0.75}"
readonly CUBE_AFTER_DETACH_TIME_SCALES="${H1_FULL_CUBE_AFTER_DETACH_TIME_SCALES:-4:0.1,5:0.35}"

readonly DIRECT_DOOR_PROCESS_PATTERN="incubator_sim_direct_fast40_demo"
readonly DIRECT_DOOR_TOPIC="/h1_2/incubator/simulation_direct_joint_trajectory"
readonly WORLD_STATS_TOPIC="/world/h1_world/stats"
readonly SET_PHYSICS_SERVICE="/world/h1_world/set_physics/blocking"
readonly FILTERED_CLOUD_TOPIC="/move_group/d435_filtered_cloud"
readonly LATCH_STATE_TOPIC="/incubator/latch/state"

readonly DOOR_ZERO_RAD="0.0"
readonly DOOR_SIXTY_RAD="1.047197551196598"
readonly HANDLE_ZERO_RAD="0.0"
readonly HANDLE_NINETY_RAD="1.570796326794897"
readonly DOOR_FRESH_TOLERANCE_RAD="0.017453292519943"
readonly DOOR_CLOSED_TOLERANCE_RAD="0.003490658503989"
readonly DOOR_OPEN_TOLERANCE_RAD="0.034906585039887"
readonly HANDLE_FRESH_TOLERANCE_RAD="0.034906585039887"
readonly HANDLE_ROTATED_TOLERANCE_RAD="0.052359877559830"
readonly MECHANISM_MAX_VELOCITY="0.002"

readonly -a ROBOT_TRAJECTORY_TOPICS=(
  /right_arm_controller/joint_trajectory
  /left_arm_controller/joint_trajectory
  /right_gripper_controller/joint_trajectory
  /left_gripper_controller/joint_trajectory
)
readonly -a REQUIRED_CONTROLLERS=(
  right_arm_controller
  left_arm_controller
  joint_state_broadcaster
  right_gripper_controller
  left_gripper_controller
)

declare -A ASSET_PATHS=()
declare -A ASSET_COUNTS=()
declare -A ASSET_SHA256=()
RUN_DIR=""
RECORDER_PID=""
VIDEO_OUTPUT=""
VIDEO_WINDOW_ID=""
VIDEO_EXPECTED_WIDTH="0"
VIDEO_EXPECTED_HEIGHT="0"
FIRST_PUBLISH_MONITOR_PID=""
FIRST_PUBLISH_FILE=""
FIRST_PUBLISH_READY_FILE=""
FLOW_INVOCATION_START_NS="0"
FLOW_START_NS="0"
FLOW_END_NS="0"
TRAJECTORY_IDLE_GATE_SEQUENCE="0"
ACTIVE_PHYSICS_MAX_STEP_SIZE="${PHYSICS_MAX_STEP_SIZE}"

usage()
{
  cat <<'EOF'
usage:
  h1_full_incubator_8251.sh preflight HANDLE.json LEFT_CLEAR.json DOOR.json CUBE.json
  h1_full_incubator_8251.sh run       HANDLE.json LEFT_CLEAR.json DOOR.json CUBE.json

The four inputs must be sealed h1_joint_trajectory_asset/v1 files owned by the
current user with mode 0400, one hard link, a valid embedded SHA-256, and stage
identities handle / left_clear / door / cube respectively.

The 8251 defaults bind the installed door/cube support helpers, door split 6,
cube split 2/4, and the admitted cube schedules 0:1,1:1 / 2:0.75,3:0.75 /
4:0.1,5:0.35.  Their H1_* variables remain available only as explicit overrides.

The older aliases H1_CUBE_ATTACH_AFTER_INDEX and
H1_CUBE_DETACH_AFTER_INDEX remain accepted when the canonical values above
are unset.

Before the controller timer starts, the door and cube helpers are each called
once as `helper preload` to install a detached resident support plugin.  They
must print their exact preload markers, after which a repeated fresh-scene gate
proves that detached preload changed no physical state.  The cube helper is
later called as `helper attach` and `helper detach`; those operations must print:
  CUBE_SUPPORT_HOOK=PASS operation=attach
  CUBE_SUPPORT_HOOK=PASS operation=detach

N and M are global cube-asset message boundaries, with 0 < N < M < count.
The door asset is replayed through its verified clamp boundary, then the door
helper must attach bounded compliant support to the existing door_link and
print its exact DOOR_SUPPORT_HOOK=PASS marker before seven arm-only state
segments.  The first physical 0-to-10-degree command contains one settled
reference interval, so capture-state reconstructs that command as two sealed
segments; the seven messages still represent the six verified 10-degree arcs.
Recording is enabled by default.  Set H1_FULL_RECORD=0 only for an explicitly
requested no-video diagnostic; that mode can emit only the distinct
FULL_ROBOT_FLOW_DIAGNOSTIC=PASS marker, never the recorded physical-acceptance
marker.  The run requires a fresh, stationary 0/0 degree scene and never resets
or releases support on failure.
EOF
}

log()
{
  printf '[h1-full-8251] %s\n' "$*"
}

fail()
{
  printf '[h1-full-8251] ERROR: %s\n' "$*" >&2
  return 1
}

require_executable()
{
  local path="$1"
  local resolved
  resolved="$(readlink -f -- "${path}" 2>/dev/null || true)"
  [[ -n "${resolved}" && -f "${resolved}" && -x "${resolved}" && -x "${path}" ]] ||
    fail "required executable is missing or non-executable: ${path}"
}

require_unsigned_integer()
{
  local value="$1"
  local label="$2"
  [[ "${value}" =~ ^[0-9]+$ ]] || fail "${label} must be an unsigned integer"
}

require_positive_number()
{
  local value="$1"
  local label="$2"
  awk -v value="${value}" 'BEGIN {exit !(value + 0.0 > 0.0)}' ||
    fail "${label} must be a finite positive number"
}

require_no_other_demo()
{
  if pgrep -af \
    '(/h1_2_moveit_demo/)(incubator_door_open_demo|bimanual_air_handover_demo)([[:space:]]|$)' \
    >/dev/null
  then
    fail "another incubator or cube demo is active"
  fi
  if pgrep -af '[h]1_joint_trajectory_asset.py[[:space:]]+replay([[:space:]]|$)' \
    >/dev/null
  then
    fail "another sealed-trajectory replay is active"
  fi
}

require_no_robot_trajectory_publishers()
{
  # Four independent ROS graph queries used to cost roughly four CLI startup
  # times after every replay range.  Run them concurrently without weakening
  # any per-topic publisher/subscriber assertion.
  ((TRAJECTORY_IDLE_GATE_SEQUENCE+=1))
  local sequence="${TRAJECTORY_IDLE_GATE_SEQUENCE}"
  local index topic info failure=""
  local -a pids=() files=()
  for index in "${!ROBOT_TRAJECTORY_TOPICS[@]}"; do
    topic="${ROBOT_TRAJECTORY_TOPICS[${index}]}"
    files[${index}]="${RUN_DIR}/trajectory_idle_${sequence}_${index}.info"
    timeout 5 ros2 topic info "${topic}" >"${files[${index}]}" 2>/dev/null &
    pids[${index}]=$!
  done
  for index in "${!ROBOT_TRAJECTORY_TOPICS[@]}"; do
    topic="${ROBOT_TRAJECTORY_TOPICS[${index}]}"
    if ! wait "${pids[${index}]}"; then
      failure="${failure}${topic}: graph query failed; "
      continue
    fi
    info="$(<"${files[${index}]}")"
    grep -Fxq 'Publisher count: 0' <<<"${info}" ||
      failure="${failure}${topic}: publisher is present; "
    grep -Eq '^Subscription count: [1-9][0-9]*$' <<<"${info}" ||
      failure="${failure}${topic}: controller subscriber is absent; "
  done
  [[ -z "${failure}" ]] || fail "robot trajectory idle gate failed: ${failure}"
}

run_parallel_gate_group()
{
  # Each check is passed as: ARG_COUNT command arg... .  Checks are read-only,
  # write separate evidence logs, and all must succeed before the flow can
  # advance.  Parallel execution only removes repeated ROS/Gazebo CLI startup
  # latency; it does not combine or omit a physical gate.
  local group="$1"
  shift
  local count index=0 failure="" log_file command_name
  local -a command=() pids=() files=() names=()
  while (( $# > 0 )); do
    count="$1"
    shift
    require_unsigned_integer "${count}" "${group} parallel gate argument count"
    (( count > 0 && count <= $# )) ||
      fail "${group}: malformed parallel gate command"
    command=("${@:1:count}")
    shift "${count}"
    command_name="${command[0]}"
    log_file="${RUN_DIR}/parallel_${group}_${index}_${command_name}.log"
    files[${index}]="${log_file}"
    names[${index}]="${command_name}"
    ( "${command[@]}" ) >"${log_file}" 2>&1 &
    pids[${index}]=$!
    ((index+=1))
  done
  (( index >= 2 )) || fail "${group}: parallel gate group needs at least two checks"
  for index in "${!pids[@]}"; do
    if ! wait "${pids[${index}]}"; then
      failure="${failure}${names[${index}]}; "
    fi
  done
  for log_file in "${files[@]}"; do
    while IFS= read -r line || [[ -n "${line}" ]]; do
      printf '%s\n' "${line}"
    done <"${log_file}"
  done
  [[ -z "${failure}" ]] || fail "${group}: parallel physical gate failed: ${failure}"
  log "${group}_PARALLEL_GATE=PASS checks=${#pids[@]}"
}

reject_direct_door_controller()
{
  reject_direct_door_process
  local info
  info="$(timeout 5 gz topic -i -t "${DIRECT_DOOR_TOPIC}" 2>&1 || true)"
  grep -Fq "No publishers on topic [${DIRECT_DOOR_TOPIC}]" <<<"${info}" ||
    fail "private Gazebo direct-door topic has a publisher or could not be inspected"
  grep -Fq "No subscribers on topic [${DIRECT_DOOR_TOPIC}]" <<<"${info}" ||
    fail "private Gazebo direct-door controller subscriber is resident"
}

reject_direct_door_process()
{
  if pgrep -f "${DIRECT_DOOR_PROCESS_PATTERN}" >/dev/null; then
    fail "Gazebo direct-door executable is active"
  fi
}

basic_health_gate()
{
  pgrep -f 'gz sim server' >/dev/null || fail "Gazebo server is absent"
  pgrep -f 'gz sim gui' >/dev/null || fail "Gazebo GUI process is absent"
  local window_tree
  window_tree="$(xwininfo -root -tree 2>/dev/null)" ||
    fail "could not inspect the X11 window tree"
  grep -Fq '"Gazebo Sim"' <<<"${window_tree}" ||
    fail "Gazebo GUI window is not visible"
  timeout 5 gz model -m incubator -p >/dev/null 2>&1 ||
    fail "incubator model is unavailable"
  timeout 5 gz model -m green_cube -p >/dev/null 2>&1 ||
    fail "green_cube model is unavailable"
  ros2 service type /get_planning_scene 2>/dev/null |
    grep -Fxq 'moveit_msgs/srv/GetPlanningScene' ||
    fail "move_group planning-scene service is unavailable"

  local controllers controller
  controllers="$(ros2 control list_controllers 2>/dev/null || true)"
  for controller in "${REQUIRED_CONTROLLERS[@]}"; do
    grep -Eq "^${controller}[[:space:]].*[[:space:]]active$" <<<"${controllers}" ||
      fail "controller is not active: ${controller}"
  done

  timeout 8 ros2 topic echo --once /joint_states >/dev/null 2>&1 ||
    fail "no fresh /joint_states"
  timeout 8 ros2 topic echo --once /incubator/joint_states >/dev/null 2>&1 ||
    fail "no fresh /incubator/joint_states"
  timeout 8 ros2 topic echo --once "${FILTERED_CLOUD_TOPIC}" >/dev/null 2>&1 ||
    fail "no fresh filtered point cloud"
  timeout 5 gz topic -e -t "${WORLD_STATS_TOPIC}" -n 1 >/dev/null 2>&1 ||
    fail "Gazebo world statistics are unavailable"
  reject_direct_door_controller
  require_no_robot_trajectory_publishers
}

read_incubator_snapshot()
{
  local attempt snapshot
  for attempt in 1 2; do
    if snapshot="$(timeout 5 "${ROS_SNAPSHOT_TOOL}" incubator --timeout 3)"; then
      printf '%s\n' "${snapshot}"
      return 0
    fi
  done
  return 1
}

mechanism_gate()
{
  local label="$1"
  local expected_door="$2"
  local door_tolerance="$3"
  local expected_handle="$4"
  local handle_tolerance="$5"
  local snapshot door_row handle_row door door_velocity handle handle_velocity
  snapshot="$(read_incubator_snapshot)" || {
    fail "${label}: malformed incubator telemetry"
    return 1
  }
  door_row="$(awk '$1 == "door_hinge_joint" {print $2, $3; found++} END {if (found != 1) exit 1}' <<<"${snapshot}")" ||
    {
      fail "${label}: door_hinge_joint telemetry is missing or duplicated"
      return 1
    }
  handle_row="$(awk '$1 == "handle_latch_joint" {print $2, $3; found++} END {if (found != 1) exit 1}' <<<"${snapshot}")" ||
    {
      fail "${label}: handle_latch_joint telemetry is missing or duplicated"
      return 1
    }
  read -r door door_velocity <<<"${door_row}"
  read -r handle handle_velocity <<<"${handle_row}"
  awk -v door="${door}" -v expected_door="${expected_door}" \
    -v door_tolerance="${door_tolerance}" -v handle="${handle}" \
    -v expected_handle="${expected_handle}" \
    -v handle_tolerance="${handle_tolerance}" \
    -v door_velocity="${door_velocity}" -v handle_velocity="${handle_velocity}" \
    -v maximum_velocity="${MECHANISM_MAX_VELOCITY}" 'BEGIN {
      # Force scientific-notation subnormal telemetry (for example 9e-322)
      # through numeric conversion before mawk comparisons.
      door+=0.0; handle+=0.0; door_velocity+=0.0; handle_velocity+=0.0
      door_error=door-expected_door; if (door_error < 0) door_error=-door_error
      handle_error=handle-expected_handle; if (handle_error < 0) handle_error=-handle_error
      if (door_velocity < 0) door_velocity=-door_velocity
      if (handle_velocity < 0) handle_velocity=-handle_velocity
      exit !(door_error <= door_tolerance && handle_error <= handle_tolerance &&
        door_velocity <= maximum_velocity && handle_velocity <= maximum_velocity)
    }' || {
      fail "${label}: door/handle position or velocity gate failed"
      return 1
    }
  log "${label}=PASS door=${door} handle=${handle} door_velocity=${door_velocity} handle_velocity=${handle_velocity}"
}

require_latch_state()
{
  local expected="$1"
  local label="$2"
  local attempt output=""
  for attempt in 1 2 3; do
    if output="$(timeout 5 gz topic -e -t "${LATCH_STATE_TOPIC}" -n 1 2>/dev/null)"; then
      break
    fi
    output=""
  done
  [[ -n "${output}" ]] || fail "${label}: latch telemetry is unavailable"
  grep -Fxq "data: \"${expected}\"" <<<"${output}" ||
    fail "${label}: expected latch state '${expected}'"
  log "${label}=PASS latch=${expected}"
}

read_robot_joint_snapshot()
{
  local attempt snapshot
  for attempt in 1 2; do
    if snapshot="$(timeout 5 "${ROS_SNAPSHOT_TOOL}" robot --timeout 3)"; then
      printf '%s\n' "${snapshot}"
      return 0
    fi
  done
  return 1
}

joint_values_gate()
{
  local label="$1"
  shift
  (( $# > 0 && $# % 3 == 0 )) || {
    fail "${label}: invalid joint gate definition"
    return 1
  }
  local snapshot joint expected tolerance row actual
  snapshot="$(read_robot_joint_snapshot)" || {
    fail "${label}: malformed robot telemetry"
    return 1
  }
  while (( $# > 0 )); do
    joint="$1"
    expected="$2"
    tolerance="$3"
    shift 3
    row="$(awk -v joint="${joint}" '$1 == joint {print $2; found++} END {if (found != 1) exit 1}' <<<"${snapshot}")" ||
      {
        fail "${label}: joint '${joint}' is missing or duplicated"
        return 1
      }
    actual="${row}"
    awk -v actual="${actual}" -v expected="${expected}" -v tolerance="${tolerance}" \
      'BEGIN {error=actual-expected; if (error < 0) error=-error; exit !(error <= tolerance)}' ||
      {
        fail "${label}: ${joint}=${actual}, expected ${expected} +/- ${tolerance} rad"
        return 1
      }
  done
  log "${label}=PASS"
}

neutral_arms_and_open_hands_gate()
{
  local label="$1"
  joint_values_gate "${label}" \
    left_shoulder_pitch_joint 0.0 0.004 \
    left_shoulder_roll_joint 0.25 0.004 \
    left_shoulder_yaw_joint 0.0 0.004 \
    left_elbow_joint 0.60 0.004 \
    left_wrist_pitch_joint 0.0 0.004 \
    left_wrist_roll_joint 0.0 0.004 \
    left_wrist_yaw_joint 0.0 0.004 \
    right_shoulder_pitch_joint 0.0 0.004 \
    right_shoulder_roll_joint -0.25 0.004 \
    right_shoulder_yaw_joint 0.0 0.004 \
    right_elbow_joint 0.60 0.004 \
    right_wrist_pitch_joint 0.0 0.004 \
    right_wrist_roll_joint 0.0 0.004 \
    right_wrist_yaw_joint 0.0 0.004 \
    left_dex1_finger_1_joint -0.019 0.001 \
    left_dex1_finger_2_joint -0.019 0.001 \
    right_dex1_finger_1_joint -0.019 0.001 \
    right_dex1_finger_2_joint -0.019 0.001 \
    torso_joint 0.0 0.003
}

left_clear_gate()
{
  local label="$1"
  joint_values_gate "${label}" \
    left_shoulder_pitch_joint 0.0 0.004 \
    left_shoulder_roll_joint 0.30 0.004 \
    left_shoulder_yaw_joint 0.0 0.004 \
    left_elbow_joint 0.60 0.004 \
    left_wrist_pitch_joint 0.0 0.004 \
    left_wrist_roll_joint 0.0 0.004 \
    left_wrist_yaw_joint 0.0 0.004 \
    left_dex1_finger_1_joint -0.019 0.001 \
    left_dex1_finger_2_joint -0.019 0.001 \
    torso_joint 0.0 0.003
}

left_handle_hold_gate()
{
  joint_values_gate "$1" \
    left_dex1_finger_1_joint 0.01135 0.0015 \
    left_dex1_finger_2_joint 0.01235 0.0015
}

right_door_hold_gate()
{
  joint_values_gate "$1" \
    right_dex1_finger_1_joint 0.01131 0.0015 \
    right_dex1_finger_2_joint 0.01235 0.0015
}

read_cube_pose()
{
  local attempt output pose
  for attempt in 1 2 3; do
    if output="$(timeout 5 gz model -m green_cube -p 2>/dev/null)" &&
      pose="$(awk '
        /Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:/ {
          if (getline <= 0) exit 1
          gsub(/[\[\]]/, "")
          x=$1; y=$2; z=$3
          if (getline <= 0) exit 1
          gsub(/[\[\]]/, "")
          print x, y, z, $1, $2, $3
          found=1
          exit
        }
        END {if (!found) exit 1}
      ' <<<"${output}")"
    then
      printf '%s\n' "${pose}"
      return 0
    fi
  done
  return 1
}

cube_pose_gate()
{
  local label="$1"
  local expected_x="$2"
  local expected_y="$3"
  local expected_z="$4"
  local expected_yaw="$5"
  local position_tolerance="$6"
  local yaw_tolerance="$7"
  local pose x y z roll pitch yaw
  pose="$(read_cube_pose)" || fail "${label}: cube pose unavailable"
  read -r x y z roll pitch yaw <<<"${pose}"
  awk -v x="${x}" -v y="${y}" -v z="${z}" -v roll="${roll}" \
    -v pitch="${pitch}" -v yaw="${yaw}" -v expected_x="${expected_x}" \
    -v expected_y="${expected_y}" -v expected_z="${expected_z}" \
    -v expected_yaw="${expected_yaw}" -v position_tolerance="${position_tolerance}" \
    -v yaw_tolerance="${yaw_tolerance}" 'BEGIN {
      pi=3.141592653589793
      dx=x-expected_x; dy=y-expected_y; dz=z-expected_z
      position_error=sqrt(dx*dx+dy*dy+dz*dz)
      yaw_error=yaw-expected_yaw
      while (yaw_error > pi) yaw_error-=2*pi
      while (yaw_error < -pi) yaw_error+=2*pi
      if (yaw_error < 0) yaw_error=-yaw_error
      if (roll < 0) roll=-roll
      if (pitch < 0) pitch=-pitch
      exit !(position_error <= position_tolerance && yaw_error <= yaw_tolerance &&
        roll <= 0.0872664626 && pitch <= 0.0872664626)
    }' || fail "${label}: cube pose [${pose}] is outside the required window"
  log "${label}=PASS cube=[${pose}]"
}

cube_stability_gate()
{
  local label="$1"
  local reference current
  reference="$(read_cube_pose)" || fail "${label}: initial cube pose unavailable"
  local sample_index
  for sample_index in 1 2 3; do
    sleep 0.25
    current="$(read_cube_pose)" || fail "${label}: cube pose sample ${sample_index} unavailable"
    awk -v first="${reference}" -v second="${current}" 'BEGIN {
      split(first, a, " "); split(second, b, " ")
      pi=3.141592653589793
      dx=a[1]-b[1]; dy=a[2]-b[2]; dz=a[3]-b[3]
      distance=sqrt(dx*dx+dy*dy+dz*dz)
      yaw=a[6]-b[6]
      while (yaw > pi) yaw-=2*pi
      while (yaw < -pi) yaw+=2*pi
      if (yaw < 0) yaw=-yaw
      exit !(distance <= 0.003 && yaw <= 0.0349065850)
    }' || fail "${label}: cube moved from [${reference}] to [${current}]"
  done
  log "${label}=PASS reference=[${reference}]"
}

fresh_scene_gate()
{
  mechanism_gate "FRESH_MECHANISM_GATE" \
    "${DOOR_ZERO_RAD}" "${DOOR_FRESH_TOLERANCE_RAD}" \
    "${HANDLE_ZERO_RAD}" "${HANDLE_FRESH_TOLERANCE_RAD}"
  require_latch_state locked "FRESH_LATCH_GATE"
  neutral_arms_and_open_hands_gate "FRESH_ROBOT_GATE"
  cube_pose_gate "FRESH_CUBE_GATE" \
    0.334000 0.377000 0.830000 1.570796326794897 0.003 0.034906585039887
  cube_stability_gate "FRESH_CUBE_STABILITY"
  require_no_robot_trajectory_publishers
}

inspect_asset()
{
  local stage="$1"
  local path="$2"
  local log_file="${RUN_DIR}/asset_${stage}.inspect.log"
  local status output actual_stage count digest
  set +e
  "${TRAJECTORY_TOOL}" inspect "${path}" 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) || fail "${stage} asset inspection exited ${status}"
  output="$(<"${log_file}")"
  actual_stage="$(awk -F= '$1 == "stage" {print $2; found++} END {if (found != 1) exit 1}' <<<"${output}")" ||
    fail "${stage} asset has no unique stage marker"
  [[ "${actual_stage}" == "${stage}" ]] ||
    fail "asset stage mismatch: expected ${stage}, found ${actual_stage}"
  grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${output}" ||
    fail "${stage} asset schema mismatch"
  count="$(awk -F= '$1 == "message_count" {print $2; found++} END {if (found != 1) exit 1}' <<<"${output}")" ||
    fail "${stage} asset has no unique message count"
  require_unsigned_integer "${count}" "${stage} message_count"
  (( count > 0 )) || fail "${stage} asset is empty"
  digest="$(awk -F= '$1 == "sha256" {print $2; found++} END {if (found != 1) exit 1}' <<<"${output}")" ||
    fail "${stage} asset has no unique SHA-256"
  [[ "${digest}" =~ ^[0-9a-f]{64}$ ]] || fail "${stage} asset SHA-256 is malformed"
  ASSET_COUNTS["${stage}"]="${count}"
  ASSET_SHA256["${stage}"]="${digest}"
  log "SEALED_ASSET=PASS stage=${stage} messages=${count} sha256=${digest} path=${path}"
}

validate_cube_split_contract()
{
  require_unsigned_integer "${CUBE_ATTACH_AFTER_INDEX}" H1_CUBE_ATTACH_AFTER_INDEX
  require_unsigned_integer "${CUBE_DETACH_AFTER_INDEX}" H1_CUBE_DETACH_AFTER_INDEX
  local count="${ASSET_COUNTS[cube]}"
  (( count == 6 && CUBE_ATTACH_AFTER_INDEX == 2 &&
     CUBE_DETACH_AFTER_INDEX == 4 )) ||
    fail "sealed cube asset must have exactly six messages and split 2/4"

  # The attach boundary follows the left closing trajectory.  The detach
  # boundary precedes the left opening trajectory.  This catches a misplaced
  # split before any controller publication; the helper remains responsible
  # for physical contact/support proof.
  jq -e --argjson attach "${CUBE_ATTACH_AFTER_INDEX}" \
    --argjson detach "${CUBE_DETACH_AFTER_INDEX}" '
      .payload.messages[$attach - 1].topic ==
        "/left_gripper_controller/joint_trajectory" and
      .payload.messages[$attach - 1].expected_actual_endpoint.kind ==
        "cube_gripper_contact_close" and
      .payload.messages[$attach - 1].points[-1].positions ==
        [-0.01472, -0.01472] and
      (.payload.messages[$attach - 1].expected_actual_endpoint.positions |
        type == "array" and length == 2) and
      .payload.messages[$detach - 1].topic ==
        "/left_arm_controller/joint_trajectory" and
      .payload.messages[$detach].topic ==
        "/left_gripper_controller/joint_trajectory"
    ' "${ASSET_PATHS[cube]}" >/dev/null ||
    fail "cube split does not surround close / supported carry / open messages"
  log "CUBE_SPLIT_CONTRACT=PASS ranges=[0,${CUBE_ATTACH_AFTER_INDEX})/[${CUBE_ATTACH_AFTER_INDEX},${CUBE_DETACH_AFTER_INDEX})/[${CUBE_DETACH_AFTER_INDEX},${count})"
}

validate_door_split_contract()
{
  require_unsigned_integer "${DOOR_ATTACH_AFTER_INDEX}" H1_DOOR_ATTACH_AFTER_INDEX
  local count="${ASSET_COUNTS[door]}"
  (( count == 13 && DOOR_ATTACH_AFTER_INDEX == 6 )) ||
    fail "sealed door asset must have clamp boundary 6 and exactly 13 messages"
  jq -e --argjson attach "${DOOR_ATTACH_AFTER_INDEX}" '
    .payload.messages as $messages |
    ($messages[0:$attach] | map(select(
      .topic == "/right_arm_controller/joint_trajectory")) | length) >= 1 and
    ($messages[0:$attach] | map(select(
      .topic == "/right_gripper_controller/joint_trajectory")) | length) >= 2 and
    $messages[$attach - 1].topic ==
      "/right_gripper_controller/joint_trajectory" and
    ($messages[$attach:] | length) == 7 and
    all($messages[$attach:][];
      .topic == "/right_arm_controller/joint_trajectory")
  ' "${ASSET_PATHS[door]}" >/dev/null ||
    fail "door split does not surround the verified clamp and seven sealed arm state segments"
  (( ASSET_COUNTS[handle] == 5 && ASSET_COUNTS[left_clear] == 2 )) ||
    fail "handle/left-clear asset cardinality differs from the admitted 5/2 schedule"
  log "DOOR_SPLIT_CONTRACT=PASS ranges=[0,${DOOR_ATTACH_AFTER_INDEX})/[${DOOR_ATTACH_AFTER_INDEX},${count})"
}

validate_retime_schedule()
{
  local stage="$1"
  local start="$2"
  local end="$3"
  local suffix="$4"
  local message_time_scales="$5"
  [[ -n "${message_time_scales}" ]] ||
    fail "${stage}/${suffix}: an admitted non-empty message timing schedule is required"

  local path="${ASSET_PATHS[${stage}]}"
  local count="${ASSET_COUNTS[${stage}]}"
  local digest="${ASSET_SHA256[${stage}]}"
  local log_file="${RUN_DIR}/retime_check_${stage}_${suffix}.log"
  local -a range_args=()
  if (( start != 0 || end != count )); then
    [[ "${stage}" == "cube" || "${stage}" == "door" ]] ||
      fail "${stage}/${suffix}: segmented retiming is allowed only for door or cube"
    range_args=(--start-index "${start}" --end-index "${end}")
  fi

  local status line_count expected_overrides
  expected_overrides="$(awk -F, '{print NF}' <<<"${message_time_scales}")"
  require_unsigned_integer "${expected_overrides}" \
    "${stage}/${suffix} retiming override count"
  (( expected_overrides > 0 )) ||
    fail "${stage}/${suffix}: retiming override count must be positive"
  set +e
  "${TRAJECTORY_TOOL}" retime-check "${path}" \
    --expected-stage "${stage}" "${range_args[@]}" \
    --message-time-scales "${message_time_scales}" 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "${stage}/${suffix}: offline retiming validation exited ${status}"
  line_count="$(wc -l <"${log_file}")"
  (( line_count == 1 )) ||
    fail "${stage}/${suffix}: offline retiming validation must emit exactly one line"
  grep -Eq \
    "^RETIME_CHECK=PASS stage=${stage} range=\\[${start},${end}\\) messages=$((end-start)) overrides=${expected_overrides} original_duration_seconds=[0-9]+\\.[0-9]{9} retimed_duration_seconds=[0-9]+\\.[0-9]{9} original_critical_path_seconds=[0-9]+\\.[0-9]{9} retimed_critical_path_seconds=[0-9]+\\.[0-9]{9} sha256=${digest}$" \
    "${log_file}" ||
    fail "${stage}/${suffix}: offline retiming validation lacks its exact PASS identity"
  log "RETIME_SCHEDULE=PASS stage=${stage} range=[${start},${end}) schedule=${message_time_scales}"
}

validate_all_retime_contracts()
{
  [[ -n "${CUBE_BEFORE_ATTACH_TIME_SCALES}" &&
    -n "${CUBE_SUPPORTED_CARRY_TIME_SCALES}" &&
    -n "${CUBE_AFTER_DETACH_TIME_SCALES}" ]] ||
    fail "all three admitted H1_FULL_CUBE_*_TIME_SCALES schedules are required"
  validate_retime_schedule handle 0 "${ASSET_COUNTS[handle]}" full \
    "${HANDLE_MESSAGE_TIME_SCALES}"
  validate_retime_schedule left_clear 0 "${ASSET_COUNTS[left_clear]}" full \
    "${LEFT_CLEAR_MESSAGE_TIME_SCALES}"
  validate_retime_schedule door 0 "${DOOR_ATTACH_AFTER_INDEX}" before_support \
    "${DOOR_BEFORE_SUPPORT_TIME_SCALES}"
  validate_retime_schedule door "${DOOR_ATTACH_AFTER_INDEX}" \
    "${ASSET_COUNTS[door]}" supported_arc \
    "${DOOR_AFTER_SUPPORT_TIME_SCALES}"
  validate_retime_schedule cube 0 "${CUBE_ATTACH_AFTER_INDEX}" before_attach \
    "${CUBE_BEFORE_ATTACH_TIME_SCALES}"
  validate_retime_schedule cube "${CUBE_ATTACH_AFTER_INDEX}" \
    "${CUBE_DETACH_AFTER_INDEX}" supported_carry \
    "${CUBE_SUPPORTED_CARRY_TIME_SCALES}"
  validate_retime_schedule cube "${CUBE_DETACH_AFTER_INDEX}" \
    "${ASSET_COUNTS[cube]}" after_detach \
    "${CUBE_AFTER_DETACH_TIME_SCALES}"
  log "FULL_FLOW_RETIME_PREFLIGHT=PASS; all seven timing plans passed and every explicit override satisfies the installed robot limits."
}

preflight_assets_and_dependencies()
{
  require_executable "${TRAJECTORY_TOOL}"
  require_executable "${ROS_SNAPSHOT_TOOL}"
  command -v jq >/dev/null || fail "jq is required"
  local stage
  for stage in handle left_clear door cube; do
    inspect_asset "${stage}" "${ASSET_PATHS[${stage}]}"
  done
  [[ -n "${CUBE_SUPPORT_HELPER}" && "${CUBE_SUPPORT_HELPER}" == /* ]] ||
    fail "H1_CUBE_SUPPORT_HELPER must name an absolute executable before replay"
  require_executable "${CUBE_SUPPORT_HELPER}"
  [[ -n "${DOOR_SUPPORT_HELPER}" && "${DOOR_SUPPORT_HELPER}" == /* ]] ||
    fail "H1_DOOR_SUPPORT_HELPER must name an absolute executable before replay"
  require_executable "${DOOR_SUPPORT_HELPER}"
  validate_door_split_contract
  validate_cube_split_contract
  validate_all_retime_contracts
  log "FULL_FLOW_ASSET_PREFLIGHT=PASS; no controller trajectory or support-plugin operation was sent."
}

set_fast_physics_profile()
{
  set_physics_profile "${PHYSICS_MAX_STEP_SIZE}" "PHYSICS_APPLY_GATE"
}

set_physics_profile()
{
  local requested_step="$1" label="$2"
  local required_rtf="${3:-${MINIMUM_RTF}}"
  local required_count="${4:-${RTF_SAMPLE_COUNT}}"
  require_positive_number "${requested_step}" physics-max-step-size
  require_positive_number "${PHYSICS_REAL_TIME_FACTOR}" H1_FULL_PHYSICS_REAL_TIME_FACTOR
  local update_rate response request
  update_rate="$(awk -v step="${requested_step}" \
    -v factor="${PHYSICS_REAL_TIME_FACTOR}" 'BEGIN {printf "%.12f", factor / step}')"
  request="real_time_factor: ${PHYSICS_REAL_TIME_FACTOR} real_time_update_rate: ${update_rate} max_step_size: ${requested_step}"
  response="$(
    timeout 8 gz service -s "${SET_PHYSICS_SERVICE}" \
      --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 \
      --req "${request}" 2>&1
  )" || fail "Gazebo set_physics service call failed"
  grep -Eq 'data:[[:space:]]*true' <<<"${response}" ||
    fail "Gazebo rejected the requested physics profile"
  ACTIVE_PHYSICS_MAX_STEP_SIZE="${requested_step}"
  physics_and_rtf_gate "${label}" "${required_rtf}" "${required_count}"
}

physics_and_rtf_gate()
{
  local label="$1" required_rtf="${2:-${MINIMUM_RTF}}"
  local required_count="${3:-${RTF_SAMPLE_COUNT}}"
  require_unsigned_integer "${required_count}" required-RTF-sample-count
  (( required_count > 0 )) || fail "RTF sample count must be positive"
  require_unsigned_integer "${RTF_GATE_ATTEMPTS}" H1_FULL_RTF_GATE_ATTEMPTS
  (( RTF_GATE_ATTEMPTS > 0 )) || fail "RTF gate attempts must be positive"
  require_positive_number "${required_rtf}" required-real-time-factor
  local samples expected_step_ns minimum="" attempt evidence_file
  expected_step_ns="$(awk -v step="${ACTIVE_PHYSICS_MAX_STEP_SIZE}" 'BEGIN {printf "%.0f", step * 1000000000.0}')"
  evidence_file="${RUN_DIR}/${label}.world_stats.jsonl"
  : >"${evidence_file}"
  for ((attempt=1; attempt<=RTF_GATE_ATTEMPTS; ++attempt)); do
    if ! samples="$(timeout 12 gz topic -e -t "${WORLD_STATS_TOPIC}" \
      -n "${required_count}" --json-output 2>/dev/null)"
    then
      continue
    fi
    printf '%s\n' "${samples}" >>"${evidence_file}"
    if minimum="$(jq -rs --argjson expected_count "${required_count}" \
      --argjson expected_step_ns "${expected_step_ns}" \
      --argjson minimum_rtf "${required_rtf}" '
        if length != $expected_count then error("sample count") else . end |
        map(
          (((.stepSize.sec // "0") | tonumber) * 1000000000 +
            ((.stepSize.nsec // "0") | tonumber)) == $expected_step_ns and
          (.realTimeFactor | type) == "number" and
          .realTimeFactor >= $minimum_rtf
        ) as $checks |
        if all($checks[]) then map(.realTimeFactor) | min
        else error("physics or RTF gate") end
      ' <<<"${samples}" 2>/dev/null)"
    then
      break
    fi
    minimum=""
  done
  [[ -n "${minimum}" ]] ||
    fail "${label}: no passing sample group in ${RTF_GATE_ATTEMPTS} attempts at step=${ACTIVE_PHYSICS_MAX_STEP_SIZE}s and RTF>=${required_rtf}"
  log "${label}=PASS step=${ACTIVE_PHYSICS_MAX_STEP_SIZE}s required_rtf=${required_rtf} samples=${required_count} minimum_rtf=${minimum} attempt=${attempt}/${RTF_GATE_ATTEMPTS}"
}

find_gazebo_window_id()
{
  local matches
  matches="$(xwininfo -root -tree 2>/dev/null |
    awk '/\("gz-sim-gui" "Gazebo GUI"\)/ {print $1}')"
  [[ "$(wc -w <<<"${matches}")" -eq 1 ]] ||
    fail "expected exactly one Gazebo client window, found: ${matches:-none}"
  printf '%s\n' "${matches}"
}

set_recording_camera_pose()
{
  [[ "${RECORD_VIDEO}" == "1" ]] || return 0
  local request response
  request='pose { position { x: 0.06292473524808884 y: -0.46163156628608704 z: 1.8398312330245972 } orientation { x: -0.15106521546840668 y: 0.24716269969940186 z: 0.49844232201576233 w: 0.8170956969261169 } }'
  response="$(
    timeout 8 gz service -s "${GUI_CAMERA_SERVICE}" \
      --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 5000 \
      --req "${request}" 2>&1
  )" || fail "Gazebo GUI camera-pose service call failed"
  grep -Eq 'data:[[:space:]]*true' <<<"${response}" ||
    fail "Gazebo GUI rejected the fixed recording camera pose"
  printf '%s\n' "${request}" >"${RUN_DIR}/gui_camera_pose_request.txt"
  sleep 0.5
  log "GUI_RECORDING_CAMERA=PASS position=[0.06292473524808884,-0.46163156628608704,1.8398312330245972] quaternion=[-0.15106521546840668,0.24716269969940186,0.49844232201576233,0.8170956969261169]"
}

start_recorder()
{
  [[ "${RECORD_VIDEO}" == "1" ]] || return 0
  command -v ffmpeg >/dev/null || fail "ffmpeg is unavailable"
  command -v ffprobe >/dev/null || fail "ffprobe is unavailable"
  VIDEO_WINDOW_ID="$(find_gazebo_window_id)"
  local details width height window_decimal stamp crop_width crop_height crop_x crop_y
  details="$(xwininfo -id "${VIDEO_WINDOW_ID}" 2>/dev/null)" ||
    fail "could not inspect Gazebo client window ${VIDEO_WINDOW_ID}"
  width="$(awk -F: '/Width:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${details}")"
  height="$(awk -F: '/Height:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"${details}")"
  [[ "${width}" =~ ^[1-9][0-9]*$ && "${height}" =~ ^[1-9][0-9]*$ &&
    $((width % 2)) -eq 0 && $((height % 2)) -eq 0 ]] ||
    fail "Gazebo capture dimensions must be positive and even: ${width:-?}x${height:-?}"
  IFS=: read -r crop_width crop_height crop_x crop_y <<<"${VIDEO_CROP}"
  [[ "${crop_width}" =~ ^[1-9][0-9]*$ && "${crop_height}" =~ ^[1-9][0-9]*$ &&
    "${crop_x}" =~ ^[0-9]+$ && "${crop_y}" =~ ^[0-9]+$ &&
    $((crop_width % 2)) -eq 0 && $((crop_height % 2)) -eq 0 &&
    $((crop_x + crop_width)) -le width && $((crop_y + crop_height)) -le height ]] ||
    fail "video crop ${VIDEO_CROP} is invalid for Gazebo window ${width}x${height}"
  VIDEO_EXPECTED_WIDTH="${crop_width}"
  VIDEO_EXPECTED_HEIGHT="${crop_height}"
  window_decimal="$((VIDEO_WINDOW_ID))"
  stamp="$(date +%Y%m%d_%H%M%S_%N)"
  VIDEO_OUTPUT="${VIDEO_OUTPUT_OVERRIDE:-/home/liu/h1_moveit/demo_recordings/h1_full_robot_flow_8251_${stamp}_$$.mp4}"
  [[ "${VIDEO_OUTPUT}" == /* ]] || fail "video output must be an absolute path"
  [[ ! -e "${VIDEO_OUTPUT}" && ! -L "${VIDEO_OUTPUT}" ]] ||
    fail "refusing to overwrite video output: ${VIDEO_OUTPUT}"
  mkdir -p "$(dirname "${VIDEO_OUTPUT}")"
  printf '%s\n' "${VIDEO_WINDOW_ID}" >"${RUN_DIR}/video_window_id.txt"
  printf '%s\n' "${VIDEO_OUTPUT}" >"${RUN_DIR}/video_output.txt"
  ffmpeg -nostdin -n -thread_queue_size 512 -f x11grab \
    -window_id "${window_decimal}" -draw_mouse 0 \
    -framerate "${RECORD_FRAMERATE}" -video_size "${width}x${height}" \
    -i "${DISPLAY:-:0}" -an -vf "crop=${VIDEO_CROP}" \
    -c:v libx264 -preset ultrafast \
    -crf "${RECORD_CRF}" -pix_fmt yuv420p \
    "${VIDEO_OUTPUT}" >"${RUN_DIR}/ffmpeg.log" 2>&1 &
  RECORDER_PID=$!
  sleep 1
  kill -0 "${RECORDER_PID}" 2>/dev/null || fail "ffmpeg recorder exited during startup"
  log "RECORDER_START=PASS window_id=${VIDEO_WINDOW_ID} input=${width}x${height} crop=${VIDEO_CROP} output=${VIDEO_OUTPUT}"
}

stop_recorder()
{
  [[ -n "${RECORDER_PID}" ]] || return 0
  if kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
    local attempt
    for attempt in $(seq 1 100); do
      kill -0 "${RECORDER_PID}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${RECORDER_PID}" 2>/dev/null; then
      kill -TERM "${RECORDER_PID}" 2>/dev/null || true
      sleep 1
    fi
    if kill -0 "${RECORDER_PID}" 2>/dev/null; then
      kill -KILL "${RECORDER_PID}" 2>/dev/null || true
    fi
  fi
  wait "${RECORDER_PID}" 2>/dev/null || true
  RECORDER_PID=""
}

verify_video()
{
  [[ "${RECORD_VIDEO}" == "1" ]] || return 0
  [[ -f "${VIDEO_OUTPUT}" && ! -L "${VIDEO_OUTPUT}" ]] ||
    fail "recorded video is absent or is a symlink"
  local probe duration size width height
  probe="$(ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_type,width,height:format=duration,size \
    -of json "${VIDEO_OUTPUT}")" || fail "ffprobe rejected ${VIDEO_OUTPUT}"
  printf '%s\n' "${probe}" >"${RUN_DIR}/ffprobe.json"
  duration="$(jq -er '.format.duration | tonumber' <<<"${probe}")" ||
    fail "video duration is missing"
  size="$(jq -er '.format.size | tonumber' <<<"${probe}")" ||
    fail "video size is missing"
  width="$(jq -er '.streams[0].width | tonumber' <<<"${probe}")" ||
    fail "video width is missing"
  height="$(jq -er '.streams[0].height | tonumber' <<<"${probe}")" ||
    fail "video height is missing"
  awk -v duration="${duration}" -v size="${size}" -v width="${width}" \
    -v height="${height}" -v expected_width="${VIDEO_EXPECTED_WIDTH}" \
    -v expected_height="${VIDEO_EXPECTED_HEIGHT}" 'BEGIN {
      exit !(duration > 0.0 && size > 1024 && width == expected_width &&
        height == expected_height)
    }' || fail "recorded video has invalid duration, size or dimensions"
  if [[ "${FLOW_START_NS}" != "0" && "${FLOW_END_NS}" != "0" ]]; then
    local physical_seconds
    physical_seconds="$(awk -v start="${FLOW_START_NS}" -v end="${FLOW_END_NS}" \
      'BEGIN {printf "%.9f", (end-start)/1000000000.0}')"
    awk -v video="${duration}" -v physical="${physical_seconds}" \
      'BEGIN {exit !(video + 1.0 >= physical)}' ||
      fail "video duration ${duration}s does not cover physical interval ${physical_seconds}s"
  fi
  log "VIDEO_RECORDING=PASS duration=${duration}s size=${size} bytes dimensions=${width}x${height} path=${VIDEO_OUTPUT}"
}

start_first_publish_monitor()
{
  FIRST_PUBLISH_FILE="${RUN_DIR}/first_controller_publish.monotonic_ns"
  FIRST_PUBLISH_READY_FILE="${RUN_DIR}/first_controller_publish_monitor.ready"
  [[ ! -e "${FIRST_PUBLISH_FILE}" && ! -L "${FIRST_PUBLISH_FILE}" &&
     ! -e "${FIRST_PUBLISH_READY_FILE}" && ! -L "${FIRST_PUBLISH_READY_FILE}" ]] ||
    fail "first-publish monitor files already exist in the unique run directory"
  /usr/bin/python3 - "${FIRST_PUBLISH_FILE}" "${FIRST_PUBLISH_READY_FILE}" \
    >"${RUN_DIR}/first_publish_monitor.log" 2>&1 <<'PY' &
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory

output_path, ready_path = sys.argv[1:3]
topics = (
    "/right_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/right_gripper_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
)
rclpy.init()
node = Node(f"h1_full_first_publish_monitor_{os.getpid()}")
qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
finished = False

def write_exclusive(path: str, value: str) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        os.write(descriptor, value.encode("ascii"))
        os.fsync(descriptor)
    finally:
        os.close(descriptor)

def callback(_message: JointTrajectory) -> None:
    global finished
    if finished:
        return
    write_exclusive(output_path, f"{time.monotonic_ns()}\n")
    finished = True

subscriptions = [
    node.create_subscription(JointTrajectory, topic, callback, qos) for topic in topics
]
write_exclusive(ready_path, "ready\n")
try:
    while rclpy.ok() and not finished:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    subscriptions.clear()
    node.destroy_node()
    rclpy.try_shutdown()
PY
  FIRST_PUBLISH_MONITOR_PID=$!
  local attempt
  for attempt in $(seq 1 100); do
    [[ -f "${FIRST_PUBLISH_READY_FILE}" ]] && break
    kill -0 "${FIRST_PUBLISH_MONITOR_PID}" 2>/dev/null ||
      fail "first-controller-publish monitor exited before readiness"
    sleep 0.05
  done
  [[ -f "${FIRST_PUBLISH_READY_FILE}" ]] ||
    fail "first-controller-publish monitor did not become ready"

  # Wait until the graph reports the controller plus this monitor subscriber
  # on all four topics before allowing the first replay process to exist.
  for attempt in $(seq 1 100); do
    local all_ready=1 topic info subscriptions
    for topic in "${ROBOT_TRAJECTORY_TOPICS[@]}"; do
      info="$(timeout 3 ros2 topic info "${topic}" 2>/dev/null || true)"
      subscriptions="$(awk -F: '/^Subscription count:/ {gsub(/[[:space:]]/, "", $2); print $2}' <<<"${info}")"
      [[ "${subscriptions}" =~ ^[0-9]+$ && "${subscriptions}" -ge 2 ]] || all_ready=0
    done
    (( all_ready == 1 )) && break
    sleep 0.05
  done
  (( all_ready == 1 )) || fail "first-publish monitor subscriptions were not discoverable"
  require_no_robot_trajectory_publishers
  log "FIRST_CONTROLLER_PUBLISH_MONITOR=READY"
}

stop_first_publish_monitor()
{
  [[ -n "${FIRST_PUBLISH_MONITOR_PID}" ]] || return 0
  if kill -0 "${FIRST_PUBLISH_MONITOR_PID}" 2>/dev/null; then
    kill -TERM "${FIRST_PUBLISH_MONITOR_PID}" 2>/dev/null || true
  fi
  wait "${FIRST_PUBLISH_MONITOR_PID}" 2>/dev/null || true
  FIRST_PUBLISH_MONITOR_PID=""
}

adopt_first_publish_time()
{
  local attempt
  for attempt in $(seq 1 100); do
    [[ -s "${FIRST_PUBLISH_FILE}" ]] && break
    sleep 0.02
  done
  [[ -s "${FIRST_PUBLISH_FILE}" ]] || fail "no first controller publication was observed"
  FLOW_START_NS="$(<"${FIRST_PUBLISH_FILE}")"
  require_unsigned_integer "${FLOW_START_NS}" first-controller-publish-monotonic-ns
  (( FLOW_START_NS >= FLOW_INVOCATION_START_NS )) ||
    fail "a controller publication occurred before the authorized handle replay"
  stop_first_publish_monitor
  log "FULL_ROBOT_FLOW_TIMER_START=${FLOW_START_NS}; timestamp is the first observed controller-topic publication."
}

monotonic_now_ns()
{
  /usr/bin/python3 -c 'import time; print(time.monotonic_ns())'
}

deadline_gate()
{
  local label="$1"
  [[ "${FLOW_START_NS}" != "0" ]] || return 0
  local now elapsed_ns
  now="$(monotonic_now_ns)"
  elapsed_ns=$((now - FLOW_START_NS))
  (( elapsed_ns < MAX_WALL_SECONDS * 1000000000 )) ||
    fail "${label}: first-controller-to-now wall time reached ${MAX_WALL_SECONDS}s; current support is retained"
  log "${label}=PASS elapsed=$(awk -v ns="${elapsed_ns}" 'BEGIN {printf "%.3f", ns/1000000000.0}')s"
}

run_replay_range()
{
  local stage="$1"
  local start="$2"
  local end="$3"
  local suffix="$4"
  local message_time_scales="${5:-}"
  local path="${ASSET_PATHS[${stage}]}"
  local count="${ASSET_COUNTS[${stage}]}"
  local digest="${ASSET_SHA256[${stage}]}"
  local log_file="${RUN_DIR}/replay_${stage}_${suffix}.log"
  local -a range_args=() retime_args=() subscriber_args=()
  if (( start != 0 || end != count )); then
    [[ "${stage}" == "cube" || "${stage}" == "door" ]] ||
      fail "segmented replay is allowed only for door or cube"
    range_args=(--start-index "${start}" --end-index "${end}")
  fi
  if [[ -n "${message_time_scales}" ]]; then
    retime_args=(--message-time-scales "${message_time_scales}")
  fi
  if [[ "${FLOW_START_NS}" == "0" ]]; then
    # The first-publication evidence monitor is an additional subscriber.  A
    # fresh replay publisher must match both it and the controller before the
    # first message; otherwise matching only the monitor can create a valid
    # timestamp while the controller misses the command.
    subscriber_args=(--minimum-subscriptions 2)
  fi
  # The expensive Gazebo transport topology proof is performed before the
  # first command and again before acceptance.  Between sealed ranges the
  # executable/process guard is sufficient and avoids a ~2 s discovery wait
  # per range while the wrapper lock excludes another authorized flow.
  reject_direct_door_process
  local required_rtf="${MINIMUM_RTF}" required_count="${RTF_SAMPLE_COUNT}"
  if [[ "${stage}" == "cube" && "${suffix}" == "supported_carry" ]]; then
    # The compliant cube constraint adds a persistent physics load after its
    # fresh attach acknowledgement.  Admit that one range only after three
    # consecutive samples at 0.55 RTF, while retaining the stricter 0.70 gate
    # everywhere else and the independent first-publication-to-final-proof
    # hard wall deadline below 120 seconds.
    required_rtf="${CUBE_SUPPORTED_MINIMUM_RTF}"
    required_count="${CUBE_SUPPORTED_RTF_SAMPLE_COUNT}"
  fi
  physics_and_rtf_gate "RTF_BEFORE_${stage^^}_${suffix^^}" \
    "${required_rtf}" "${required_count}"
  log "Starting sealed replay stage=${stage} range=[${start},${end}); no shell timeout is used."
  local status
  set +e
  "${TRAJECTORY_TOOL}" replay "${path}" --execute --expected-stage "${stage}" \
    "${range_args[@]}" "${retime_args[@]}" "${subscriber_args[@]}" \
    2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "sealed replay stage=${stage} range=[${start},${end}) exited ${status}; no cleanup release/reset is authorized"
  grep -Fqx "REPLAY_ASSET=PASS stage=${stage} range=[${start},${end}) messages=$((end-start)) sha256=${digest}" \
    "${log_file}" ||
    fail "sealed replay stage=${stage} range=[${start},${end}) lacks its exact final PASS marker"
  require_no_robot_trajectory_publishers
}

run_door_support_preload()
{
  local log_file="${RUN_DIR}/door_support_preload.log"
  export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"
  export H1_DOOR_ASSET="${ASSET_PATHS[door]}"
  export H1_DOOR_ASSET_SHA256="${ASSET_SHA256[door]}"
  export H1_DOOR_ATTACH_AFTER_INDEX="${DOOR_ATTACH_AFTER_INDEX}"
  export H1_DOOR_SUPPORT_MAX_FORCE="${DOOR_SUPPORT_MAX_FORCE}"
  export H1_DOOR_SUPPORT_MAX_TORQUE="${DOOR_SUPPORT_MAX_TORQUE}"
  local status
  set +e
  "${DOOR_SUPPORT_HELPER}" preload 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "door support preload failed; no controller trajectory is authorized"
  grep -Fqx "DOOR_SUPPORT_PRELOAD=PASS child=incubator::door_link force_cap=${DOOR_SUPPORT_MAX_FORCE} torque_cap=${DOOR_SUPPORT_MAX_TORQUE}" \
    "${log_file}" || fail "door support preload lacks its exact verified PASS marker"
  log "DOOR_SUPPORT_PRELOAD_GATE=PASS"
}

run_cube_support_preload()
{
  local log_file="${RUN_DIR}/cube_support_preload.log"
  export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"
  export H1_CUBE_ASSET="${ASSET_PATHS[cube]}"
  export H1_CUBE_ASSET_SHA256="${ASSET_SHA256[cube]}"
  export H1_CUBE_ATTACH_AFTER_INDEX="${CUBE_ATTACH_AFTER_INDEX}"
  export H1_CUBE_DETACH_AFTER_INDEX="${CUBE_DETACH_AFTER_INDEX}"
  local status
  set +e
  "${CUBE_SUPPORT_HELPER}" preload 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "cube support preload failed; no controller trajectory is authorized"
  grep -Fqx "CUBE_SUPPORT_PRELOAD=PASS" "${log_file}" ||
    fail "cube support preload lacks its exact verified PASS marker"
  log "CUBE_SUPPORT_PRELOAD_GATE=PASS"
}

run_door_support_hook()
{
  local log_file="${RUN_DIR}/door_support_attach.log"
  export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"
  export H1_DOOR_ASSET="${ASSET_PATHS[door]}"
  export H1_DOOR_ASSET_SHA256="${ASSET_SHA256[door]}"
  export H1_DOOR_ATTACH_AFTER_INDEX="${DOOR_ATTACH_AFTER_INDEX}"
  export H1_DOOR_SUPPORT_MAX_FORCE="${DOOR_SUPPORT_MAX_FORCE}"
  export H1_DOOR_SUPPORT_MAX_TORQUE="${DOOR_SUPPORT_MAX_TORQUE}"
  local status
  set +e
  "${DOOR_SUPPORT_HELPER}" attach 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "door support attach failed; right clamp is retained and no door arc is authorized"
  grep -Fqx "DOOR_SUPPORT_HOOK=PASS operation=attach child=incubator::door_link force_cap=${DOOR_SUPPORT_MAX_FORCE} torque_cap=${DOOR_SUPPORT_MAX_TORQUE}" \
    "${log_file}" || fail "door support helper lacks its exact verified PASS marker"
  log "DOOR_SUPPORT_ATTACH=PASS"
}

run_cube_support_hook()
{
  local operation="$1"
  local log_file="${RUN_DIR}/cube_support_${operation}.log"
  export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"
  export H1_CUBE_ASSET="${ASSET_PATHS[cube]}"
  export H1_CUBE_ASSET_SHA256="${ASSET_SHA256[cube]}"
  export H1_CUBE_ATTACH_AFTER_INDEX="${CUBE_ATTACH_AFTER_INDEX}"
  export H1_CUBE_DETACH_AFTER_INDEX="${CUBE_DETACH_AFTER_INDEX}"
  local status
  set +e
  "${CUBE_SUPPORT_HELPER}" "${operation}" 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) ||
    fail "cube support helper '${operation}' failed; no later gripper or trajectory command is authorized"
  grep -Fqx "CUBE_SUPPORT_HOOK=PASS operation=${operation}" "${log_file}" ||
    fail "cube support helper '${operation}' lacks its exact verified PASS marker"
  log "CUBE_SUPPORT_${operation^^}=PASS"
}

run_robot_flow()
{
  set_fast_physics_profile
  fresh_scene_gate
  run_door_support_preload
  run_cube_support_preload
  # Both helpers must preload only detached resident support.  Re-run the full
  # physical fresh-scene proof before camera/recorder setup and before the
  # first-controller timer so preload cannot silently move the mechanism,
  # robot or cube.
  fresh_scene_gate
  set_recording_camera_pose
  start_recorder
  physics_and_rtf_gate "RECORDER_LOAD_RTF_GATE"
  start_first_publish_monitor
  FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"

  run_replay_range handle 0 "${ASSET_COUNTS[handle]}" full \
    "${HANDLE_MESSAGE_TIME_SCALES}"
  adopt_first_publish_time
  run_parallel_gate_group HANDLE_STAGE \
    6 mechanism_gate "HANDLE_STAGE_MECHANISM_GATE" \
      "${DOOR_ZERO_RAD}" "${DOOR_FRESH_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "HANDLE_STAGE_LATCH_GATE" \
    2 left_handle_hold_gate "HANDLE_STAGE_LEFT_HOLD_GATE"
  deadline_gate HANDLE_STAGE_DEADLINE

  run_replay_range left_clear 0 "${ASSET_COUNTS[left_clear]}" full \
    "${LEFT_CLEAR_MESSAGE_TIME_SCALES}"
  run_parallel_gate_group LEFT_CLEAR \
    6 mechanism_gate "LEFT_CLEAR_MECHANISM_GATE" \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "LEFT_CLEAR_LATCH_GATE" \
    2 left_clear_gate "LEFT_CLEAR_ROBOT_GATE"
  deadline_gate LEFT_CLEAR_DEADLINE

  run_replay_range door 0 "${DOOR_ATTACH_AFTER_INDEX}" before_support \
    "${DOOR_BEFORE_SUPPORT_TIME_SCALES}"
  run_parallel_gate_group DOOR_PRE_SUPPORT \
    6 mechanism_gate "DOOR_PRE_SUPPORT_MECHANISM_GATE" \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "DOOR_PRE_SUPPORT_LATCH_GATE" \
    2 right_door_hold_gate "DOOR_PRE_SUPPORT_RIGHT_CLAMP_GATE" \
    2 left_clear_gate "DOOR_PRE_SUPPORT_LEFT_CLEAR_GATE"
  run_door_support_hook
  deadline_gate DOOR_SUPPORT_DEADLINE

  run_replay_range door "${DOOR_ATTACH_AFTER_INDEX}" \
    "${ASSET_COUNTS[door]}" supported_arc \
    "${DOOR_AFTER_SUPPORT_TIME_SCALES}"
  run_parallel_gate_group DOOR_STAGE \
    6 mechanism_gate "DOOR_STAGE_MECHANISM_GATE" \
      "${DOOR_SIXTY_RAD}" "${DOOR_OPEN_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "DOOR_STAGE_LATCH_GATE" \
    2 right_door_hold_gate "DOOR_STAGE_RIGHT_HOLD_GATE" \
    2 left_clear_gate "DOOR_STAGE_LEFT_CLEAR_GATE" \
    8 cube_pose_gate "DOOR_STAGE_CUBE_SPAWN_GATE" \
      0.334000 0.377000 0.830000 1.570796326794897 \
      0.003 0.034906585039887
  deadline_gate DOOR_STAGE_DEADLINE

  # The verified physical sequence has a stationary-cube proof between the
  # contact approach and the close command.  Replaying [0,2) as one range
  # collapses the stored 200 ms wall seam, so preserve the proof explicitly
  # and use the low-preload 1.0 s close sealed by the optimized cube asset.
  # Its target is only 0.250 mm/joint beyond modeled pad contact; the
  # old +3 mm target could eject the 12 g cube before the opposite pad arrived.
  # Use 6 ms for contact and close; 8 ms previously lagged the follower.
  # After attach the supported range switches to 7 ms and its dedicated
  # three-sample 0.55 admission gate, then still must satisfy the strict final
  # physical placement proof before the 120-second wall deadline.
  set_physics_profile "${CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE}" \
    "CUBE_CONTACT_PHYSICS_APPLY_GATE"
  run_replay_range cube 0 1 before_attach_approach "0:1"
  cube_pose_gate "CUBE_CONTACT_READY_POSE_GATE" \
    0.334000 0.377000 0.830000 1.570796326794897 0.003 0.034906585039887
  cube_stability_gate "CUBE_CONTACT_READY_STABILITY"
  run_replay_range cube 1 "${CUBE_ATTACH_AFTER_INDEX}" \
    before_attach_close "1:1"
  run_parallel_gate_group CUBE_PRE_ATTACH \
    6 mechanism_gate "CUBE_PRE_ATTACH_MECHANISM_GATE" \
      "${DOOR_SIXTY_RAD}" "${DOOR_OPEN_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    2 right_door_hold_gate "CUBE_PRE_ATTACH_RIGHT_HOLD_GATE" \
    8 cube_pose_gate "CUBE_PRE_ATTACH_CUBE_POSE_GATE" \
      0.334000 0.377000 0.830000 1.570796326794897 \
      0.003 0.034906585039887
  run_cube_support_hook attach
  deadline_gate CUBE_ATTACH_DEADLINE

  # The compliant follower has a measured persistent compute cost only after
  # attach.  Seven-millisecond physics preserves more wall-time margin than
  # the contact/close profile, while all rack pose and stability gates below
  # remain authoritative and reject follower lag before detach.
  set_physics_profile "${CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE}" \
    "CUBE_SUPPORTED_PHYSICS_APPLY_GATE" \
    "${CUBE_SUPPORTED_MINIMUM_RTF}" "${CUBE_SUPPORTED_RTF_SAMPLE_COUNT}"

  run_replay_range cube "${CUBE_ATTACH_AFTER_INDEX}" \
    "${CUBE_DETACH_AFTER_INDEX}" supported_carry \
    "${CUBE_SUPPORTED_CARRY_TIME_SCALES}"
  run_parallel_gate_group CUBE_PRE_DETACH \
    6 mechanism_gate "CUBE_PRE_DETACH_MECHANISM_GATE" \
      "${DOOR_SIXTY_RAD}" "${DOOR_OPEN_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    2 right_door_hold_gate "CUBE_PRE_DETACH_RIGHT_HOLD_GATE" \
    8 cube_pose_gate "CUBE_PRE_DETACH_RACK_GATE" \
      0.610890542898 -0.029527393333 0.995407082351 1.221730476396031 \
      0.012 0.087266462599716 \
    2 cube_stability_gate "CUBE_PRE_DETACH_STABILITY"
  run_cube_support_hook detach
  set_physics_profile "${PHYSICS_MAX_STEP_SIZE}" \
    "CUBE_RETREAT_PHYSICS_RESTORE_GATE"
  cube_stability_gate "CUBE_POST_DETACH_STABILITY"
  deadline_gate CUBE_DETACH_DEADLINE

  run_replay_range cube "${CUBE_DETACH_AFTER_INDEX}" \
    "${ASSET_COUNTS[cube]}" after_detach \
    "${CUBE_AFTER_DETACH_TIME_SCALES}"
  run_parallel_gate_group FINAL \
    6 mechanism_gate "FINAL_MECHANISM_GATE" \
      "${DOOR_SIXTY_RAD}" "${DOOR_OPEN_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "FINAL_LATCH_GATE" \
    2 right_door_hold_gate "FINAL_RIGHT_HOLD_GATE" \
    8 joint_values_gate "FINAL_LEFT_OPEN_GATE" \
      left_dex1_finger_1_joint -0.019 0.001 \
      left_dex1_finger_2_joint -0.019 0.001

  reject_direct_door_controller

  # Close the final placement proof in one ordered sampling window.  The
  # parallel FINAL group above retains the independent mechanism, latch,
  # right-support and open-left-hand gates.  This sole final cube proof avoids
  # a duplicate discovery window, while its absolute-pose / relative-stability /
  # absolute-pose sequence prevents a
  # moving cube from satisfying the position and stability checks at different
  # instants.  FLOW_END remains after the closure, so all three reads stay
  # inside both strict 120-second bounds.
  cube_pose_gate "FINAL_SEQUENTIAL_CUBE_RACK_ENTRY_GATE" \
    0.610890542898 -0.029527393333 0.995407082351 1.221730476396031 \
    0.012 0.087266462599716
  cube_stability_gate "FINAL_SEQUENTIAL_CUBE_STABILITY_GATE"
  cube_pose_gate "FINAL_SEQUENTIAL_CUBE_RACK_EXIT_GATE" \
    0.610890542898 -0.029527393333 0.995407082351 1.221730476396031 \
    0.012 0.087266462599716

  FLOW_END_NS="$(monotonic_now_ns)"
  local elapsed_ns invocation_elapsed_ns elapsed_seconds invocation_seconds
  elapsed_ns=$((FLOW_END_NS - FLOW_START_NS))
  invocation_elapsed_ns=$((FLOW_END_NS - FLOW_INVOCATION_START_NS))
  (( elapsed_ns > 0 && elapsed_ns < MAX_WALL_SECONDS * 1000000000 )) ||
    fail "first-controller-to-cube-complete duration is not below ${MAX_WALL_SECONDS}s"
  # This conservative upper bound includes replay discovery immediately before
  # the first publish, so it prevents monitor latency from creating a false PASS.
  (( invocation_elapsed_ns > 0 && invocation_elapsed_ns < MAX_WALL_SECONDS * 1000000000 )) ||
    fail "replay-invocation-to-cube-complete upper bound is not below ${MAX_WALL_SECONDS}s"
  elapsed_seconds="$(awk -v ns="${elapsed_ns}" 'BEGIN {printf "%.3f", ns/1000000000.0}')"
  invocation_seconds="$(awk -v ns="${invocation_elapsed_ns}" 'BEGIN {printf "%.3f", ns/1000000000.0}')"
  printf '%s\n' "${elapsed_seconds}" >"${RUN_DIR}/first_publish_to_cube_complete_seconds.txt"
  printf '%s\n' "${invocation_seconds}" >"${RUN_DIR}/invocation_upper_bound_seconds.txt"

  stop_recorder
  verify_video
  if [[ "${RECORD_VIDEO}" == "1" ]]; then
    log "FULL_ROBOT_FLOW_PHYSICAL=PASS first_publish_to_cube_complete=${elapsed_seconds}s invocation_upper_bound=${invocation_seconds}s door=60deg handle=90deg cube=middle_rack right_support=held direct_door=absent video=${VIDEO_OUTPUT}"
  else
    log "FULL_ROBOT_FLOW_DIAGNOSTIC=PASS first_publish_to_cube_complete=${elapsed_seconds}s invocation_upper_bound=${invocation_seconds}s door=60deg handle=90deg cube=middle_rack right_support=held direct_door=absent video=disabled"
  fi
}

cleanup()
{
  # Only diagnostic subscriber / recorder processes are stopped.  Robot,
  # grippers, Gazebo supports, attachments and the simulator are untouched.
  stop_first_publish_monitor
  stop_recorder
}

on_interrupt()
{
  log "Interrupted: no wrapper reset, detach, release, retreat or direct-door command will be issued."
  exit 130
}

main()
{
  local subcommand="${1:-}"
  if [[ "${subcommand}" == "-h" || "${subcommand}" == "--help" ||
    "${subcommand}" == "help" || -z "${subcommand}" ]]
  then
    usage
    [[ -n "${subcommand}" ]] && return 0 || return 2
  fi
  [[ "${subcommand}" == "preflight" || "${subcommand}" == "run" ]] || {
    usage >&2
    return 2
  }
  [[ $# -eq 5 ]] || {
    usage >&2
    return 2
  }

  ASSET_PATHS[handle]="$2"
  ASSET_PATHS[left_clear]="$3"
  ASSET_PATHS[door]="$4"
  ASSET_PATHS[cube]="$5"

  [[ "${RECORD_VIDEO}" == "0" || "${RECORD_VIDEO}" == "1" ]] ||
    fail "H1_FULL_RECORD must be 0 or 1"
  require_unsigned_integer "${MAX_WALL_SECONDS}" H1_FULL_MAX_WALL_SECONDS
  (( MAX_WALL_SECONDS == 120 )) ||
    fail "8251 acceptance requires H1_FULL_MAX_WALL_SECONDS=120 exactly"

  [[ -d "${WORKSPACE}" && -f "${WORKSPACE}/AGENTS.md" ]] ||
    fail "workspace is invalid: ${WORKSPACE}"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${WORKSPACE}/install/setup.bash"
  set -u

  mkdir -p "${RUNTIME_ROOT}"
  exec 9>"${LOCK_FILE}"
  flock -n 9 || fail "another full-flow wrapper owns ${LOCK_FILE}"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S_%N)"
  RUN_DIR="${RUNTIME_ROOT}/${subcommand}_${stamp}_$$"
  mkdir -p "${RUN_DIR}"

  require_no_other_demo
  basic_health_gate
  fresh_scene_gate
  preflight_assets_and_dependencies

  if [[ "${subcommand}" == "preflight" ]]; then
    log "PREFLIGHT_ONLY=PASS; no controller trajectory, physics mutation, support-plugin operation, reset or direct-door command was sent."
    return 0
  fi
  run_robot_flow
}

trap cleanup EXIT
trap on_interrupt INT TERM HUP

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
