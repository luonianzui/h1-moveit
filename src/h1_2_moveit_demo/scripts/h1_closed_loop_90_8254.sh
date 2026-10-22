#!/usr/bin/env bash
# Continuous 90-second Gazebo experiment: unlock, open, place, close, relock.
# It replays only sealed robot-controller assets.  Gazebo support hooks remain
# independent and the original incubator joints are never commanded directly.
set -Eeuo pipefail

readonly closed_loop_mode="${1:-}"
shift || true
[[ $# == 0 && ( "${closed_loop_mode}" == preflight ||
  "${closed_loop_mode}" == prepare-q60 || "${closed_loop_mode}" == run ||
  "${closed_loop_mode}" == reverse-run ||
  "${closed_loop_mode}" == right-middle-run ||
  "${closed_loop_mode}" == right-middle-finalize ) ]] || {
  printf 'usage: h1_closed_loop_90_8254.sh preflight|prepare-q60|run|reverse-run|right-middle-run|right-middle-finalize\n' >&2
  exit 2
}

readonly closed_loop_workspace="${H1_90_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly cube_parallel_yaw_rad='1.570796326794897'
readonly cube_parallel_orientation_tolerance_rad='0.034906585039887'
export H1_FULL_WORKSPACE="${closed_loop_workspace}"
export H1_FULL_RUNTIME_ROOT="${H1_90_RUNTIME_ROOT:-/home/liu/h1_moveit/runtime_logs/closed_loop_90_8254}"
export H1_FULL_LOCK_FILE="${H1_FULL_RUNTIME_ROOT}/active.lock"
export H1_FULL_MAX_WALL_SECONDS=90
export H1_FULL_ROS_SNAPSHOT_TOOL="${closed_loop_workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_ros_snapshot_cache_8254.sh"
export H1_FULL_CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE=0.006
export H1_CUBE_RACK_POSITION_TOLERANCE=0.030
export H1_CUBE_RACK_EXPECTED_YAW=90
export H1_CUBE_RACK_ORIENTATION_TOLERANCE="${cube_parallel_orientation_tolerance_rad}"
export H1_CUBE_ARM_ENDPOINT_TOLERANCE=0.012
export H1_FULL_RECORD="${H1_90_RECORD:-1}"
export H1_FULL_RECORD_FRAMERATE="${H1_90_RECORD_FRAMERATE:-12}"
export H1_FULL_VIDEO_OUTPUT="${H1_90_VIDEO_OUTPUT:-}"
readonly right_regrasp_release_path="${H1_RIGHT_REGRASP_RELEASE_PATH:-}"

# Every selected message is explicit.  These schedules pass the installed
# per-joint position and segment-average velocity limits before any publisher
# is created.  The speed run remains a Gazebo experiment, not a hardware
# safety certificate.
export H1_FULL_HANDLE_MESSAGE_TIME_SCALES='0:0.08,1:0.13,2:0.13,3:0.08,4:0.24'
export H1_FULL_LEFT_CLEAR_MESSAGE_TIME_SCALES='0:0.15,1:0.25'
export H1_FULL_DOOR_BEFORE_SUPPORT_TIME_SCALES='0:0.25,1:0.08,2:0.3,3:0.12,4:0.35,5:0.05'
export H1_FULL_DOOR_AFTER_SUPPORT_TIME_SCALES='6:0.1,7:0.12,8:0.08,9:0.08,10:0.08,11:0.08,12:0.08'
export H1_FULL_CUBE_BEFORE_ATTACH_TIME_SCALES='0:0.36,1:0.08'
export H1_FULL_CUBE_SUPPORTED_CARRY_TIME_SCALES='2:0.36,3:0.36'
export H1_FULL_CUBE_AFTER_DETACH_TIME_SCALES='4:0.08,5:0.36'

# Reuse the mature physical gates, support ownership contracts, camera and
# recorder from 8251.  The guarded source file does not execute its main when
# sourced by this wrapper.
# shellcheck disable=SC1091
source "${closed_loop_workspace}/src/h1_2_moveit_demo/scripts/h1_full_incubator_8251.sh"
trap - EXIT INT TERM HUP

readonly asset_root="${closed_loop_workspace}/runtime_assets"
readonly forward_root="${asset_root}/full_robot_flow_8251"
readonly reverse_root="${asset_root}/close_and_relock_8253"
readonly fast_root="${asset_root}/closed_loop_90_8254"
readonly parallel_cube_root="${asset_root}/parallel_cube_8262"
readonly handle_asset="${fast_root}/handle_position_only.trajectory.json"
readonly left_clear_asset="${forward_root}/left_clear.trajectory.json"
readonly door_open_asset="${forward_root}/door.trajectory.json"
readonly cube_asset="${parallel_cube_root}/cube.parallel90.neutral.trajectory.json"
readonly door_close_asset="${reverse_root}/door_close.trajectory.json"
readonly left_regrasp_asset="${reverse_root}/left_regrasp.trajectory.json"
readonly handle_relock_asset="${reverse_root}/handle_relock.trajectory.json"

readonly snapshot_monitor="${closed_loop_workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_ros_snapshot_monitor_8254.py"
readonly persistent_runner="${closed_loop_workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_persistent_trajectory_runner_8254.py"
readonly detach_helper="${closed_loop_workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_door_support_detach_8253.sh"

readonly handle_payload_sha='c03df7c51daac6d2059ab5316d17dd9d486191d1bc2aadd0395cfe73044adc6b'
readonly handle_file_sha='89e35a80b3d012871935dff280a93e878d3f9e7d0eb8f51e23bcfc1f0a310f0b'
readonly left_clear_payload_sha='066b2437fa320d2db895a927e24f0d5044ff2a9786638e335f8117695e105b88'
readonly left_clear_file_sha='9e3daae8d7fdc20702a31ec60c475d78da08f05b91550279779845ab22dcc0b3'
readonly door_open_payload_sha='867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19'
readonly door_open_file_sha='c481657ac205017a5be16646532236c97dedf909eb0571297e70f9d8690131c9'
readonly cube_payload_sha='acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705'
readonly cube_file_sha='09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134'
readonly door_close_payload_sha='5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038'
readonly door_close_file_sha='3cbb2decfe3128e6a7ca95238085f4285b06a79486c40733db7d426d0a3e5afb'
readonly left_regrasp_payload_sha='5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d'
readonly left_regrasp_file_sha='8d4443c50e9d5d18bc4b1df0cef6a5af132fd4b577c4dada964f3350ec6ef443'
readonly handle_relock_payload_sha='b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3'
readonly handle_relock_file_sha='3d88b71fc9c748b6669831467b7cbd6b6f21f67a5026ea88da99e4bc93fa07e9'

readonly door_close_scales='0:0.08,1:0.08,2:0.08,3:0.08,4:0.08,5:0.12,6:0.1'
readonly right_release_scale='7:0.08'
readonly right_retreat_scales='8:0.341,9:0.105,10:0.285,11:0.075,12:0.235'
readonly left_regrasp_scales='0:0.15,1:0.3'
readonly handle_relock_scales='0:0.234,1:0.052,2:0.131,3:0.131,4:0.052'
readonly handle_relock_rotate_scale='0:0.234'
readonly handle_relock_release_retreat_scales='1:0.052,2:0.131,3:0.131,4:0.052'
readonly cube_release_scale='4:0.08'
readonly cube_retreat_scale='5:0.36'
readonly cube_retreat_minimum_rtf='0.55'
readonly gazebo_state_cache_max_age_ns='500000000'

SNAPSHOT_MONITOR_PID=""
SUPPORT_STATE_MONITOR_PID=""
LATCH_STATE_MONITOR_PID=""
GAZEBO_STATE_CACHE_DIR=""
SUPPORT_STATE_CACHE=""
LATCH_STATE_CACHE=""
PERSISTENT_RUNNER_PID=""
PERSISTENT_COMMAND_DIR=""
PERSISTENT_REQUEST_ID=1
MOTION_STARTED=0

closed_loop_fail()
{
  if (( MOTION_STARTED )); then
    printf '[h1-closed-loop-90-8254] STOP-AND-HOLD: %s No reset, direct incubator command, support cleanup detach, or gripper cleanup was sent.\n' "$*" >&2
  else
    printf '[h1-closed-loop-90-8254] REJECTED WITHOUT MOTION: %s\n' "$*" >&2
  fi
  return 1
}

inspect_bound_asset()
{
  local label="$1" path="$2" expected_stage="$3" expected_count="$4"
  local expected_payload="$5" expected_file="$6" output actual_file
  [[ "${path}" == /* && -f "${path}" && ! -L "${path}" ]] ||
    closed_loop_fail "${label}: bound asset is absent or not a regular file"
  [[ "$(stat -c '%a %u %h' -- "${path}")" == "400 ${EUID} 1" ]] ||
    closed_loop_fail "${label}: asset must be current-user mode-0400 with one link"
  actual_file="$(sha256sum -- "${path}" | awk '{print $1}')"
  [[ "${actual_file}" == "${expected_file}" ]] ||
    closed_loop_fail "${label}: file SHA-256 differs from the wrapper binding"
  output="$("${TRAJECTORY_TOOL}" inspect "${path}" 2>&1)" ||
    closed_loop_fail "${label}: strict asset inspection failed"
  grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${output}" ||
    closed_loop_fail "${label}: schema mismatch"
  grep -Fxq "stage=${expected_stage}" <<<"${output}" ||
    closed_loop_fail "${label}: stage mismatch"
  grep -Fxq "message_count=${expected_count}" <<<"${output}" ||
    closed_loop_fail "${label}: message count mismatch"
  grep -Fxq "sha256=${expected_payload}" <<<"${output}" ||
    closed_loop_fail "${label}: payload SHA-256 mismatch"
  printf '%s\n' "${output}" >"${RUN_DIR}/${label}.inspect.log"
  log "BOUND_ASSET=PASS label=${label} stage=${expected_stage} messages=${expected_count} sha256=${expected_payload}"
}

offline_retime()
{
  local label="$1" path="$2" stage="$3" start="$4" end="$5"
  local scales="$6" digest="$7" output
  output="$("${TRAJECTORY_TOOL}" retime-check "${path}" \
    --expected-stage "${stage}" --start-index "${start}" --end-index "${end}" \
    --message-time-scales "${scales}" 2>&1)" ||
    closed_loop_fail "${label}: offline retime rejected"
  grep -Eq "^RETIME_CHECK=PASS stage=${stage} range=\\[${start},${end}\\).*sha256=${digest}$" <<<"${output}" ||
    closed_loop_fail "${label}: exact retime identity is missing"
  printf '%s\n' "${output}" >"${RUN_DIR}/${label}.retime.log"
}

offline_retime_full()
{
  local label="$1" path="$2" stage="$3" count="$4" scales="$5" digest="$6"
  local output
  output="$("${TRAJECTORY_TOOL}" retime-check "${path}" \
    --expected-stage "${stage}" --message-time-scales "${scales}" 2>&1)" ||
    closed_loop_fail "${label}: offline retime rejected"
  grep -Eq "^RETIME_CHECK=PASS stage=${stage} range=\\[0,${count}\\).*sha256=${digest}$" <<<"${output}" ||
    closed_loop_fail "${label}: exact retime identity is missing"
  printf '%s\n' "${output}" >"${RUN_DIR}/${label}.retime.log"
}

validate_cross_asset_topology()
{
  jq -e '
    (.payload.messages|length)==6 and
    .payload.messages[5].topic=="/left_arm_controller/joint_trajectory" and
    (.payload.messages[5].points[-1].positions[0] | fabs) <= 0.001 and
    (.payload.messages[5].points[-1].positions[1] - 0.30 | fabs) <= 0.001 and
    (.payload.messages[5].points[-1].positions[2] | fabs) <= 0.001 and
    (.payload.messages[5].points[-1].positions[3] - 0.60 | fabs) <= 0.001 and
    (.payload.messages[5].points[-1].positions[4:] | all(fabs <= 0.001))
  ' "${cube_asset}" >/dev/null ||
    closed_loop_fail "cube retreat does not include its sealed neutral endpoint"
  jq -s -e '
    .[0].payload.messages[12].points[-1].positions ==
      .[1].payload.messages[0].points[0].positions
  ' "${door_open_asset}" "${door_close_asset}" >/dev/null ||
    closed_loop_fail "door open and close assets are not exactly connected"
  jq -s -e '
    .[0].payload.messages[0].points[-1].positions as $arm_end |
    .[1].payload.messages[0].points[0].positions as $arm_start |
    .[0].payload.messages[1].points[-1].positions as $finger_end |
    .[1].payload.messages[1].points[0].positions as $finger_start |
    (([range(0;7) | (($arm_end[.] - $arm_start[.]) | fabs)] | max) <= 1e-12) and
    (([range(0;2) | (($finger_end[.] - $finger_start[.]) | fabs)] | max) <= 1e-9)
  ' "${left_regrasp_asset}" "${handle_relock_asset}" >/dev/null ||
    closed_loop_fail "left regrasp and relock assets exceed sealed numerical seams"
  jq -e '
    (.payload.messages|length)==5 and
    .payload.messages[0].topic=="/left_arm_controller/joint_trajectory" and
    .payload.messages[1].topic=="/left_gripper_controller/joint_trajectory" and
    all(.payload.messages[2:5][];
      .topic=="/left_arm_controller/joint_trajectory")
  ' "${handle_relock_asset}" >/dev/null ||
    closed_loop_fail "relock asset does not seal rotate / release / retreat ordering"
  log 'CROSS_ASSET_TOPOLOGY=PASS; cube-neutral and reverse boundaries are within sealed numerical precision.'
}

preflight_all_assets()
{
  inspect_bound_asset handle_fast "${handle_asset}" handle 5 "${handle_payload_sha}" "${handle_file_sha}"
  inspect_bound_asset left_clear "${left_clear_asset}" left_clear 2 "${left_clear_payload_sha}" "${left_clear_file_sha}"
  inspect_bound_asset door_open "${door_open_asset}" door 13 "${door_open_payload_sha}" "${door_open_file_sha}"
  inspect_bound_asset cube "${cube_asset}" cube 6 "${cube_payload_sha}" "${cube_file_sha}"
  inspect_bound_asset door_close "${door_close_asset}" door 13 "${door_close_payload_sha}" "${door_close_file_sha}"
  inspect_bound_asset left_regrasp "${left_regrasp_asset}" left_clear 2 "${left_regrasp_payload_sha}" "${left_regrasp_file_sha}"
  inspect_bound_asset handle_relock "${handle_relock_asset}" handle 5 "${handle_relock_payload_sha}" "${handle_relock_file_sha}"

  ASSET_PATHS[handle]="${handle_asset}"
  ASSET_PATHS[left_clear]="${left_clear_asset}"
  ASSET_PATHS[door]="${door_open_asset}"
  ASSET_PATHS[cube]="${cube_asset}"
  preflight_assets_and_dependencies
  offline_retime door_close "${door_close_asset}" door 0 7 "${door_close_scales}" "${door_close_payload_sha}"
  offline_retime door_release "${door_close_asset}" door 7 8 "${right_release_scale}" "${door_close_payload_sha}"
  offline_retime door_retreat "${door_close_asset}" door 8 13 "${right_retreat_scales}" "${door_close_payload_sha}"
  offline_retime_full left_regrasp "${left_regrasp_asset}" left_clear 2 "${left_regrasp_scales}" "${left_regrasp_payload_sha}"
  offline_retime_full handle_relock "${handle_relock_asset}" handle 5 "${handle_relock_scales}" "${handle_relock_payload_sha}"
  validate_cross_asset_topology
  log 'CLOSED_LOOP_90_ASSET_PREFLIGHT=PASS; all seven assets and twelve timing ranges are sealed before motion.'
}

start_snapshot_monitor()
{
  require_executable "${snapshot_monitor}"
  require_executable "${ROS_SNAPSHOT_TOOL}"
  export H1_ROS_SNAPSHOT_CACHE_DIR="${RUN_DIR}/ros_snapshot_cache"
  "${snapshot_monitor}" --cache-dir "${H1_ROS_SNAPSHOT_CACHE_DIR}" \
    >"${RUN_DIR}/snapshot_monitor.log" 2>&1 &
  SNAPSHOT_MONITOR_PID=$!
  local attempt
  for attempt in $(seq 1 200); do
    if "${ROS_SNAPSHOT_TOOL}" robot --timeout 0.1 >/dev/null 2>&1 &&
      "${ROS_SNAPSHOT_TOOL}" incubator --timeout 0.1 >/dev/null 2>&1
    then
      log "ROS_SNAPSHOT_MONITOR_8254=READY pid=${SNAPSHOT_MONITOR_PID}"
      return 0
    fi
    kill -0 "${SNAPSHOT_MONITOR_PID}" 2>/dev/null ||
      closed_loop_fail "persistent ROS snapshot monitor exited before readiness"
    sleep 0.02
  done
  closed_loop_fail "persistent ROS snapshot monitor did not become ready"
}

start_gazebo_value_monitor()
{
  local topic="$1" cache_file="$2" label="$3" allowed_csv="$4" pid_variable="$5"
  /usr/bin/python3 - "${topic}" "${cache_file}" "${allowed_csv}" \
    >"${RUN_DIR}/${label}_monitor.log" 2>&1 <<'PY' &
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

topic, cache_text, allowed_text = sys.argv[1:4]
cache = Path(cache_text)
allowed = frozenset(allowed_text.split(","))
if not topic.startswith("/") or not allowed or any(not value for value in allowed):
    raise SystemExit("invalid resident Gazebo value-monitor contract")

stopping = False
child = None
sequence = 0
value_pattern = re.compile(r'^\s*data:\s*["\']([^"\']+)["\']\s*$')

def request_stop(_signum, _frame):
    global stopping
    stopping = True
    if child is not None and child.poll() is None:
        child.terminate()

for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(signum, request_stop)

try:
    while not stopping:
        child = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        assert child.stdout is not None
        for line in child.stdout:
            if stopping:
                break
            match = value_pattern.match(line)
            if match is None:
                continue
            value = match.group(1)
            if value not in allowed:
                continue
            sequence += 1
            receipt_ns = time.monotonic_ns()
            temporary = cache.with_name(
                f".{cache.name}.tmp.{os.getpid()}.{sequence}"
            )
            descriptor = os.open(
                temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
            )
            try:
                payload = (
                    "schema=h1_gazebo_value_cache/v1\n"
                    f"topic={topic}\n"
                    f"sequence={sequence}\n"
                    f"receipt_ns={receipt_ns}\n"
                    f"value={value}\n"
                ).encode("utf-8")
                written = 0
                while written < len(payload):
                    count = os.write(descriptor, payload[written:])
                    if count <= 0:
                        raise OSError("short cache write")
                    written += count
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, cache)
        if child.poll() is None:
            child.terminate()
        child.wait(timeout=2.0)
        child = None
        if not stopping:
            # The support topic does not exist until its detached plugin is
            # preloaded.  Retry the same read-only subscription without ever
            # publishing or mutating Gazebo.
            time.sleep(0.05)
finally:
    if child is not None and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2.0)
PY
  printf -v "${pid_variable}" '%s' "$!"
}

read_cached_gazebo_value()
{
  local cache_file="$1" expected_topic="$2"
  [[ -f "${cache_file}" && ! -L "${cache_file}" ]] || return 1
  [[ "$(stat -c '%a %u %h' -- "${cache_file}")" == "600 ${EUID} 1" ]] ||
    return 1
  local content schema topic sequence receipt value now age
  content="$(<"${cache_file}")" || return 1
  [[ "$(awk 'END {print NR}' <<<"${content}")" == 5 ]] || return 1
  schema="$(awk -F= '$1=="schema" {print $2; found++} END {if(found!=1) exit 1}' \
    <<<"${content}")" || return 1
  topic="$(awk -F= '$1=="topic" {print substr($0,index($0,"=")+1); found++} END {if(found!=1) exit 1}' \
    <<<"${content}")" || return 1
  sequence="$(awk -F= '$1=="sequence" {print $2; found++} END {if(found!=1) exit 1}' \
    <<<"${content}")" || return 1
  receipt="$(awk -F= '$1=="receipt_ns" {print $2; found++} END {if(found!=1) exit 1}' \
    <<<"${content}")" || return 1
  value="$(awk -F= '$1=="value" {print $2; found++} END {if(found!=1) exit 1}' \
    <<<"${content}")" || return 1
  [[ "${schema}" == h1_gazebo_value_cache/v1 && "${topic}" == "${expected_topic}" &&
    "${sequence}" =~ ^[1-9][0-9]*$ && "${receipt}" =~ ^[1-9][0-9]*$ &&
    "${value}" =~ ^[a-z]+$ ]] || return 1
  now="$(monotonic_now_ns)"
  (( receipt <= now )) || return 1
  age=$((now - receipt))
  (( age <= gazebo_state_cache_max_age_ns )) || return 1
  printf '%s %s %s\n' "${value}" "${sequence}" "${receipt}"
}

start_gazebo_state_monitors()
{
  GAZEBO_STATE_CACHE_DIR="${RUN_DIR}/gazebo_state_cache"
  mkdir -m 0700 -- "${GAZEBO_STATE_CACHE_DIR}"
  SUPPORT_STATE_CACHE="${GAZEBO_STATE_CACHE_DIR}/right_door_support.state"
  LATCH_STATE_CACHE="${GAZEBO_STATE_CACHE_DIR}/incubator_latch.state"
  start_gazebo_value_monitor /incubator/latch/state \
    "${LATCH_STATE_CACHE}" latch_state 'locked,unlocked' \
    LATCH_STATE_MONITOR_PID

  local attempt
  for attempt in $(seq 1 100); do
    if read_cached_gazebo_value "${LATCH_STATE_CACHE}" \
      /incubator/latch/state >/dev/null 2>&1
    then
      log "GAZEBO_LATCH_STATE_MONITOR=READY pid=${LATCH_STATE_MONITOR_PID}"
      return 0
    fi
    kill -0 "${LATCH_STATE_MONITOR_PID}" 2>/dev/null ||
      closed_loop_fail "resident latch-state monitor exited before readiness"
    sleep 0.02
  done
  closed_loop_fail "resident latch-state monitor did not become ready"
}

# The preload helper requires all three support topics to have zero endpoints.
# Start this read-only subscriber only after the detached plugin has been
# installed and proved, otherwise its transient discovery endpoint races the
# helper's exact empty-topic contract.
start_support_state_monitor()
{
  [[ -n "${GAZEBO_STATE_CACHE_DIR}" && -n "${SUPPORT_STATE_CACHE}" ]] ||
    closed_loop_fail "support-state cache was not initialized"
  [[ -z "${SUPPORT_STATE_MONITOR_PID}" ]] ||
    closed_loop_fail "resident support-state monitor is already running"
  start_gazebo_value_monitor /h1_2/right_door_support/state \
    "${SUPPORT_STATE_CACHE}" support_state 'attached,detached' \
    SUPPORT_STATE_MONITOR_PID

  local attempt
  for attempt in $(seq 1 100); do
    if read_cached_gazebo_value "${SUPPORT_STATE_CACHE}" \
      /h1_2/right_door_support/state >/dev/null 2>&1
    then
      log "GAZEBO_SUPPORT_STATE_MONITOR=READY pid=${SUPPORT_STATE_MONITOR_PID}"
      return 0
    fi
    kill -0 "${SUPPORT_STATE_MONITOR_PID}" 2>/dev/null ||
      closed_loop_fail "resident support-state monitor exited before readiness"
    sleep 0.02
  done
  closed_loop_fail "resident support-state monitor did not become ready"
}

stop_gazebo_state_monitors()
{
  local pid
  for pid in "${SUPPORT_STATE_MONITOR_PID}" "${LATCH_STATE_MONITOR_PID}"; do
    [[ -n "${pid}" ]] || continue
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${SUPPORT_STATE_MONITOR_PID}" "${LATCH_STATE_MONITOR_PID}"; do
    [[ -n "${pid}" ]] || continue
    wait "${pid}" 2>/dev/null || true
  done
  SUPPORT_STATE_MONITOR_PID=""
  LATCH_STATE_MONITOR_PID=""
}

stop_snapshot_monitor()
{
  [[ -n "${SNAPSHOT_MONITOR_PID}" ]] || return 0
  if kill -0 "${SNAPSHOT_MONITOR_PID}" 2>/dev/null; then
    kill -TERM "${SNAPSHOT_MONITOR_PID}" 2>/dev/null || true
  fi
  wait "${SNAPSHOT_MONITOR_PID}" 2>/dev/null || true
  SNAPSHOT_MONITOR_PID=""
}

start_persistent_runner()
{
  require_executable "${persistent_runner}"
  PERSISTENT_COMMAND_DIR="${RUN_DIR}/persistent_runner"
  mkdir -m 0700 -- "${PERSISTENT_COMMAND_DIR}"
  "${persistent_runner}" --command-dir "${PERSISTENT_COMMAND_DIR}" \
    --trajectory-tool "${TRAJECTORY_TOOL}" \
    --initial-minimum-subscriptions 2 \
    --discovery-timeout 20 \
    >"${RUN_DIR}/persistent_runner.log" 2>&1 &
  PERSISTENT_RUNNER_PID=$!
  local attempt
  # DDS discovery is intentionally proved independently inside the runner.
  # Under simultaneous Gazebo rendering/recording load it can take longer
  # than the old four-second shell-side window even though all four controller
  # subscribers and the first-publish monitor are healthy.  Give the worker
  # enough time to complete its unchanged two-subscriber and advancing-clock
  # gates; no publisher can exist outside that fail-closed worker.
  for attempt in $(seq 1 1500); do
    [[ -f "${PERSISTENT_COMMAND_DIR}/ready" ]] && break
    kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null ||
      closed_loop_fail "persistent trajectory runner exited before readiness"
    sleep 0.02
  done
  grep -Eq '^PERSISTENT_RUNNER_8254=READY pid=[1-9][0-9]*$' \
    "${PERSISTENT_COMMAND_DIR}/ready" ||
    closed_loop_fail "persistent trajectory runner ready marker is missing"
  log "PERSISTENT_TRAJECTORY_RUNNER=READY pid=${PERSISTENT_RUNNER_PID}"
}

fast_replay()
{
  local label="$1" path="$2" stage="$3" start="$4" end="$5"
  local scales="$6" digest="$7"
  local start_tolerance="${8:-0.003}" endpoint_tolerance="${9:-0.003}"
  local request response temporary
  kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null ||
    closed_loop_fail "${label}: persistent trajectory runner is absent"
  request="${PERSISTENT_COMMAND_DIR}/request_$(printf '%06d' "${PERSISTENT_REQUEST_ID}").txt"
  response="${PERSISTENT_COMMAND_DIR}/response_$(printf '%06d' "${PERSISTENT_REQUEST_ID}").txt"
  temporary="${PERSISTENT_COMMAND_DIR}/.request_${PERSISTENT_REQUEST_ID}.tmp.$$"
  (umask 077; printf 'run\t%d\t%s\t%s\t%d\t%d\t%s\t%s\t%s\t%s\n' \
    "${PERSISTENT_REQUEST_ID}" "${path}" "${stage}" "${start}" "${end}" \
    "${scales}" "${digest}" "${start_tolerance}" "${endpoint_tolerance}" \
    >"${temporary}")
  chmod 0600 "${temporary}"
  mv -T -- "${temporary}" "${request}"
  MOTION_STARTED=1
  local attempt error_file
  error_file="${PERSISTENT_COMMAND_DIR}/error_$(printf '%06d' "${PERSISTENT_REQUEST_ID}").txt"
  for attempt in $(seq 1 3000); do
    if [[ -f "${response}" ]]; then
      grep -Fxq "PERSISTENT_REPLAY_8254=PASS request=${PERSISTENT_REQUEST_ID} stage=${stage} range=[${start},${end}) messages=$((end-start)) sha256=${digest}" \
        "${response}" || closed_loop_fail "${label}: response identity is wrong"
      log "FAST_REPLAY=PASS label=${label} request=${PERSISTENT_REQUEST_ID} stage=${stage} range=[${start},${end})"
      ((PERSISTENT_REQUEST_ID+=1))
      return 0
    fi
    [[ ! -f "${error_file}" ]] ||
      closed_loop_fail "${label}: $(<"${error_file}")"
    kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null ||
      closed_loop_fail "${label}: persistent trajectory runner exited"
    sleep 0.02
  done
  closed_loop_fail "${label}: persistent replay response timed out"
}

stop_persistent_runner()
{
  [[ -n "${PERSISTENT_RUNNER_PID}" ]] || return 0
  if ! kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null; then
    wait "${PERSISTENT_RUNNER_PID}" 2>/dev/null || true
    PERSISTENT_RUNNER_PID=""
    return 1
  fi
  local request response temporary attempt
  request="${PERSISTENT_COMMAND_DIR}/request_$(printf '%06d' "${PERSISTENT_REQUEST_ID}").txt"
  response="${PERSISTENT_COMMAND_DIR}/response_$(printf '%06d' "${PERSISTENT_REQUEST_ID}").txt"
  temporary="${PERSISTENT_COMMAND_DIR}/.request_stop.tmp.$$"
  (umask 077; printf 'stop\t%d\t\t\t\t\t\t\t\t\n' \
    "${PERSISTENT_REQUEST_ID}" >"${temporary}")
  chmod 0600 "${temporary}"
  mv -T -- "${temporary}" "${request}"
  for attempt in $(seq 1 300); do
    ! kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null && break
    sleep 0.02
  done
  wait "${PERSISTENT_RUNNER_PID}" ||
    closed_loop_fail "persistent trajectory runner did not stop cleanly"
  PERSISTENT_RUNNER_PID=""
  grep -Fxq "PERSISTENT_RUNNER_8254=STOPPING request=${PERSISTENT_REQUEST_ID}" "${response}" ||
    closed_loop_fail "persistent runner stopping identity is missing"
  grep -Fxq 'PERSISTENT_RUNNER_8254=STOPPED publishers=0' \
    "${PERSISTENT_COMMAND_DIR}/stopped" ||
    closed_loop_fail "persistent runner stopped marker is missing"
  ((PERSISTENT_REQUEST_ID+=1))
}

support_state_value()
{
  local row
  row="$(read_cached_gazebo_value "${SUPPORT_STATE_CACHE}" \
    /h1_2/right_door_support/state)" || return 1
  awk '{print $1}' <<<"${row}"
}

support_state_gate()
{
  local expected="$1" label="$2" actual="" attempt
  for attempt in $(seq 1 26); do
    actual="$(support_state_value 2>/dev/null || true)"
    if [[ "${actual}" == "${expected}" ]]; then
      log "${label}=PASS support=${actual} cache=fresh attempt=${attempt}/26"
      return 0
    fi
    kill -0 "${SUPPORT_STATE_MONITOR_PID}" 2>/dev/null ||
      closed_loop_fail "${label}: resident support-state monitor exited"
    (( attempt == 26 )) || sleep 0.02
  done
  closed_loop_fail "${label}: expected fresh cached support=${expected}, found ${actual:-unavailable}"
}

# Override the general wrapper's one-shot `gz topic` latch reader.  The value,
# monotonically increasing sequence and monotonic receipt time are maintained by
# the resident read-only subscriber above; every consumer still requires the
# exact expected state and a sample no older than 0.5 seconds.
require_latch_state()
{
  local expected="$1" label="$2" row="" value="" attempt
  for attempt in $(seq 1 26); do
    row="$(read_cached_gazebo_value "${LATCH_STATE_CACHE}" \
      /incubator/latch/state 2>/dev/null || true)"
    value="$(awk '{print $1}' <<<"${row}")"
    if [[ "${value}" == "${expected}" ]]; then
      log "${label}=PASS latch=${value} cache=fresh attempt=${attempt}/26"
      return 0
    fi
    kill -0 "${LATCH_STATE_MONITOR_PID}" 2>/dev/null ||
      closed_loop_fail "${label}: resident latch-state monitor exited"
    (( attempt == 26 )) || sleep 0.02
  done
  closed_loop_fail "${label}: expected fresh cached latch=${expected}, found ${value:-unavailable}"
}

# The resident monitor refreshes a local cache continuously, so the 90-second
# runner does not need the two multi-second ROS fallback attempts used by the
# general 8251 wrapper.  Keep every settling sample independently bounded.
read_incubator_snapshot()
{
  local snapshot
  snapshot="$(timeout 0.2 "${ROS_SNAPSHOT_TOOL}" incubator --timeout 0.1)" ||
    return 1
  printf '%s\n' "${snapshot}"
}

read_robot_joint_snapshot()
{
  local snapshot
  snapshot="$(timeout 0.2 "${ROS_SNAPSHOT_TOOL}" robot --timeout 0.1)" ||
    return 1
  printf '%s\n' "${snapshot}"
}

settled_mechanism_gate()
{
  local label="$1" attempt last_log
  last_log="${RUN_DIR}/settle_${label}.last.log"
  shift
  for attempt in $(seq 1 11); do
    if ( mechanism_gate "${label}" "$@" ) >"${last_log}" 2>&1; then
      while IFS= read -r line || [[ -n "${line}" ]]; do
        printf '%s\n' "${line}"
      done <"${last_log}"
      log "${label}_SETTLE=PASS attempt=${attempt}/11"
      return 0
    fi
    (( attempt == 11 )) || sleep 0.05
  done
  while IFS= read -r line || [[ -n "${line}" ]]; do
    printf '%s\n' "${line}" >&2
  done <"${last_log}"
  closed_loop_fail "${label}: mechanism did not settle within the bounded retry window"
}

settled_left_handle_hold_gate()
{
  local label="$1" attempt last_log
  last_log="${RUN_DIR}/settle_${label}.last.log"
  for attempt in $(seq 1 11); do
    if ( left_handle_hold_gate "${label}" ) >"${last_log}" 2>&1; then
      while IFS= read -r line || [[ -n "${line}" ]]; do
        printf '%s\n' "${line}"
      done <"${last_log}"
      log "${label}_SETTLE=PASS attempt=${attempt}/11"
      return 0
    fi
    (( attempt == 11 )) || sleep 0.05
  done
  while IFS= read -r line || [[ -n "${line}" ]]; do
    printf '%s\n' "${line}" >&2
  done <"${last_log}"
  closed_loop_fail "${label}: left handle clamp did not settle"
}

settled_cube_release_gate()
{
  local label="$1" attempt consecutive=0
  local last_log="${RUN_DIR}/settle_${label}.last.log"
  for attempt in $(seq 1 16); do
    sleep 0.10
    if ( cube_pose_gate "${label}" \
      0.610890542898 -0.029527393333 0.995407082351 \
      "${cube_parallel_yaw_rad}" 0.025 \
      "${cube_parallel_orientation_tolerance_rad}" ) >"${last_log}" 2>&1
    then
      ((consecutive+=1))
      if (( consecutive == 2 )); then
        cat "${last_log}"
        log "${label}_SETTLE=PASS attempt=${attempt}/16 consecutive=2"
        return 0
      fi
    else
      consecutive=0
    fi
  done
  cat "${last_log}" >&2 || true
  closed_loop_fail "${label}: cube did not settle flat before the left-arm retreat"
}

left_q60_neutral_gate()
{
  joint_values_gate "$1" \
    left_shoulder_pitch_joint 0.0 0.004 \
    left_shoulder_roll_joint 0.30 0.004 \
    left_shoulder_yaw_joint 0.0 0.004 \
    left_elbow_joint 0.60 0.004 \
    left_wrist_pitch_joint 0.0 0.004 \
    left_wrist_roll_joint 0.0 0.004 \
    left_wrist_yaw_joint 0.0 0.004 \
    left_dex1_finger_1_joint -0.019 0.001 \
    left_dex1_finger_2_joint -0.019 0.001
}

q60_checkpoint()
{
  local prefix="$1"
  run_parallel_gate_group "${prefix}" \
    6 settled_mechanism_gate "${prefix}_MECHANISM" \
      "${DOOR_SIXTY_RAD}" "${DOOR_OPEN_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "${prefix}_LATCH" \
    2 right_door_hold_gate "${prefix}_RIGHT_HOLD" \
    3 support_state_gate attached "${prefix}_SUPPORT"
}

q0_unlocked_checkpoint()
{
  local prefix="$1"
  run_parallel_gate_group "${prefix}" \
    6 settled_mechanism_gate "${prefix}_MECHANISM" \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked "${prefix}_LATCH" \
    3 support_state_gate attached "${prefix}_SUPPORT"
}

right_middle_robot_gate()
{
  joint_values_gate "$1" \
    left_shoulder_pitch_joint 0.0 0.004 \
    left_shoulder_roll_joint 0.30 0.004 \
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

settled_right_middle_robot_gate()
{
  local label="$1" attempt consecutive=0
  local last_log="${RUN_DIR}/settle_${label}.last.log"
  for attempt in $(seq 1 31); do
    if ( right_middle_robot_gate "${label}" ) >"${last_log}" 2>&1; then
      ((consecutive+=1))
      if (( consecutive == 2 )); then
        while IFS= read -r line || [[ -n "${line}" ]]; do
          printf '%s\n' "${line}"
        done <"${last_log}"
        log "${label}_SETTLE=PASS attempt=${attempt}/31 consecutive=2"
        return 0
      fi
    else
      consecutive=0
    fi
    (( attempt == 31 )) || sleep 0.05
  done
  while IFS= read -r line || [[ -n "${line}" ]]; do
    printf '%s\n' "${line}" >&2
  done <"${last_log}"
  closed_loop_fail "${label}: robot joints did not settle within the bounded retry window"
}

right_middle_entry_gate()
{
  run_parallel_gate_group RIGHT_MIDDLE_ENTRY \
    6 settled_mechanism_gate RIGHT_MIDDLE_ENTRY_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked RIGHT_MIDDLE_ENTRY_LATCH \
    3 support_state_gate detached RIGHT_MIDDLE_ENTRY_SUPPORT \
    2 right_middle_robot_gate RIGHT_MIDDLE_ENTRY_ROBOT
}

handle_vertical_down_gate()
{
  run_parallel_gate_group HANDLE_VERTICAL_DOWN \
    6 settled_mechanism_gate HANDLE_VERTICAL_DOWN_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_ZERO_RAD}" 0.087266462599716 \
    3 require_latch_state locked HANDLE_VERTICAL_DOWN_LATCH
  log 'HANDLE_VERTICAL_DOWN=PASS door=0deg handle=0deg latch=locked'
}

handle_vertical_down_while_clamped_gate()
{
  run_parallel_gate_group HANDLE_DOWN_WHILE_CLAMPED \
    6 settled_mechanism_gate HANDLE_DOWN_WHILE_CLAMPED_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_ZERO_RAD}" 0.087266462599716 \
    3 require_latch_state locked HANDLE_DOWN_WHILE_CLAMPED_LATCH \
    2 left_handle_hold_gate HANDLE_DOWN_WHILE_CLAMPED_LEFT_HOLD
  log 'HANDLE_DOWN_WHILE_CLAMPED=PASS door=0deg handle=0deg latch=locked left_clamp=held'
}

run_detach_at_q0()
{
  export H1_CLOSE_WORKSPACE="${closed_loop_workspace}"
  export H1_CLOSE_DOOR_ASSET="${door_close_asset}"
  export H1_CLOSE_DOOR_ASSET_SHA256="${door_close_payload_sha}"
  export H1_CLOSE_DOOR_DETACH_AFTER_INDEX=7
  export H1_CLOSE_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"
  export H1_CLOSE_SKIP_PRE_TRANSPORT=1
  export H1_CLOSE_SKIP_POST_TRANSPORT=1
  local status log_file="${RUN_DIR}/door_support_detach.log"
  set +e
  "${detach_helper}" detach 2>&1 | tee "${log_file}"
  status="${PIPESTATUS[0]}"
  set -e
  (( status == 0 )) || closed_loop_fail "q0 door support detach exited ${status}"
  grep -Fq 'DOOR_SUPPORT_DETACH_8253=PASS' "${log_file}" ||
    closed_loop_fail "q0 door support detach PASS marker is missing"
}

run_reverse_sequence()
{
  fast_replay door_close "${door_close_asset}" door 0 7 \
    "${door_close_scales}" "${door_close_payload_sha}"
  if [[ "${FLOW_START_NS}" == 0 ]]; then
    adopt_first_publish_time
  fi
  q0_unlocked_checkpoint CLOSED_SUPPORTED
  deadline_gate CLOSED_SUPPORTED_DEADLINE
  run_detach_at_q0
  support_state_gate detached Q0_DETACHED_GATE
  deadline_gate DETACH_DEADLINE

  fast_replay right_release "${door_close_asset}" door 7 8 \
    "${right_release_scale}" "${door_close_payload_sha}"
  fast_replay right_retreat "${door_close_asset}" door 8 13 \
    "${right_retreat_scales}" "${door_close_payload_sha}"
  run_parallel_gate_group RIGHT_RETREATED \
    6 settled_mechanism_gate RIGHT_RETREATED_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 support_state_gate detached RIGHT_RETREATED_SUPPORT \
    2 cube_stability_gate RIGHT_RETREATED_CUBE_STABILITY
  deadline_gate RIGHT_RETREAT_DEADLINE

  fast_replay left_regrasp "${left_regrasp_asset}" left_clear 0 2 \
    "${left_regrasp_scales}" "${left_regrasp_payload_sha}"
  settled_left_handle_hold_gate LEFT_REGRASP_HOLD_GATE
  # The only authorized handle split is sealed in the persistent runner by the
  # exact reverse-handle payload SHA and the two ranges [0,1) / [1,5).  First
  # rotate q90->q0 while still clamped, prove the live locked/down state, then
  # authorize the stored release and retreat suffix.  No arbitrary handle
  # partial range is accepted.
  fast_replay handle_relock_rotate "${handle_relock_asset}" handle 0 1 \
    "${handle_relock_rotate_scale}" "${handle_relock_payload_sha}"
  handle_vertical_down_while_clamped_gate
  deadline_gate HANDLE_DOWN_WHILE_CLAMPED_DEADLINE
  fast_replay handle_relock_release_retreat "${handle_relock_asset}" handle 1 5 \
    "${handle_relock_release_retreat_scales}" "${handle_relock_payload_sha}"
  FLOW_END_NS="$(monotonic_now_ns)"
  handle_vertical_down_gate
  deadline_gate HANDLE_RELOCK_DEADLINE
}

# Close the door with the right hand and stop at the proved q0/handle90
# checkpoint.  This deliberately excludes both legacy left-handle assets.
run_right_middle_reverse_sequence()
{
  fast_replay door_close "${door_close_asset}" door 0 7 \
    "${door_close_scales}" "${door_close_payload_sha}"
  q0_unlocked_checkpoint RIGHT_MIDDLE_CLOSED_SUPPORTED
  deadline_gate RIGHT_MIDDLE_CLOSED_SUPPORTED_DEADLINE
  run_detach_at_q0
  support_state_gate detached RIGHT_MIDDLE_Q0_DETACHED
  deadline_gate RIGHT_MIDDLE_DETACH_DEADLINE

  fast_replay right_release "${door_close_asset}" door 7 8 \
    "${right_release_scale}" "${door_close_payload_sha}"
  fast_replay right_retreat "${door_close_asset}" door 8 13 \
    "${right_retreat_scales}" "${door_close_payload_sha}"
  FLOW_END_NS="$(monotonic_now_ns)"
  run_parallel_gate_group RIGHT_MIDDLE_RETREATED \
    6 settled_mechanism_gate RIGHT_MIDDLE_RETREATED_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked RIGHT_MIDDLE_RETREATED_LATCH \
    3 support_state_gate detached RIGHT_MIDDLE_RETREATED_SUPPORT \
    2 settled_right_middle_robot_gate RIGHT_MIDDLE_RETREATED_ROBOT \
    2 cube_stability_gate RIGHT_MIDDLE_RETREATED_CUBE
  deadline_gate RIGHT_MIDDLE_RETREAT_DEADLINE
}

# Engineering-only collector entry.  It starts from the same fully proved
# fresh q0 state as the complete run, executes only the sealed unlock/open
# prefix, and deliberately exits at the stationary q60 support checkpoint.
# The resident door support stays attached after process exit so the separate
# no-controller candidate-3 collector can audit a new cube orientation from
# the exact live state.  No cube trajectory, release, reset or direct
# incubator command is issued here.
run_prepare_q60_sequence()
{
  set_fast_physics_profile
  fresh_scene_gate
  run_door_support_preload
  start_support_state_monitor
  fresh_scene_gate
  physics_and_rtf_gate PREPARE_Q60_RTF_GATE
  start_first_publish_monitor
  start_persistent_runner
  FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"

  fast_replay handle_rotate "${handle_asset}" handle 0 5 \
    "${HANDLE_MESSAGE_TIME_SCALES}" "${handle_payload_sha}"
  adopt_first_publish_time
  run_parallel_gate_group HANDLE_DONE \
    6 settled_mechanism_gate HANDLE_DONE_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_FRESH_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked HANDLE_DONE_LATCH \
    2 left_handle_hold_gate HANDLE_DONE_LEFT_HOLD

  fast_replay left_clear "${left_clear_asset}" left_clear 0 2 \
    "${LEFT_CLEAR_MESSAGE_TIME_SCALES}" "${left_clear_payload_sha}"
  left_clear_gate LEFT_CLEAR_FAST_GATE
  fast_replay door_pre_support "${door_open_asset}" door 0 6 \
    "${DOOR_BEFORE_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  right_door_hold_gate DOOR_PRE_SUPPORT_HOLD_GATE
  H1_DOOR_SUPPORT_FAST_RESIDENT=1 run_door_support_hook

  fast_replay door_open_arc "${door_open_asset}" door 6 13 \
    "${DOOR_AFTER_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  q60_checkpoint PREPARE_Q60_FINAL
  left_clear_gate PREPARE_Q60_LEFT_CLEAR
  deadline_gate PREPARE_Q60_DEADLINE
  stop_persistent_runner
  require_no_robot_trajectory_publishers
  log 'CLOSED_LOOP_90_8254_PREPARE_Q60_PHYSICAL=PASS; q60/handle90 stationary, right door support attached, left arm clear, cube untouched, publishers=0.'
}

run_right_middle_sequence()
{
  [[ "${right_regrasp_release_path}" == /* &&
    ! -e "${right_regrasp_release_path}" ]] ||
    closed_loop_fail 'right-middle-run requires an absolute, absent H1_RIGHT_REGRASP_RELEASE_PATH'
  set_fast_physics_profile
  run_door_support_preload
  start_support_state_monitor
  right_middle_entry_gate
  run_cube_support_preload
  right_middle_entry_gate
  set_recording_camera_pose
  start_recorder
  physics_and_rtf_gate RIGHT_MIDDLE_RECORDER_LOAD_RTF_GATE
  start_first_publish_monitor
  start_persistent_runner
  FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"

  fast_replay door_pre_support "${door_open_asset}" door 0 6 \
    "${DOOR_BEFORE_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  adopt_first_publish_time
  right_door_hold_gate RIGHT_MIDDLE_DOOR_PRE_SUPPORT_HOLD
  H1_DOOR_SUPPORT_FAST_RESIDENT=1 run_door_support_hook
  deadline_gate RIGHT_MIDDLE_DOOR_SUPPORT_DEADLINE

  fast_replay door_open_arc "${door_open_asset}" door 6 13 \
    "${DOOR_AFTER_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  q60_checkpoint RIGHT_MIDDLE_DOOR_OPEN_60
  deadline_gate RIGHT_MIDDLE_DOOR_OPEN_DEADLINE

  set_physics_profile "${CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE}" \
    RIGHT_MIDDLE_CUBE_CONTACT_PROFILE
  fast_replay cube_contact "${cube_asset}" cube 0 2 \
    "${CUBE_BEFORE_ATTACH_TIME_SCALES}" "${cube_payload_sha}"
  H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook attach
  set_physics_profile "${CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE}" \
    RIGHT_MIDDLE_CUBE_SUPPORTED_PROFILE "${CUBE_SUPPORTED_MINIMUM_RTF}" \
    "${CUBE_SUPPORTED_RTF_SAMPLE_COUNT}"
  fast_replay cube_carry "${cube_asset}" cube 2 4 \
    "${CUBE_SUPPORTED_CARRY_TIME_SCALES}" "${cube_payload_sha}" 0.012 0.012
  H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook detach
  set_physics_profile "${PHYSICS_MAX_STEP_SIZE}" \
    RIGHT_MIDDLE_CUBE_RETREAT_PROFILE "${cube_retreat_minimum_rtf}" 1
  fast_replay cube_release "${cube_asset}" cube 4 5 \
    "${cube_release_scale}" "${cube_payload_sha}"
  settled_cube_release_gate RIGHT_MIDDLE_CUBE_RELEASED_ON_RACK
  deadline_gate RIGHT_MIDDLE_CUBE_RELEASE_SETTLE_DEADLINE
  fast_replay cube_retreat "${cube_asset}" cube 5 6 \
    "${cube_retreat_scale}" "${cube_payload_sha}"
  cube_pose_gate RIGHT_MIDDLE_CUBE_ON_RACK \
    0.610890542898 -0.029527393333 0.995407082351 \
    "${cube_parallel_yaw_rad}" 0.025 \
    "${cube_parallel_orientation_tolerance_rad}"
  run_parallel_gate_group RIGHT_MIDDLE_CUBE_RETREAT_DONE \
    2 left_q60_neutral_gate RIGHT_MIDDLE_LEFT_CUBE_READY \
    2 right_door_hold_gate RIGHT_MIDDLE_RIGHT_DOOR_HOLD \
    3 support_state_gate attached RIGHT_MIDDLE_DOOR_SUPPORT_ATTACHED
  deadline_gate RIGHT_MIDDLE_CUBE_DONE_DEADLINE

  run_right_middle_reverse_sequence
  cube_pose_gate RIGHT_MIDDLE_FINAL_CUBE_RACK \
    0.610890542898 -0.029527393333 0.995407082351 \
    "${cube_parallel_yaw_rad}" 0.015 \
    "${cube_parallel_orientation_tolerance_rad}"
  stop_persistent_runner
  require_only_waiting_right_handle_publishers

  local elapsed_ns invocation_ns elapsed invocation
  elapsed_ns=$((FLOW_END_NS - FLOW_START_NS))
  invocation_ns=$((FLOW_END_NS - FLOW_INVOCATION_START_NS))
  (( elapsed_ns > 0 && elapsed_ns < 90000000000 )) ||
    closed_loop_fail 'right-middle first-publish-to-final interval is not below 90 seconds'
  (( invocation_ns > 0 && invocation_ns < 90000000000 )) ||
    closed_loop_fail 'right-middle invocation upper bound is not below 90 seconds'
  elapsed="$(awk -v ns="${elapsed_ns}" 'BEGIN {printf "%.3f",ns/1e9}')"
  invocation="$(awk -v ns="${invocation_ns}" 'BEGIN {printf "%.3f",ns/1e9}')"
  stop_recorder
  verify_video
  mkdir -m 0700 -- "${right_regrasp_release_path}" ||
    closed_loop_fail 'could not create the one-shot right-regrasp release path'
  log "RIGHT_MIDDLE_ONLY_PHYSICAL=PASS first_publish_to_last_motion=${elapsed}s invocation_to_last_motion=${invocation}s door=0deg handle=90deg latch=unlocked cube=middle_rack support=detached; right hand performed both door directions, left hand handled only the cube, regrasp released=${right_regrasp_release_path}."
}

run_right_middle_finalize_sequence()
{
  [[ "${right_regrasp_release_path}" == /* &&
    ! -e "${right_regrasp_release_path}" ]] ||
    closed_loop_fail 'right-middle-finalize requires an absolute, absent release path'
  [[ "${VIDEO_OUTPUT_OVERRIDE}" == /* &&
    -f "${VIDEO_OUTPUT_OVERRIDE}" && ! -L "${VIDEO_OUTPUT_OVERRIDE}" ]] ||
    closed_loop_fail 'right-middle-finalize requires the existing non-symlink run video'
  VIDEO_OUTPUT="${VIDEO_OUTPUT_OVERRIDE}"
  local crop_x crop_y
  IFS=: read -r VIDEO_EXPECTED_WIDTH VIDEO_EXPECTED_HEIGHT crop_x crop_y \
    <<<"${VIDEO_CROP}"
  [[ "${VIDEO_EXPECTED_WIDTH}" =~ ^[1-9][0-9]*$ &&
    "${VIDEO_EXPECTED_HEIGHT}" =~ ^[1-9][0-9]*$ &&
    "${crop_x}" =~ ^[0-9]+$ && "${crop_y}" =~ ^[0-9]+$ ]] ||
    closed_loop_fail 'right-middle-finalize video crop contract is malformed'
  start_support_state_monitor
  right_middle_entry_gate
  cube_pose_gate RIGHT_MIDDLE_FINALIZE_CUBE_RACK \
    0.610890542898 -0.029527393333 0.995407082351 \
    "${cube_parallel_yaw_rad}" 0.015 \
    "${cube_parallel_orientation_tolerance_rad}"
  require_only_waiting_right_handle_publishers
  verify_video
  mkdir -m 0700 -- "${right_regrasp_release_path}" ||
    closed_loop_fail 'could not create the one-shot right-regrasp release path'
  log "RIGHT_MIDDLE_ONLY_PHYSICAL=PASS recovery-finalized door=0deg handle=90deg latch=unlocked cube=middle_rack support=detached robot=handoff right-fingers=open; right hand performed both door directions, left hand handled only the cube, regrasp released=${right_regrasp_release_path}."
}

run_full_sequence()
{
  set_fast_physics_profile
  fresh_scene_gate
  run_door_support_preload
  start_support_state_monitor
  run_cube_support_preload
  fresh_scene_gate
  set_recording_camera_pose
  start_recorder
  physics_and_rtf_gate RECORDER_LOAD_RTF_GATE
  start_first_publish_monitor
  start_persistent_runner
  FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"

  fast_replay handle_rotate "${handle_asset}" handle 0 5 \
    "${HANDLE_MESSAGE_TIME_SCALES}" "${handle_payload_sha}"
  adopt_first_publish_time
  run_parallel_gate_group HANDLE_DONE \
    6 settled_mechanism_gate HANDLE_DONE_MECHANISM \
      "${DOOR_ZERO_RAD}" "${DOOR_FRESH_TOLERANCE_RAD}" \
      "${HANDLE_NINETY_RAD}" "${HANDLE_ROTATED_TOLERANCE_RAD}" \
    3 require_latch_state unlocked HANDLE_DONE_LATCH \
    2 left_handle_hold_gate HANDLE_DONE_LEFT_HOLD
  deadline_gate HANDLE_DONE_DEADLINE

  fast_replay left_clear "${left_clear_asset}" left_clear 0 2 \
    "${LEFT_CLEAR_MESSAGE_TIME_SCALES}" "${left_clear_payload_sha}"
  left_clear_gate LEFT_CLEAR_FAST_GATE
  fast_replay door_pre_support "${door_open_asset}" door 0 6 \
    "${DOOR_BEFORE_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  right_door_hold_gate DOOR_PRE_SUPPORT_HOLD_GATE
  H1_DOOR_SUPPORT_FAST_RESIDENT=1 run_door_support_hook
  deadline_gate DOOR_SUPPORT_DEADLINE

  fast_replay door_open_arc "${door_open_asset}" door 6 13 \
    "${DOOR_AFTER_SUPPORT_TIME_SCALES}" "${door_open_payload_sha}"
  q60_checkpoint DOOR_OPEN_60
  deadline_gate DOOR_OPEN_DEADLINE

  set_physics_profile "${CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE}" CUBE_CONTACT_PROFILE
  fast_replay cube_contact "${cube_asset}" cube 0 2 \
    "${CUBE_BEFORE_ATTACH_TIME_SCALES}" "${cube_payload_sha}"
  H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook attach
  set_physics_profile "${CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE}" \
    CUBE_SUPPORTED_PROFILE "${CUBE_SUPPORTED_MINIMUM_RTF}" \
    "${CUBE_SUPPORTED_RTF_SAMPLE_COUNT}"
  fast_replay cube_carry "${cube_asset}" cube 2 4 \
    "${CUBE_SUPPORTED_CARRY_TIME_SCALES}" "${cube_payload_sha}" 0.012 0.012
  H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook detach
  set_physics_profile "${PHYSICS_MAX_STEP_SIZE}" CUBE_RETREAT_PROFILE \
    "${cube_retreat_minimum_rtf}" 1
  fast_replay cube_release "${cube_asset}" cube 4 5 \
    "${cube_release_scale}" "${cube_payload_sha}"
  settled_cube_release_gate CUBE_RELEASED_ON_MIDDLE_RACK
  deadline_gate CUBE_RELEASE_SETTLE_DEADLINE
  fast_replay cube_retreat "${cube_asset}" cube 5 6 \
    "${cube_retreat_scale}" "${cube_payload_sha}"
  cube_pose_gate CUBE_ON_MIDDLE_RACK \
    0.610890542898 -0.029527393333 0.995407082351 \
    "${cube_parallel_yaw_rad}" 0.025 \
    "${cube_parallel_orientation_tolerance_rad}"
  deadline_gate CUBE_PLACED_DEADLINE

  run_parallel_gate_group CUBE_RETREAT_DONE \
    2 left_q60_neutral_gate CUBE_RETREAT_LEFT_NEUTRAL \
    2 right_door_hold_gate CUBE_RETREAT_RIGHT_HOLD \
    3 support_state_gate attached CUBE_RETREAT_SUPPORT
  deadline_gate CUBE_RETREAT_DONE_DEADLINE

  run_reverse_sequence

  run_parallel_gate_group FINAL_PHYSICAL \
    6 settled_mechanism_gate FINAL_Q0_LOCKED \
      "${DOOR_ZERO_RAD}" "${DOOR_CLOSED_TOLERANCE_RAD}" \
      "${HANDLE_ZERO_RAD}" 0.087266462599716 \
    3 require_latch_state locked FINAL_LATCH \
    3 support_state_gate detached FINAL_SUPPORT \
    2 neutral_arms_and_open_hands_gate FINAL_ROBOT_NEUTRAL \
    2 cube_stability_gate FINAL_CUBE_STABILITY
  cube_pose_gate FINAL_CUBE_RACK \
    0.610890542898 -0.029527393333 0.995407082351 \
    "${cube_parallel_yaw_rad}" 0.015 \
    "${cube_parallel_orientation_tolerance_rad}"
  stop_persistent_runner
  require_no_robot_trajectory_publishers
  local validation_end_ns validation_seconds
  validation_end_ns="$(monotonic_now_ns)"
  validation_seconds="$(awk -v start="${FLOW_END_NS}" \
    -v end="${validation_end_ns}" 'BEGIN {printf "%.3f",(end-start)/1e9}')"

  local elapsed_ns invocation_ns elapsed invocation
  elapsed_ns=$((FLOW_END_NS - FLOW_START_NS))
  invocation_ns=$((FLOW_END_NS - FLOW_INVOCATION_START_NS))
  (( elapsed_ns > 0 && elapsed_ns < 90000000000 )) ||
    closed_loop_fail "complete first-publish-to-final interval is not below 90 seconds"
  (( invocation_ns > 0 && invocation_ns < 90000000000 )) ||
    closed_loop_fail "complete invocation upper bound is not below 90 seconds"
  elapsed="$(awk -v ns="${elapsed_ns}" 'BEGIN {printf "%.3f",ns/1e9}')"
  invocation="$(awk -v ns="${invocation_ns}" 'BEGIN {printf "%.3f",ns/1e9}')"
  printf '%s\n' "${elapsed}" >"${RUN_DIR}/first_publish_to_final_seconds.txt"
  printf '%s\n' "${invocation}" >"${RUN_DIR}/invocation_to_final_seconds.txt"
  printf '%s\n' "${validation_seconds}" >"${RUN_DIR}/post_motion_validation_seconds.txt"
  stop_recorder
  verify_video
  log "CLOSED_LOOP_90_8254_PHYSICAL=PASS first_publish_to_last_motion=${elapsed}s invocation_to_last_motion=${invocation}s post_motion_validation=${validation_seconds}s door=0deg handle=0deg latch=locked cube=middle_rack_flat support=detached publishers=0 video=${VIDEO_OUTPUT:-disabled}"
}

cleanup_8254()
{
  stop_first_publish_monitor
  if [[ -n "${PERSISTENT_RUNNER_PID}" ]] &&
    kill -0 "${PERSISTENT_RUNNER_PID}" 2>/dev/null
  then
    kill -TERM "${PERSISTENT_RUNNER_PID}" 2>/dev/null || true
    wait "${PERSISTENT_RUNNER_PID}" 2>/dev/null || true
    PERSISTENT_RUNNER_PID=""
  fi
  stop_snapshot_monitor
  stop_gazebo_state_monitors
  stop_recorder
}

interrupt_8254()
{
  log 'Interrupted: diagnostic processes stop; no reset, support detach, gripper release, or recovery trajectory is sent.'
  exit 130
}

require_exact_waiting_right_handle_demo()
{
  local demos count
  demos="$(pgrep -af \
    '(/h1_2_moveit_demo/)(incubator_door_open_demo|bimanual_air_handover_demo)([[:space:]]|$)' || true)"
  count="$(awk 'NF {count++} END {print count+0}' <<<"${demos}")"
  [[ "${count}" == 1 ]] ||
    closed_loop_fail "right-middle-run requires exactly one waiting handle demo, found ${count}"
  grep -Fq 'right_handle_wait_regrasp_after_clear:=true' <<<"${demos}" ||
    closed_loop_fail 'the resident handle demo is not in wait/regrasp mode'
  grep -Fq "right_handle_regrasp_release_path:=${right_regrasp_release_path}" \
    <<<"${demos}" ||
    closed_loop_fail 'the resident handle demo is bound to a different release path'
  if pgrep -af '[h]1_joint_trajectory_asset.py[[:space:]]+replay([[:space:]]|$)' \
    >/dev/null
  then
    closed_loop_fail 'another sealed-trajectory replay is active'
  fi
}

require_only_waiting_right_handle_publishers()
{
  require_exact_waiting_right_handle_demo
  local topic info publishers subscriber_count unexpected=""
  for topic in "${ROBOT_TRAJECTORY_TOPICS[@]}"; do
    info="$(timeout 5 ros2 topic info -v "${topic}" 2>/dev/null)" ||
      closed_loop_fail "right-middle ROS graph query failed: ${topic}"
    subscriber_count="$(sed -n \
      's/^Subscription count: \([0-9][0-9]*\)$/\1/p' <<<"${info}")"
    [[ "${subscriber_count}" =~ ^[1-9][0-9]*$ ]] ||
      closed_loop_fail "right-middle controller subscriber is absent: ${topic}"
    publishers="$(awk '
      /^Publisher count:/ {in_publishers=1; next}
      /^Subscription count:/ {in_publishers=0}
      in_publishers && /^Node name:/ {print $3}
    ' <<<"${info}")"
    if awk 'NF && $0 != "incubator_door_open_demo" {exit 1}' \
      <<<"${publishers}"
    then
      :
    else
      unexpected="${unexpected}${topic}: non-waiting publisher; "
    fi
  done
  [[ -z "${unexpected}" ]] ||
    closed_loop_fail "right-middle publisher ownership gate failed: ${unexpected}"
  if pgrep -af '[h]1_joint_trajectory_asset.py[[:space:]]+replay([[:space:]]|$)' \
    >/dev/null
  then
    closed_loop_fail 'another sealed-trajectory replay is active'
  fi
}

main_8254()
{
  [[ -d "${closed_loop_workspace}" && -f "${closed_loop_workspace}/AGENTS.md" ]] ||
    closed_loop_fail "workspace is invalid"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "${closed_loop_workspace}/install/setup.bash"
  set -u
  mkdir -p "${RUNTIME_ROOT}"
  exec 9>"${LOCK_FILE}"
  flock -n 9 || closed_loop_fail "another 8254 wrapper owns ${LOCK_FILE}"
  RUN_DIR="${RUNTIME_ROOT}/${closed_loop_mode}_$(date +%Y%m%d_%H%M%S_%N)_$$_${RANDOM}"
  mkdir -m 0700 -- "${RUN_DIR}"
  start_snapshot_monitor
  start_gazebo_state_monitors
  if [[ "${closed_loop_mode}" == right-middle-run ||
    "${closed_loop_mode}" == right-middle-finalize ]]; then
    require_exact_waiting_right_handle_demo
    # The sourced health gate invokes this callback internally.  In the
    # right-middle process only, preserve all controller-subscriber checks but
    # admit the already-authenticated dormant MoveGroup endpoints owned by the
    # single waiting demo.  Every other mode retains the sourced zero-publisher
    # implementation.
    require_no_robot_trajectory_publishers()
    {
      require_only_waiting_right_handle_publishers
    }
  else
    require_no_other_demo
  fi
  basic_health_gate
  if [[ "${closed_loop_mode}" == right-middle-run ||
    "${closed_loop_mode}" == right-middle-finalize ]]; then
    require_only_waiting_right_handle_publishers
  else
    require_no_robot_trajectory_publishers
  fi
  if [[ "${closed_loop_mode}" == right-middle-finalize ]]; then
    run_right_middle_finalize_sequence
    return 0
  fi
  preflight_all_assets

  if [[ "${closed_loop_mode}" == preflight ]]; then
    fresh_scene_gate
    log 'CLOSED_LOOP_90_8254_PREFLIGHT_ONLY=PASS; no controller trajectory, support operation, physics mutation, reset, or direct-door command was sent.'
    return 0
  fi
  if [[ "${closed_loop_mode}" == prepare-q60 ]]; then
    run_prepare_q60_sequence
    return 0
  fi
  if [[ "${closed_loop_mode}" == reverse-run ]]; then
    start_support_state_monitor
    q60_checkpoint REVERSE_ENTRY_Q60
    left_q60_neutral_gate REVERSE_ENTRY_LEFT_NEUTRAL
    cube_pose_gate REVERSE_ENTRY_CUBE \
      0.610890542898 -0.029527393333 0.995407082351 \
      "${cube_parallel_yaw_rad}" 0.040 \
      "${cube_parallel_orientation_tolerance_rad}"
    set_recording_camera_pose
    start_recorder
    start_first_publish_monitor
    start_persistent_runner
    FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"
    run_reverse_sequence
    stop_persistent_runner
    require_no_robot_trajectory_publishers
    FLOW_END_NS="$(monotonic_now_ns)"
    stop_recorder
    verify_video
    log 'CLOSED_LOOP_90_8254_REVERSE_RECOVERY=PASS door=0 handle=0 support=detached cube=middle_rack.'
    return 0
  fi
  if [[ "${closed_loop_mode}" == right-middle-run ]]; then
    run_right_middle_sequence
    return 0
  fi
  run_full_sequence
}

trap cleanup_8254 EXIT
trap interrupt_8254 INT TERM HUP
main_8254
