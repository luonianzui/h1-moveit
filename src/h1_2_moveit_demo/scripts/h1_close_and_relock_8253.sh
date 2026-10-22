#!/usr/bin/env bash
# Append the admitted reverse sequence to the successful 8251 end state:
# left cube-retreat -> neutral, right hand closes the original door, right
# support detaches at q0, right hand retreats, left hand re-grasps and rotates
# the original handle from 90 to 0, then releases and retreats.  Failure never
# triggers reset, direct incubator motion, an unplanned release, or cleanup
# detach.
set -Eeuo pipefail

readonly mode="${1:-}"
shift || true
readonly workspace="${H1_CLOSE_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly trajectory_tool="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_joint_trajectory_asset.py"
readonly snapshot_tool="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_ros_snapshot_8251.py"
readonly detach_helper="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_door_support_detach_8253.sh"
readonly demo="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/incubator_door_open_demo"
readonly default_asset_dir="${workspace}/runtime_assets/close_and_relock_8253"
readonly door_asset="${1:-${default_asset_dir}/door_close.trajectory.json}"
readonly left_asset="${2:-${default_asset_dir}/left_regrasp.trajectory.json}"
readonly handle_asset="${3:-${default_asset_dir}/handle_relock.trajectory.json}"

readonly door_payload_sha="5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038"
readonly left_payload_sha="5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d"
readonly handle_payload_sha="b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3"
readonly door_file_sha="3cbb2decfe3128e6a7ca95238085f4285b06a79486c40733db7d426d0a3e5afb"
readonly left_file_sha="8d4443c50e9d5d18bc4b1df0cef6a5af132fd4b577c4dada964f3350ec6ef443"
readonly handle_file_sha="3d88b71fc9c748b6669831467b7cbd6b6f21f67a5026ea88da99e4bc93fa07e9"
readonly door_close_scales="0:0.1,1:0.1,2:0.1,3:0.1,4:0.1,5:0.15,6:0.12"
readonly door_retreat_scales="7:0.25,8:0.35,9:0.15,10:0.3,11:0.1,12:0.3"
readonly left_regrasp_scales="0:0.25,1:0.25"
# The first reverse-handle physical run deliberately keeps the certified
# 19.152 s q90->q0 arm message unscaled.  Only release/far-retreat messages
# reuse already admitted forward retiming.
readonly handle_relock_scales="0:1,1:0.25,2:1,3:1,4:0.25"
readonly direct_door_topic="/h1_2/incubator/simulation_direct_joint_trajectory"
readonly support_state_topic="/h1_2/right_door_support/state"
readonly run_root="${H1_CLOSE_RUN_ROOT:-/home/liu/h1_moveit/runtime_logs/close_and_relock_8253}"
readonly run_dir="${run_root}/run_$(date +%Y%m%d_%H%M%S)_$$_${RANDOM}"
motion_started=0

fail()
{
  if (( motion_started )); then
    printf 'h1_close_and_relock_8253: STOP-AND-HOLD: %s No reset, direct-door command, cleanup detach, or gripper cleanup was sent.\n' "$*" >&2
  else
    printf 'h1_close_and_relock_8253: REJECTED WITHOUT MOTION: %s\n' "$*" >&2
  fi
  exit 1
}

usage()
{
  printf 'usage: h1_close_and_relock_8253.sh preflight|run [door_close.json left_regrasp.json handle_relock.json]\n' >&2
}

[[ "${mode}" == preflight || "${mode}" == run ]] || { usage; exit 2; }
(( $# <= 3 )) || { usage; exit 2; }
mkdir -p -- "${run_dir}"
printf 'mode=%s run_dir=%s\n' "${mode}" "${run_dir}" | tee "${run_dir}/start.log"

for command in ros2 gz jq timeout awk grep pgrep sha256sum stat tee; do
  command -v "${command}" >/dev/null || fail "${command} is unavailable"
done
for executable in "${trajectory_tool}" "${snapshot_tool}" "${detach_helper}" "${demo}"; do
  [[ -x "${executable}" ]] || fail "required executable is unavailable: ${executable}"
done

inspect_asset()
{
  local label="$1" asset="$2" stage="$3" count="$4" payload_sha="$5" file_sha="$6"
  [[ "${asset}" == /* && -f "${asset}" && ! -L "${asset}" ]] ||
    fail "${label} asset must be an absolute non-symlink file"
  [[ "$(stat -c '%a %u %h' -- "${asset}")" == "400 ${EUID} 1" ]] ||
    fail "${label} asset must be owned mode-0400 with one link"
  [[ "$(sha256sum -- "${asset}" | awk '{print $1}')" == "${file_sha}" ]] ||
    fail "${label} asset file SHA-256 differs from the wrapper binding"
  local output
  output="$("${trajectory_tool}" inspect "${asset}" 2>&1)" ||
    fail "${label} asset inspection failed"
  grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${output}" ||
    fail "${label} asset schema mismatch"
  grep -Fxq "stage=${stage}" <<<"${output}" || fail "${label} stage mismatch"
  grep -Fxq "message_count=${count}" <<<"${output}" ||
    fail "${label} message count mismatch"
  grep -Fxq "sha256=${payload_sha}" <<<"${output}" ||
    fail "${label} payload SHA-256 mismatch"
  printf '%s\n' "${output}" >"${run_dir}/${label}.inspect.log"
}

retime_check()
{
  local label="$1" asset="$2" stage="$3" start="$4" end="$5" scales="$6"
  local command=("${trajectory_tool}" retime-check "${asset}"
    --expected-stage "${stage}" --message-time-scales "${scales}")
  if [[ -n "${start}" ]]; then
    command+=(--start-index "${start}" --end-index "${end}")
  fi
  "${command[@]}" >"${run_dir}/${label}.retime.log" 2>&1 ||
    fail "${label} retime check failed"
  grep -Fq 'RETIME_CHECK=PASS' "${run_dir}/${label}.retime.log" ||
    fail "${label} retime PASS marker is missing"
}

require_controllers()
{
  local rows controller
  rows="$(ros2 control list_controllers 2>&1)" || fail "controller query failed"
  for controller in right_arm_controller left_arm_controller \
    joint_state_broadcaster left_gripper_controller right_gripper_controller
  do
    awk -v wanted="${controller}" '$1==wanted && $NF=="active" {found++}
      END {exit !(found==1)}' <<<"${rows}" ||
      fail "controller ${controller} is not uniquely active"
  done
}

require_no_trajectory_publishers()
{
  local topic info
  for topic in /right_arm_controller/joint_trajectory \
    /left_arm_controller/joint_trajectory \
    /right_gripper_controller/joint_trajectory \
    /left_gripper_controller/joint_trajectory
  do
    info="$(ros2 topic info -v "${topic}" 2>&1)" ||
      fail "could not inspect ${topic}"
    grep -Fxq 'Publisher count: 0' <<<"${info}" ||
      fail "unexpected publisher exists on ${topic}"
    grep -Fxq 'Subscription count: 1' <<<"${info}" ||
      fail "${topic} does not have exactly one controller subscriber"
  done
}

support_state()
{
  timeout 6 gz topic -e -t "${support_state_topic}" -n 1 2>/dev/null |
    awk -F'"' '/^[[:space:]]*data:/ {print $2; found++}
      END {if(found!=1) exit 1}'
}

latch_state()
{
  timeout 5 gz topic -e -t /incubator/latch/state -n 1 2>/dev/null |
    awk -F'"' '/^[[:space:]]*data:/ {print $2; found++}
      END {if(found!=1) exit 1}'
}

mechanism_gate()
{
  local label="$1" expected="$2" rows latch
  rows="$(timeout 5 "${snapshot_tool}" incubator --timeout 3)" ||
    fail "${label}: mechanism snapshot failed"
  latch="$(latch_state)" || fail "${label}: latch snapshot failed"
  awk -v mode="${expected}" -v latch="${latch}" '
    $1=="door_hinge_joint" {door=$2; dv=$3; dc++}
    $1=="handle_latch_joint" {handle=$2; hv=$3; hc++}
    END {
      pi=3.141592653589793
      if(dc!=1 || hc!=1) exit 1
      if(dv<0)dv=-dv; if(hv<0)hv=-hv
      if(dv>0.002 || hv>0.002) exit 1
      if(mode=="q60_unlocked") {
        de=door-pi/3; if(de<0)de=-de; he=handle-pi/2; if(he<0)he=-he
        exit !(de<=0.003490658503989 && he<=0.052359877559830 && latch=="unlocked")
      }
      if(mode=="q0_unlocked") {
        if(door<0)door=-door; he=handle-pi/2; if(he<0)he=-he
        exit !(door<=0.003490658503989 && he<=0.052359877559830 && latch=="unlocked")
      }
      if(mode=="q0_locked") {
        if(door<0)door=-door; if(handle<0)handle=-handle
        exit !(door<=0.003490658503989 && handle<=0.087266462599716 && latch=="locked")
      }
      exit 1
    }
  ' <<<"${rows}" || fail "${label}: expected mechanism state ${expected} is absent"
  printf '%s\n%s\n' "${rows}" "latch ${latch}" >"${run_dir}/${label}.mechanism.log"
}

cube_rack_gate()
{
  local label="$1" cube
  cube="$(timeout 6 gz topic -e -t /world/h1_world/dynamic_pose/info -n 1 \
    --json-output 2>/dev/null | jq -cer '
      [.pose[]|select(.name=="green_cube")]|if length==1 then .[0] else error("cube") end
    ')" || fail "${label}: fresh green_cube pose is unavailable"
  jq -e '
    .position.x>=0.58 and .position.x<=0.70 and
    .position.y>=-0.08 and .position.y<=0.08 and
    .position.z>=0.95 and .position.z<=1.05 and
    ([.position.x,.position.y,.position.z,.orientation.x,.orientation.y,
      .orientation.z,.orientation.w]|all(isfinite))
  ' <<<"${cube}" >/dev/null || fail "${label}: cube is outside the middle-rack gate"
  printf '%s\n' "${cube}" >"${run_dir}/${label}.cube.json"
}

health_gate()
{
  pgrep -af '[g]z sim gui' >/dev/null || fail "Gazebo GUI is not running"
  pgrep -af '[m]ove_group([^_]|$)' >/dev/null || fail "move_group is not running"
  pgrep -af '[r]obot_state_publisher' >/dev/null ||
    fail "robot_state_publisher is not running"
  require_controllers
  require_no_trajectory_publishers
  if ros2 topic list | grep -Fxq "${direct_door_topic}"; then
    fail "direct incubator controller topic exists"
  fi
  [[ "$(support_state)" == attached ]] ||
    fail "right door support is not fresh attached at q60"
  mechanism_gate initial q60_unlocked
  cube_rack_gate initial
}

run_left_neutral_bridge()
{
  local preflight="$1" label="$2"
  "${demo}" --ros-args \
    -p use_sim_time:=true \
    -p preflight_only:="${preflight}" \
    -p door_handoff_left_clear_recovery:=true \
    -p front_handle_rz_test:=true \
    -p door_handoff_only_candidate:=3 \
    -p door_handoff_simulation_allow_right_elbow_torso_collision:=true \
    -p motion_scaling:=0.15 \
    -p planning_time_seconds:=30.0 \
    2>&1 | tee "${run_dir}/${label}.log"
  local status=${PIPESTATUS[0]}
  (( status == 0 )) || fail "${label} exited ${status}"
  if [[ "${preflight}" == true ]]; then
    grep -Fq 'DOOR_HANDOFF_LEFT_CLEAR_RECOVERY_PREFLIGHT_ONLY_SUCCESS' \
      "${run_dir}/${label}.log" || fail "${label} preflight PASS marker is missing"
  else
    grep -Fq 'DOOR_HANDOFF_LEFT_CLEAR_RECOVERY_SUCCESS' \
      "${run_dir}/${label}.log" || fail "${label} physical PASS marker is missing"
  fi
}

run_replay()
{
  local label="$1" asset="$2" stage="$3" start="$4" end="$5" scales="$6"
  local command=("${trajectory_tool}" replay "${asset}" --execute
    --expected-stage "${stage}" --message-time-scales "${scales}")
  if [[ -n "${start}" ]]; then
    command+=(--start-index "${start}" --end-index "${end}")
  fi
  motion_started=1
  "${command[@]}" 2>&1 | tee "${run_dir}/${label}.log"
  local status=${PIPESTATUS[0]}
  (( status == 0 )) || fail "${label} replay exited ${status}"
  grep -Fq 'REPLAY_ASSET=PASS' "${run_dir}/${label}.log" ||
    fail "${label} replay PASS marker is missing"
  require_no_trajectory_publishers
}

inspect_asset door "${door_asset}" door 13 "${door_payload_sha}" "${door_file_sha}"
inspect_asset left_regrasp "${left_asset}" left_clear 2 "${left_payload_sha}" "${left_file_sha}"
inspect_asset handle_relock "${handle_asset}" handle 5 "${handle_payload_sha}" "${handle_file_sha}"
retime_check door_close "${door_asset}" door 0 7 "${door_close_scales}"
retime_check door_retreat "${door_asset}" door 7 13 "${door_retreat_scales}"
retime_check left_regrasp "${left_asset}" left_clear '' '' "${left_regrasp_scales}"
retime_check handle_relock "${handle_asset}" handle '' '' "${handle_relock_scales}"
health_gate

if [[ "${mode}" == preflight ]]; then
  run_left_neutral_bridge true left_neutral_preflight
  require_no_trajectory_publishers
  mechanism_gate preflight_final q60_unlocked
  [[ "$(support_state)" == attached ]] || fail "preflight changed support state"
  printf 'CLOSE_RELOCK_8253_PREFLIGHT_ONLY=PASS no_controller_trajectory=1 support=attached door=60 handle=90\n' |
    tee "${run_dir}/result.log"
  exit 0
fi

motion_started=1
run_left_neutral_bridge false left_neutral_physical
mechanism_gate after_left_neutral q60_unlocked
[[ "$(support_state)" == attached ]] || fail "left neutral bridge changed support state"
cube_rack_gate after_left_neutral

run_replay door_close "${door_asset}" door 0 7 "${door_close_scales}"
mechanism_gate closed_supported q0_unlocked
[[ "$(support_state)" == attached ]] || fail "support detached before the q0 gate"

export H1_CLOSE_WORKSPACE="${workspace}"
export H1_CLOSE_DOOR_ASSET="${door_asset}"
export H1_CLOSE_DOOR_ASSET_SHA256="${door_payload_sha}"
export H1_CLOSE_DOOR_DETACH_AFTER_INDEX=7
"${detach_helper}" detach 2>&1 | tee "${run_dir}/door_support_detach.log"
detach_status=${PIPESTATUS[0]}
(( detach_status == 0 )) || fail "door support detach helper exited ${detach_status}"
grep -Fq 'DOOR_SUPPORT_DETACH_8253=PASS' "${run_dir}/door_support_detach.log" ||
  fail "door support detach PASS marker is missing"
[[ "$(support_state)" == detached ]] || fail "support is not fresh detached"
mechanism_gate after_detach q0_unlocked

run_replay right_release "${door_asset}" door 7 8 "7:0.25"
mechanism_gate after_right_release q0_unlocked
[[ "$(support_state)" == detached ]] || fail "support reattached after right release"

run_replay right_retreat "${door_asset}" door 8 13 \
  "8:0.35,9:0.15,10:0.3,11:0.1,12:0.3"
mechanism_gate after_right_retreat q0_unlocked
[[ "$(support_state)" == detached ]] || fail "support unexpectedly reattached"
cube_rack_gate after_right_retreat

run_replay left_regrasp "${left_asset}" left_clear '' '' "${left_regrasp_scales}"
mechanism_gate left_clamped_q90 q0_unlocked

run_replay handle_relock "${handle_asset}" handle '' '' "${handle_relock_scales}"
mechanism_gate final q0_locked
[[ "$(support_state)" == detached ]] || fail "final support state is not detached"
cube_rack_gate final
require_no_trajectory_publishers

printf 'CLOSE_RELOCK_8253_PHYSICAL=PASS door=0deg handle=0deg latch=locked right_support=detached cube=middle_rack both_hands=released_neutral\n' |
  tee "${run_dir}/result.log"
