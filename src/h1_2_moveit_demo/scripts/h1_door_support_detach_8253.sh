#!/usr/bin/env bash
# Adopt the already-attached 8251 right-door support only at the exact q0
# close checkpoint, then detach it while the physical right fingers still
# hold the unchanged SolidWorks handle.  This helper never commands a robot,
# resets Gazebo, changes a model pose, or opens either gripper.
set -Eeuo pipefail

readonly workspace="${H1_CLOSE_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly asset_tool="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_joint_trajectory_asset.py"
readonly close_asset="${H1_CLOSE_DOOR_ASSET:-}"
readonly close_asset_sha256="${H1_CLOSE_DOOR_ASSET_SHA256:-}"
readonly detach_after_index="${H1_CLOSE_DOOR_DETACH_AFTER_INDEX:-7}"
readonly ros_snapshot_tool="${H1_CLOSE_ROS_SNAPSHOT_TOOL:-}"
readonly skip_pre_transport="${H1_CLOSE_SKIP_PRE_TRANSPORT:-0}"
readonly skip_post_transport="${H1_CLOSE_SKIP_POST_TRANSPORT:-0}"
readonly state_topic="/h1_2/right_door_support/state"
readonly attach_topic="/h1_2/right_door_support/attach"
readonly detach_topic="/h1_2/right_door_support/detach"

fail()
{
  printf 'h1_door_support_detach_8253: ERROR: %s\n' "$*" >&2
  exit 1
}

[[ "${1:-}" == "detach" && $# == 1 ]] || {
  printf 'usage: h1_door_support_detach_8253.sh detach\n' >&2
  exit 2
}
[[ -x "${asset_tool}" ]] || fail "installed trajectory tool is unavailable"
[[ "${close_asset}" == /* && -f "${close_asset}" && ! -L "${close_asset}" ]] ||
  fail "H1_CLOSE_DOOR_ASSET must name an absolute non-symlink file"
[[ "${close_asset_sha256}" =~ ^[0-9a-f]{64}$ ]] ||
  fail "H1_CLOSE_DOOR_ASSET_SHA256 is malformed"
[[ "${detach_after_index}" =~ ^[1-9][0-9]*$ ]] ||
  fail "H1_CLOSE_DOOR_DETACH_AFTER_INDEX is malformed"
[[ "${skip_post_transport}" == 0 || "${skip_post_transport}" == 1 ]] ||
  fail "H1_CLOSE_SKIP_POST_TRANSPORT must be 0 or 1"
[[ "${skip_pre_transport}" == 0 || "${skip_pre_transport}" == 1 ]] ||
  fail "H1_CLOSE_SKIP_PRE_TRANSPORT must be 0 or 1"
for command in gz jq ros2 timeout awk grep stat mktemp rm rmdir sed; do
  command -v "${command}" >/dev/null || fail "${command} is unavailable"
done
if [[ -n "${ros_snapshot_tool}" ]]; then
  [[ "${ros_snapshot_tool}" == /* && -f "${ros_snapshot_tool}" &&
    -x "${ros_snapshot_tool}" ]] ||
    fail "H1_CLOSE_ROS_SNAPSHOT_TOOL must name an absolute executable"
fi
[[ "$(stat -c '%a %u %h' -- "${close_asset}")" == "400 ${EUID} 1" ]] ||
  fail "close asset must be owned mode-0400 with one link"

inspection="$("${asset_tool}" inspect "${close_asset}" 2>&1)" ||
  fail "close asset inspection failed"
grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${inspection}" ||
  fail "close asset schema mismatch"
grep -Fxq 'stage=door' <<<"${inspection}" || fail "close asset stage mismatch"
grep -Fxq "sha256=${close_asset_sha256}" <<<"${inspection}" ||
  fail "close asset payload SHA-256 differs from wrapper binding"
message_count="$(awk -F= '$1=="message_count" {print $2; found++}
  END {if(found!=1) exit 1}' <<<"${inspection}")" ||
  fail "close asset message count is unavailable"
(( detach_after_index == 7 && message_count == 13 )) ||
  fail "only the admitted seven-message supported close split is allowed"

jq -e --argjson split "${detach_after_index}" '
  .payload.messages as $m |
  ($m|length)==13 and
  all($m[0:$split][];
    .topic=="/right_arm_controller/joint_trajectory") and
  $m[$split].topic=="/right_gripper_controller/joint_trajectory" and
  all($m[($split+1):12][];
    .topic=="/right_arm_controller/joint_trajectory") and
  $m[12].topic=="/right_gripper_controller/joint_trajectory"
' "${close_asset}" >/dev/null || fail "close asset support split topology is wrong"

require_transport_contract()
{
  local state_info attach_info detach_info scratch
  local state_pid attach_pid detach_pid discovery_failed=0
  scratch="$(mktemp -d /tmp/h1_door_detach_transport.XXXXXX)" ||
    fail "cannot allocate bounded transport scratch directory"

  timeout 5 gz topic -i -t "${state_topic}" >"${scratch}/state" 2>&1 &
  state_pid=$!
  timeout 5 gz topic -i -t "${attach_topic}" >"${scratch}/attach" 2>&1 &
  attach_pid=$!
  timeout 5 gz topic -i -t "${detach_topic}" >"${scratch}/detach" 2>&1 &
  detach_pid=$!
  wait "${state_pid}" || discovery_failed=1
  wait "${attach_pid}" || discovery_failed=1
  wait "${detach_pid}" || discovery_failed=1
  state_info="$(<"${scratch}/state")"
  attach_info="$(<"${scratch}/attach")"
  detach_info="$(<"${scratch}/detach")"
  rm -f -- "${scratch}/state" "${scratch}/attach" "${scratch}/detach"
  rmdir -- "${scratch}"
  (( discovery_failed == 0 )) || fail "support topic discovery failed"
  [[ "$(grep -Fc 'gz.msgs.StringMsg' <<<"${state_info}")" == 1 ]] ||
    fail "support state must have exactly one StringMsg publisher"
  [[ "$(grep -Fc 'gz.msgs.Empty' <<<"${attach_info}")" == 1 ]] ||
    fail "support attach must have exactly one Empty subscriber"
  [[ "$(grep -Fc 'gz.msgs.Empty' <<<"${detach_info}")" == 1 ]] ||
    fail "support detach must have exactly one Empty subscriber"
}

support_state()
{
  timeout 6 gz topic -e -t "${state_topic}" -n 1 2>/dev/null |
    awk -F'"' '/^[[:space:]]*data:/ {print $2; found++}
      END {if(found!=1) exit 1}'
}

robot_rows()
{
  local attempt rows sample
  if [[ -n "${ros_snapshot_tool}" ]]; then
    for attempt in 1 2 3; do
      if rows="$(timeout 5 "${ros_snapshot_tool}" robot --timeout 3)"; then
        printf '%s\n' "${rows}"
        return 0
      fi
    done
    return 1
  fi
  sample="$(timeout 8 ros2 topic echo --once /joint_states 2>/dev/null)" || return 1
  awk '
    /^name:/ {section="name"; next}
    /^position:/ {section="position"; next}
    /^velocity:/ {section="velocity"; next}
    /^effort:/ {section="effort"; next}
    section=="name" && /^- / {name[++names]=$2; next}
    section=="position" && /^- / {position[++positions]=$2; next}
    END {
      if(names==0 || positions!=names) exit 1
      for(i=1;i<=names;++i) {gsub(/"/,"",name[i]); print name[i],position[i]}
    }
  ' <<<"${sample}"
}

mechanism_rows()
{
  local attempt rows sample
  if [[ -n "${ros_snapshot_tool}" ]]; then
    for attempt in 1 2 3; do
      if rows="$(timeout 5 "${ros_snapshot_tool}" incubator --timeout 3)"; then
        printf '%s\n' "${rows}"
        return 0
      fi
    done
    return 1
  fi
  sample="$(timeout 8 ros2 topic echo --once /incubator/joint_states 2>/dev/null)" ||
    return 1
  awk '
    /^name:/ {section="name"; next}
    /^position:/ {section="position"; next}
    /^velocity:/ {section="velocity"; next}
    /^effort:/ {section="effort"; next}
    section=="name" && /^- / {name[++n]=$2; next}
    section=="position" && /^- / {position[++p]=$2; next}
    section=="velocity" && /^- / {velocity[++v]=$2; next}
    END {
      if(n==0 || p!=n || v!=n) exit 1
      for(i=1;i<=n;i++) {
        gsub(/"/,"",name[i])
        print name[i],position[i],velocity[i]
      }
    }
  ' <<<"${sample}"
}

require_robot_close_endpoint()
{
  local rows endpoint joint expected actual count=0
  rows="$(robot_rows)" || fail "$1: robot telemetry is malformed"
  for topic in /right_arm_controller/joint_trajectory \
    /right_gripper_controller/joint_trajectory
  do
    endpoint="$(jq -er --arg topic "${topic}" --argjson split "${detach_after_index}" '
      (if $topic=="/right_arm_controller/joint_trajectory" then
        [.payload.messages[0:$split][]|select(.topic==$topic)]|last |
        {joint_names:.joint_names, positions:.points[-1].positions}
      else
        .payload.messages[$split] |
        {joint_names:.joint_names, positions:.points[0].positions}
      end) |
      .joint_names as $n | .positions as $p |
      range(0;($n|length)) as $i | [$n[$i],$p[$i]]|@tsv
    ' "${close_asset}")" || fail "$1: sealed ${topic} endpoint is absent"
    while IFS=$'\t' read -r joint expected; do
      actual="$(awk -v joint="${joint}" '$1==joint {print $2; found++}
        END {if(found!=1) exit 1}' <<<"${rows}")" ||
        fail "$1: live ${joint} is missing"
      awk -v a="${actual}" -v e="${expected}" 'BEGIN {
        d=a-e; if(d<0)d=-d; exit !(d<=0.003)
      }' || fail "$1: ${joint} differs from the q0 sealed endpoint"
      ((count+=1))
    done <<<"${endpoint}"
  done
  (( count == 9 )) || fail "$1: endpoint cardinality is not 7+2"
}

mechanism_rows_gate()
{
  local label="$1" rows="$2"
  awk '
    NF!=3 {bad=1; next}
    $1=="door_hinge_joint" {door=$2; dv=$3; dc++}
    $1=="handle_latch_joint" {handle=$2; hv=$3; hc++}
    END {
      pi=3.141592653589793
      if(bad || dc!=1 || hc!=1) exit 1
      if(door<0)door=-door; h=handle-pi/2; if(h<0)h=-h
      if(dv<0)dv=-dv; if(hv<0)hv=-hv
      exit !(door<=0.003490658503989 && h<=0.052359877559830 &&
        dv<=0.002 && hv<=0.002)
    }
  ' <<<"${rows}" || fail "${label}: mechanism is not stationary q0/90"
}

mechanism_gate()
{
  local latch rows
  rows="$(mechanism_rows)" ||
    fail "$1: mechanism telemetry is unavailable"
  mechanism_rows_gate "$1" "${rows}"
  latch="$(timeout 5 gz topic -e -t /incubator/latch/state -n 1 2>/dev/null)" ||
    fail "$1: latch telemetry is unavailable"
  grep -Eq 'data:[[:space:]]*"unlocked"' <<<"${latch}" ||
    fail "$1: latch is not unlocked"
}

mechanism_triplet_gate()
{
  local phase="$1" latch rows sample
  # One retained subscriber collects three distinct latch publications.  The
  # previous implementation started six separate gz CLI processes, which
  # added only discovery latency while checking the same stationary q0/90
  # state.  Three independent ROS mechanism snapshots remain mandatory on
  # each side of detach.
  latch="$(timeout 5 gz topic -e -t /incubator/latch/state -n 3 2>/dev/null)" ||
    fail "${phase}: three latch samples are unavailable"
  awk -F'"' '
    /^[[:space:]]*data:/ {count++; if($2!="unlocked") bad=1}
    END {exit !(count==3 && !bad)}
  ' <<<"${latch}" || fail "${phase}: three latch samples are not all unlocked"
  for sample in 1 2 3; do
    rows="$(mechanism_rows)" ||
      fail "${phase} sample ${sample}: mechanism telemetry is unavailable"
    mechanism_rows_gate "${phase} sample ${sample}" "${rows}"
    (( sample == 3 )) || sleep 0.02
  done
}

parallel_detach_physical_gate()
{
  local phase="$1" expected_support="$2" scratch
  local support_pid robot_pid mechanism_pid status=0
  scratch="$(mktemp -d /tmp/h1_door_detach_gate.XXXXXX)" ||
    fail "${phase}: cannot allocate parallel physical-gate scratch directory"

  (
    [[ "$(support_state)" == "${expected_support}" ]] ||
      fail "${phase}: support is not fresh ${expected_support}"
  ) >"${scratch}/support.log" 2>&1 &
  support_pid=$!
  (
    require_robot_close_endpoint "${phase}"
  ) >"${scratch}/robot.log" 2>&1 &
  robot_pid=$!
  (
    mechanism_triplet_gate "${phase}"
  ) >"${scratch}/mechanism.log" 2>&1 &
  mechanism_pid=$!

  wait "${support_pid}" || status=1
  wait "${robot_pid}" || status=1
  wait "${mechanism_pid}" || status=1
  if (( status != 0 )); then
    sed -n 'p' "${scratch}/support.log" >&2
    sed -n 'p' "${scratch}/robot.log" >&2
    sed -n 'p' "${scratch}/mechanism.log" >&2
    rm -f -- "${scratch}/support.log" "${scratch}/robot.log" \
      "${scratch}/mechanism.log"
    rmdir -- "${scratch}" 2>/dev/null || true
    fail "${phase}: parallel support/endpoint/mechanism gate failed"
  fi
  rm -f -- "${scratch}/support.log" "${scratch}/robot.log" \
    "${scratch}/mechanism.log"
  rmdir -- "${scratch}"
  printf 'DOOR_SUPPORT_DETACH_PARALLEL_GATE=PASS phase=%s support=%s endpoint=sealed mechanism=three_samples\n' \
    "${phase}" "${expected_support}"
}

if [[ "${skip_pre_transport}" == 0 ]]; then
  require_transport_contract
else
  printf 'DOOR_SUPPORT_DETACH_8253_PRE_TRANSPORT=PRELOAD_ATTACH_CHECK_REUSED\n'
fi
parallel_detach_physical_gate "pre-detach" attached

timeout 5 gz topic -t "${detach_topic}" -m gz.msgs.Empty -p '' >/dev/null 2>&1 ||
  fail "support detach publication failed"

if [[ "${skip_post_transport}" == 0 ]]; then
  require_transport_contract
else
  printf 'DOOR_SUPPORT_DETACH_8253_POST_TRANSPORT=PRECHECK_REUSED\n'
fi
parallel_detach_physical_gate "post-detach" detached
printf 'DOOR_SUPPORT_DETACH_8253=PASS child=incubator::door_link door=q0 handle=q90 right_clamp=held\n'
