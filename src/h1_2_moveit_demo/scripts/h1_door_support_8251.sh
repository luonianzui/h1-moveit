#!/usr/bin/env bash
# Preload a detached support plugin, then attach the existing incubator door
# link to the verified right-hand clamp by a bounded Gazebo wrench.  This helper
# never commands a robot trajectory, an incubator joint, a model pose, reset,
# release, or collision-state change.
set -Eeuo pipefail

readonly operation="${1:-}"
readonly workspace="${H1_FULL_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly plugin_so="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/libh1_gazebo_compliant_grasp_system.so"
readonly asset_tool="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_joint_trajectory_asset.py"
readonly ros_snapshot_tool="${H1_FULL_ROS_SNAPSHOT_TOOL:-}"
readonly door_asset="${H1_DOOR_ASSET:-}"
readonly door_asset_sha256="${H1_DOOR_ASSET_SHA256:-}"
readonly attach_after_index="${H1_DOOR_ATTACH_AFTER_INDEX:-}"
readonly pose_topic="/world/h1_world/dynamic_pose/info"
readonly add_service="/world/h1_world/entity/system/add"
readonly latch_topic="/incubator/latch/state"
readonly state_topic="/h1_2/right_door_support/state"
readonly attach_topic="/h1_2/right_door_support/attach"
readonly detach_topic="/h1_2/right_door_support/detach"
readonly max_force="${H1_DOOR_SUPPORT_MAX_FORCE:-120.0}"
readonly max_torque="${H1_DOOR_SUPPORT_MAX_TORQUE:-8.0}"
readonly fast_resident="${H1_DOOR_SUPPORT_FAST_RESIDENT:-0}"
readonly recovery_owner_file="${H1_DOOR_RECOVERY_OWNER_FILE:-}"

fail()
{
  printf 'h1_door_support_8251: ERROR: %s\n' "$*" >&2
  exit 1
}

is_finite_decimal_text()
{
  [[ "$1" =~ ^[-+]?(([0-9]+(\.[0-9]*)?)|(\.[0-9]+))([eE][-+]?[0-9]+)?$ ]]
}

is_bounded_positive_decimal()
{
  local value="$1" maximum="$2"
  [[ "${value}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]] || return 1
  awk -v value="${value}" -v maximum="${maximum}" 'BEGIN {
    exit !(value+0 > 0 && value+0 <= maximum)
  }'
}

[[ "${operation}" == "preload" || "${operation}" == "attach" ||
  "${operation}" == "recover_attach" ]] || {
  printf 'usage: h1_door_support_8251.sh preload|attach|recover_attach\n' >&2
  exit 2
}
[[ "${fast_resident}" == "0" || "${fast_resident}" == "1" ]] ||
  fail "H1_DOOR_SUPPORT_FAST_RESIDENT must be exactly 0 or 1"
[[ "${fast_resident}" == "0" || "${operation}" == "attach" ||
  "${operation}" == "recover_attach" ]] ||
  fail "H1_DOOR_SUPPORT_FAST_RESIDENT=1 is valid only for attach after preload"
[[ "${operation}" != "recover_attach" || "${fast_resident}" == "1" ]] ||
  fail "recover_attach requires the already-resident fast path"
if [[ "${fast_resident}" == "0" ]]; then
  [[ -f "${plugin_so}" && -r "${plugin_so}" ]] ||
    fail "door support plugin is unavailable: ${plugin_so}"
  [[ "${plugin_so}" =~ ^/[-A-Za-z0-9_./+]+$ ]] ||
    fail "door support plugin path is unsafe for a protobuf request"
  [[ -x "${asset_tool}" ]] ||
    fail "sealed-asset tool is unavailable: ${asset_tool}"
fi
for command in gz jq ros2 timeout awk stat readlink ln chmod sed; do
  command -v "${command}" >/dev/null || fail "${command} is unavailable"
done
if [[ -n "${ros_snapshot_tool}" ]]; then
  [[ "${ros_snapshot_tool}" == /* && -f "${ros_snapshot_tool}" &&
    -x "${ros_snapshot_tool}" ]] ||
    fail "H1_FULL_ROS_SNAPSHOT_TOOL must name an absolute executable"
fi
[[ "${door_asset}" == /* && -f "${door_asset}" && ! -L "${door_asset}" ]] ||
  fail "H1_DOOR_ASSET must be an absolute non-symlink regular file"
[[ "${door_asset_sha256}" =~ ^[0-9a-f]{64}$ ]] ||
  fail "H1_DOOR_ASSET_SHA256 is malformed"
[[ "${attach_after_index}" =~ ^[1-9][0-9]*$ ]] ||
  fail "H1_DOOR_ATTACH_AFTER_INDEX must be positive"
is_bounded_positive_decimal "${max_force}" 250 &&
  is_bounded_positive_decimal "${max_torque}" 20 ||
  fail "door support force/torque caps must be canonical decimals in (0,250]/(0,20]"

if [[ "${fast_resident}" == "0" ]]; then
  # Preload proves the plugin binding and the complete sealed-asset topology.
  # A same-parent fast attach may trust that immutable ownership evidence and
  # retain only the live endpoint checks below.
  asset_inspection="$("${asset_tool}" inspect "${door_asset}" 2>&1)" ||
    fail "sealed door asset inspection failed"
  grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${asset_inspection}" ||
    fail "door asset schema is wrong"
  grep -Fxq 'stage=door' <<<"${asset_inspection}" ||
    fail "door asset stage is wrong"
  grep -Fxq "sha256=${door_asset_sha256}" <<<"${asset_inspection}" ||
    fail "door asset SHA-256 differs from the wrapper binding"
  asset_message_count="$(awk -F= '$1=="message_count" {print $2; found++}
    END {if(found!=1) exit 1}' <<<"${asset_inspection}")" ||
    fail "door asset message count is missing"
  [[ "${asset_message_count}" =~ ^[1-9][0-9]*$ &&
    ${attach_after_index} -lt ${asset_message_count} ]] ||
    fail "door support split is outside the sealed asset"
  jq -e --argjson attach "${attach_after_index}" '
    ["right_shoulder_pitch_joint", "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint", "right_elbow_joint",
      "right_wrist_roll_joint", "right_wrist_pitch_joint",
      "right_wrist_yaw_joint"] as $right_arm_joints |
    ["right_dex1_finger_1_joint", "right_dex1_finger_2_joint"] as $right_finger_joints |
    .payload.messages as $messages |
    all($messages[];
      (.topic == "/right_arm_controller/joint_trajectory" and
        .joint_names == $right_arm_joints) or
      (.topic == "/right_gripper_controller/joint_trajectory" and
        .joint_names == $right_finger_joints)) and
    ($messages[0:$attach] | map(select(
      .topic == "/right_arm_controller/joint_trajectory")) | length) >= 1 and
    ($messages[0:$attach] | map(select(
      .topic == "/right_gripper_controller/joint_trajectory")) | length) >= 2 and
    $messages[$attach - 1].topic ==
      "/right_gripper_controller/joint_trajectory" and
    ($messages[$attach:] | length) == 7 and
    all($messages[$attach:][];
      .topic == "/right_arm_controller/joint_trajectory")
  ' "${door_asset}" >/dev/null ||
    fail "door split must end at the clamp and precede exactly seven arm-only state segments"
fi

door_asset_real="$(readlink -f -- "${door_asset}")" ||
  fail "sealed door asset cannot be resolved"
plugin_so_real="$(readlink -f -- "${plugin_so}")" ||
  fail "door support plugin cannot be resolved"
parent_pid="${PPID}"
parent_start_ticks="$(sed 's/^[^)]*) //' "/proc/${parent_pid}/stat" 2>/dev/null |
  awk '{print $20}')" || fail "wrapper process identity is unavailable"
[[ "${parent_pid}" =~ ^[1-9][0-9]*$ &&
  "${parent_start_ticks}" =~ ^[1-9][0-9]*$ ]] ||
  fail "wrapper process identity is malformed"
ownership_dir="/tmp/h1_door_support_8251_owner_${EUID}"
if [[ ! -e "${ownership_dir}" && ! -L "${ownership_dir}" ]]; then
  mkdir -m 0700 -- "${ownership_dir}" ||
    fail "could not create door-support ownership directory"
fi
[[ -d "${ownership_dir}" && ! -L "${ownership_dir}" ]] ||
  fail "door-support ownership directory is not a real directory"
[[ "$(stat -c '%u %a' -- "${ownership_dir}")" == "${EUID} 700" ]] ||
  fail "door-support ownership directory metadata is invalid"
if [[ "${operation}" == "recover_attach" ]]; then
  [[ "${recovery_owner_file}" == /* && -f "${recovery_owner_file}" &&
    ! -L "${recovery_owner_file}" ]] ||
    fail "H1_DOOR_RECOVERY_OWNER_FILE must be an absolute sealed regular file"
  [[ "$(readlink -f -- "${recovery_owner_file}")" == "${recovery_owner_file}" ]] ||
    fail "recovery ownership path must already be canonical"
  recovery_owner_name="${recovery_owner_file##*/}"
  recovery_owner_pattern="^${door_asset_sha256}_([1-9][0-9]*)_([1-9][0-9]*)\.owner$"
  [[ "${recovery_owner_file%/*}" == "${ownership_dir}" &&
    "${recovery_owner_name}" =~ ${recovery_owner_pattern} ]] ||
    fail "recovery ownership path is outside the exact asset ownership namespace"
  parent_pid="${BASH_REMATCH[1]}"
  parent_start_ticks="${BASH_REMATCH[2]}"
  if [[ -r "/proc/${parent_pid}/stat" ]]; then
    live_parent_start_ticks="$(sed 's/^[^)]*) //' "/proc/${parent_pid}/stat" |
      awk '{print $20}')"
    [[ "${live_parent_start_ticks}" != "${parent_start_ticks}" ]] ||
      fail "recovery ownership parent is still active"
  fi
fi
ownership_file="${ownership_dir}/${door_asset_sha256}_${parent_pid}_${parent_start_ticks}.owner"
if [[ "${operation}" == "recover_attach" ]]; then
  [[ "${ownership_file}" == "${recovery_owner_file}" ]] ||
    fail "recovery ownership identity reconstruction failed"
fi

read_robot_joint_rows()
{
  if [[ -n "${ros_snapshot_tool}" ]]; then
    local attempt rows
    for attempt in 1 2 3; do
      if rows="$(timeout 5 "${ros_snapshot_tool}" robot --timeout 3)"; then
        printf '%s\n' "${rows}"
        return 0
      fi
    done
    return 1
  fi
  local attempt sample
  for attempt in 1 2 3; do
    if sample="$(timeout 8 ros2 topic echo --once /joint_states 2>/dev/null)"; then
      break
    fi
    sample=""
  done
  [[ -n "${sample}" ]] || return 1
  awk '
    /^name:/ {section="name"; next}
    /^position:/ {section="position"; next}
    /^velocity:/ {section="velocity"; next}
    /^effort:/ {section="effort"; next}
    section=="name" && /^- / {name[++names]=$2; next}
    section=="position" && /^- / {position[++positions]=$2; next}
    END {
      if (names==0 || positions!=names) exit 1
      for (i=1; i<=names; ++i) {gsub(/"/, "", name[i]); print name[i], position[i]}
    }
  ' <<<"${sample}"
}

asset_endpoint_gate()
{
  local topic="$1" expected_count="$2" label="$3"
  local live_rows="${4:-}"
  local endpoint_rows joint expected actual nominal count=0
  local -A seen=()
  endpoint_rows="$(jq -er --arg topic "${topic}" --argjson boundary "${attach_after_index}" \
    --argjson expected_count "${expected_count}" '
    [.payload.messages[0:$boundary][] | select(.topic==$topic)] as $selected |
    if ($selected|length)<1 then error("endpoint absent") else $selected[-1] end |
    . as $message | $message.points[-1].positions as $positions |
    if (($message.joint_names|length)==$expected_count and
      ($positions|length)==$expected_count)
    then range(0;$expected_count) as $i |
      [$message.joint_names[$i],$positions[$i]] | @tsv
    else error("malformed endpoint") end
  ' "${door_asset}")" || fail "${label}: sealed endpoint is unavailable"
  if [[ -z "${live_rows}" ]]; then
    live_rows="$(read_robot_joint_rows)" ||
      fail "${label}: robot telemetry is malformed"
  fi
  while IFS=$'\t' read -r joint expected; do
    [[ -n "${joint}" && -z "${seen[${joint}]:-}" ]] ||
      fail "${label}: duplicate or empty joint"
    case "${topic}" in
      /right_arm_controller/joint_trajectory)
        case "${joint}" in
          right_shoulder_pitch_joint|right_shoulder_roll_joint|\
          right_shoulder_yaw_joint|right_elbow_joint|right_wrist_roll_joint|\
          right_wrist_pitch_joint|right_wrist_yaw_joint)
            ;;
          *)
            fail "${label}: sealed right-arm endpoint has an unexpected joint"
            ;;
        esac
        ;;
      /right_gripper_controller/joint_trajectory)
        case "${joint}" in
          right_dex1_finger_1_joint|right_dex1_finger_2_joint)
            ;;
          *)
            fail "${label}: sealed right-finger endpoint has an unexpected joint"
            ;;
        esac
        ;;
      *)
        fail "${label}: unsupported endpoint topic"
        ;;
    esac
    seen["${joint}"]=1
    actual="$(awk -v joint="${joint}" '$1==joint {print $2; found++}
      END {if(found!=1) exit 1}' <<<"${live_rows}")" ||
      fail "${label}: live ${joint} is absent or duplicated"
    is_finite_decimal_text "${expected}" && is_finite_decimal_text "${actual}" ||
      fail "${label}: ${joint} endpoint is not a finite decimal"
    awk -v actual="${actual}" -v expected="${expected}" 'BEGIN {
      error=actual-expected; if(error<0) error=-error; exit !(error<=0.003)
    }' || fail "${label}: live ${joint} differs from the sealed endpoint"
    case "${joint}" in
      right_dex1_finger_1_joint)
        nominal=0.01131
        ;;
      right_dex1_finger_2_joint)
        nominal=0.01235
        ;;
      *)
        nominal=""
        ;;
    esac
    if [[ -n "${nominal}" ]]; then
      awk -v actual="${actual}" -v expected="${expected}" \
        -v nominal="${nominal}" 'BEGIN {
        actual_error=actual-nominal; if(actual_error<0) actual_error=-actual_error
        expected_error=expected-nominal; if(expected_error<0) expected_error=-expected_error
        exit !(actual_error<=0.0015 && expected_error<=0.0015)
      }' || fail "${label}: ${joint} does not certify the right-hand clamp"
    fi
    ((count+=1))
  done <<<"${endpoint_rows}"
  (( count == expected_count )) || fail "${label}: endpoint cardinality is wrong"
}

mechanism_rows()
{
  if [[ -n "${ros_snapshot_tool}" ]]; then
    local attempt rows
    for attempt in 1 2 3; do
      if rows="$(timeout 5 "${ros_snapshot_tool}" incubator --timeout 3)"; then
        printf '%s\n' "${rows}"
        return 0
      fi
    done
    return 1
  fi
  local attempt sample
  for attempt in 1 2 3; do
    if sample="$(timeout 8 ros2 topic echo --once /incubator/joint_states 2>/dev/null)"; then
      break
    fi
    sample=""
  done
  [[ -n "${sample}" ]] || return 1
  awk '
    /^name:/ {section="name"; next}
    /^position:/ {section="position"; next}
    /^velocity:/ {section="velocity"; next}
    /^effort:/ {section="effort"; next}
    section=="name" && /^- / {name[++names]=$2; next}
    section=="position" && /^- / {position[++positions]=$2; next}
    section=="velocity" && /^- / {velocity[++velocities]=$2; next}
    END {
      if (names<2 || positions!=names || velocities!=names) exit 1
      for(i=1;i<=names;++i) {gsub(/"/,"",name[i]); print name[i],position[i],velocity[i]}
    }
  ' <<<"${sample}"
}

mechanism_gate()
{
  local label="$1" rows door handle door_position door_velocity
  local handle_position handle_velocity value
  rows="$(mechanism_rows)" || fail "${label}: incubator telemetry is malformed"
  door="$(awk '$1=="door_hinge_joint" {print $2,$3; found++}
    END {if(found!=1) exit 1}' <<<"${rows}")" || fail "${label}: door row missing"
  handle="$(awk '$1=="handle_latch_joint" {print $2,$3; found++}
    END {if(found!=1) exit 1}' <<<"${rows}")" || fail "${label}: handle row missing"
  read -r door_position door_velocity <<<"${door}"
  read -r handle_position handle_velocity <<<"${handle}"
  for value in "${door_position}" "${door_velocity}" \
    "${handle_position}" "${handle_velocity}"
  do
    is_finite_decimal_text "${value}" ||
      fail "${label}: mechanism telemetry is not finite"
  done
  awk -v door="${door}" -v handle="${handle}" 'BEGIN {
    split(door,d," "); split(handle,h," "); pi=3.141592653589793
    de=d[1]; if(de<0)de=-de; he=h[1]-pi/2; if(he<0)he=-he
    dv=d[2]; if(dv<0)dv=-dv; hv=h[2]; if(hv<0)hv=-hv
    exit !(de<=0.003490658503989 && he<=0.052359877559830 &&
      dv<=0.002 && hv<=0.002)
  }' || fail "${label}: door/handle is not a stationary q0/90 clamp"
}

monitor_complete_sample_count()
{
  awk '$0=="---" {count++} END {print count+0}' \
    "${temporary_dir}/mechanism.txt" 2>/dev/null
}

start_mechanism_pulse_monitor()
{
  : >"${temporary_dir}/mechanism.txt"
  PYTHONUNBUFFERED=1 timeout 45 ros2 topic echo /incubator/joint_states \
    >"${temporary_dir}/mechanism.txt" 2>&1 &
  mechanism_listener_pid=$!
  local attempt count
  for attempt in {1..80}; do
    kill -0 "${mechanism_listener_pid}" 2>/dev/null ||
      fail "mechanism pulse monitor exited before its first complete sample"
    count="$(monitor_complete_sample_count)"
    if (( count >= 1 )); then
      refresh_mechanism_monitor_baseline
      return 0
    fi
    sleep 0.05
  done
  fail "mechanism pulse monitor produced no complete pre-attach sample"
}

stop_mechanism_pulse_monitor()
{
  if [[ -n "${mechanism_listener_pid}" ]] &&
    kill -0 "${mechanism_listener_pid}" 2>/dev/null
  then
    kill -TERM "${mechanism_listener_pid}" 2>/dev/null || true
    wait "${mechanism_listener_pid}" 2>/dev/null || true
  fi
  mechanism_listener_pid=""
}

mechanism_monitor_rows()
{
  awk '
    function emit(i, door_count, handle_count, door_position, door_velocity,
      handle_position, handle_velocity, number_pattern) {
      if(names==0 && positions==0 && velocities==0) return
      number_pattern="^[-+]?(([0-9]+(\\.[0-9]*)?)|(\\.[0-9]+))([eE][-+]?[0-9]+)?$"
      if(names<2 || positions!=names || velocities!=names) {bad=1; reset(); return}
      for(i=1;i<=names;++i) {
        gsub(/\"/, "", name[i])
        if(position[i] !~ number_pattern || velocity[i] !~ number_pattern) bad=1
        if(name[i]=="door_hinge_joint") {
          door_count++; door_position=position[i]; door_velocity=velocity[i]
        }
        if(name[i]=="handle_latch_joint") {
          handle_count++; handle_position=position[i]; handle_velocity=velocity[i]
        }
      }
      if(door_count!=1 || handle_count!=1) bad=1
      if(!bad) {print door_position,door_velocity,handle_position,handle_velocity; samples++}
      reset()
    }
    function reset() {names=positions=velocities=0; section=""}
    /^name:/ {section="name"; next}
    /^position:/ {section="position"; next}
    /^velocity:/ {section="velocity"; next}
    /^effort:/ {section="effort"; next}
    section=="name" && /^- / {name[++names]=$2; next}
    section=="position" && /^- / {position[++positions]=$2; next}
    section=="velocity" && /^- / {velocity[++velocities]=$2; next}
    $0=="---" {emit(); next}
    END {if(bad || samples<1) exit 1}
  ' "${temporary_dir}/mechanism.txt"
}

mechanism_rows_envelope_gate()
{
  local rows="$1" label="$2"
  awk 'BEGIN {pi=3.141592653589793}
    {
      door=$1; if(door<0)door=-door
      door_velocity=$2; if(door_velocity<0)door_velocity=-door_velocity
      handle=$3-pi/2; if(handle<0)handle=-handle
      handle_velocity=$4; if(handle_velocity<0)handle_velocity=-handle_velocity
      if(door>0.003490658503989 || handle>0.052359877559830 ||
        door_velocity>0.002 || handle_velocity>0.002) bad=1
    }
    END {exit !(NR>0 && !bad)}' <<<"${rows}" ||
    fail "${label}: door/handle crossed the stationary q0/90 envelope"
}

refresh_mechanism_monitor_baseline()
{
  local rows parsed_count
  kill -0 "${mechanism_listener_pid}" 2>/dev/null ||
    fail "mechanism pulse monitor exited before the attach publication"
  rows="$(mechanism_monitor_rows)" ||
    fail "mechanism pulse monitor pre-attach sample is malformed"
  parsed_count="$(awk 'END {print NR+0}' <<<"${rows}")"
  (( parsed_count >= 1 )) ||
    fail "mechanism pulse monitor has no parsed pre-attach sample"
  mechanism_rows_envelope_gate "${rows}" "pre-attach mechanism pulse monitor"
  pre_attach_mechanism_sample_count="${parsed_count}"
}

mechanism_pulse_gate()
{
  local rows sample_count
  rows="$(mechanism_monitor_rows)" ||
    fail "post-attach pulse monitor contains malformed mechanism telemetry"
  sample_count="$(awk 'END {print NR+0}' <<<"${rows}")"
  (( sample_count >= pre_attach_mechanism_sample_count + 3 )) ||
    fail "post-attach pulse monitor lacks three samples after the attach boundary"
  mechanism_rows_envelope_gate "${rows}" "post-attach mechanism pulse monitor"
}

require_unlocked_latch()
{
  local output
  output="$(timeout 5 gz topic -e -t "${latch_topic}" -n 1 2>/dev/null)" ||
    fail "latch telemetry is unavailable"
  grep -Fxq 'data: "unlocked"' <<<"${output}" || fail "latch is not unlocked"
}

read_incubator_json()
{
  timeout 5 gz topic -e --json-output -t "${pose_topic}" -n 1 |
    jq -cer '[.pose[] | select(.name=="incubator" and .id!=0)] |
      if length==1 then .[0] else error("incubator entity not unique") end'
}

transport_info()
{
  timeout 5 gz topic -i -t "$1" 2>&1
}

endpoint_count()
{
  awk '/^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// {count++}
    END {print count+0}' <<<"$1"
}

type_count()
{
  local info="$1" type="$2"
  awk -F', ' -v type="${type}" '
    /^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// && $2==type {count++}
    END {print count+0}' <<<"${info}"
}

listener_is_discovered()
{
  local mode="$1" info endpoints state_types listener_types
  info="$(transport_info "${state_topic}" 2>/dev/null || true)"
  endpoints="$(endpoint_count "${info}")"
  state_types="$(type_count "${info}" gz.msgs.StringMsg)"
  listener_types="$(type_count "${info}" google.protobuf.Message)"
  if [[ "${mode}" == "before_add" ]]; then
    grep -Fqx "No publishers on topic [${state_topic}]" <<<"${info}" &&
      grep -Fq 'Subscribers [Address, Message Type]:' <<<"${info}" &&
      [[ "${endpoints}" == 1 && "${state_types}" == 0 &&
        "${listener_types}" == 1 ]]
  else
    grep -Fq 'Publishers [Address, Message Type]:' <<<"${info}" &&
      grep -Fq 'Subscribers [Address, Message Type]:' <<<"${info}" &&
      [[ "${endpoints}" == 2 && "${state_types}" == 1 &&
        "${listener_types}" == 1 ]]
  fi
}

wait_for_listener_discovery()
{
  local mode="$1" attempt
  for attempt in {1..40}; do
    kill -0 "${listener_pid}" 2>/dev/null ||
      fail "state listener exited during ${mode} discovery"
    if listener_is_discovered "${mode}"; then
      return 0
    fi
    sleep 0.05
  done
  fail "state listener was not discovered with exact ${mode} direction/type/count"
}

start_state_listener()
{
  local discovery_mode="$1"
  : >"${temporary_dir}/state.txt"
  timeout 20 gz topic -e -t "${state_topic}" \
    >"${temporary_dir}/state.txt" 2>&1 &
  listener_pid=$!
  if [[ "${discovery_mode}" != "fast_resident" ]]; then
    wait_for_listener_discovery "${discovery_mode}"
  fi
}

require_fresh_state()
{
  local expected="$1" attempt
  for attempt in {1..160}; do
    if grep -Eq "^[[:space:]]*data:[[:space:]]*['\"]?${expected}['\"]?[[:space:]]*$" \
      "${temporary_dir}/state.txt"
    then
      return 0
    fi
    kill -0 "${listener_pid}" 2>/dev/null ||
      fail "state listener stopped before fresh ${expected} confirmation"
    sleep 0.05
  done
  fail "door support did not report fresh ${expected} state"
}

require_resident_transport_contract()
{
  local listener_mode="${1:-with_listener}"
  [[ "${listener_mode}" == "with_listener" ||
    "${listener_mode}" == "without_listener" ]] ||
    fail "resident transport listener mode is invalid"
  local state_info attach_info detach_info
  read_transport_infos || fail "resident support topic inspection failed"
  state_info="$(<"${temporary_dir}/state.info")"
  attach_info="$(<"${temporary_dir}/attach.info")"
  detach_info="$(<"${temporary_dir}/detach.info")"
  if [[ "${listener_mode}" == "with_listener" ]]; then
    grep -Fq 'Publishers [Address, Message Type]:' <<<"${state_info}" &&
      grep -Fq 'Subscribers [Address, Message Type]:' <<<"${state_info}" &&
      [[ "$(endpoint_count "${state_info}")" == 2 &&
        "$(type_count "${state_info}" gz.msgs.StringMsg)" == 1 &&
        "$(type_count "${state_info}" google.protobuf.Message)" == 1 ]] ||
      fail "state topic does not have one plugin publisher and one listener"
  else
    grep -Fq 'Publishers [Address, Message Type]:' <<<"${state_info}" &&
      grep -Fqx "No subscribers on topic [${state_topic}]" <<<"${state_info}" &&
      [[ "$(endpoint_count "${state_info}")" == 1 &&
        "$(type_count "${state_info}" gz.msgs.StringMsg)" == 1 &&
        "$(type_count "${state_info}" google.protobuf.Message)" == 0 ]] ||
      fail "state topic is not owned by exactly one plugin publisher"
  fi
  grep -Fqx "No publishers on topic [${attach_topic}]" <<<"${attach_info}" &&
    grep -Fq 'Subscribers [Address, Message Type]:' <<<"${attach_info}" &&
    [[ "$(endpoint_count "${attach_info}")" == 1 &&
      "$(type_count "${attach_info}" gz.msgs.Empty)" == 1 ]] ||
    fail "attach topic does not have exactly one Empty subscriber"
  grep -Fqx "No publishers on topic [${detach_topic}]" <<<"${detach_info}" &&
    grep -Fq 'Subscribers [Address, Message Type]:' <<<"${detach_info}" &&
    [[ "$(endpoint_count "${detach_info}")" == 1 &&
      "$(type_count "${detach_info}" gz.msgs.Empty)" == 1 ]] ||
    fail "detach topic does not have exactly one Empty subscriber"
}

read_transport_infos()
{
  local state_pid attach_pid detach_pid status=0
  transport_info "${state_topic}" >"${temporary_dir}/state.info" &
  state_pid=$!
  transport_info "${attach_topic}" >"${temporary_dir}/attach.info" &
  attach_pid=$!
  transport_info "${detach_topic}" >"${temporary_dir}/detach.info" &
  detach_pid=$!
  wait "${state_pid}" || status=1
  wait "${attach_pid}" || status=1
  wait "${detach_pid}" || status=1
  (( status == 0 ))
}

require_empty_transport_contract()
{
  read_transport_infos || fail "could not inspect the door-support topics"
  local index topic info
  local -a topics=("${state_topic}" "${attach_topic}" "${detach_topic}")
  local -a info_files=("${temporary_dir}/state.info" \
    "${temporary_dir}/attach.info" "${temporary_dir}/detach.info")
  for index in "${!topics[@]}"; do
    topic="${topics[${index}]}"
    info="$(<"${info_files[${index}]}")"
    grep -Fqx "No publishers on topic [${topic}]" <<<"${info}" &&
      grep -Fqx "No subscribers on topic [${topic}]" <<<"${info}" &&
      [[ "$(endpoint_count "${info}")" == 0 ]] ||
      fail "a door-support endpoint already exists on ${topic}"
  done
}

ownership_record()
{
  printf '%s\n' \
    'schema=h1_door_support_preload_owner/v1' \
    "door_asset=${door_asset_real}" \
    "door_asset_sha256=${door_asset_sha256}" \
    "attach_after_index=${attach_after_index}" \
    "plugin_so=${plugin_so_real}" \
    "max_force=${max_force}" \
    "max_torque=${max_torque}" \
    'initially_attached=false' \
    "parent_pid=${parent_pid}" \
    "parent_start_ticks=${parent_start_ticks}"
}

create_ownership_evidence()
{
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run door preload ownership evidence already exists"
  (umask 077; ownership_record >"${temporary_dir}/owner.tmp") ||
    fail "could not create door preload ownership payload"
  chmod 0400 "${temporary_dir}/owner.tmp" ||
    fail "could not seal door preload ownership payload"
  ln -- "${temporary_dir}/owner.tmp" "${ownership_file}" ||
    fail "exclusive door preload ownership create failed"
  rm -f -- "${temporary_dir}/owner.tmp"
}

require_ownership_evidence()
{
  [[ -f "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run detached door preload ownership evidence is absent"
  [[ "$(stat -c '%u %a %h' -- "${ownership_file}")" == "${EUID} 400 1" ]] ||
    fail "door preload ownership evidence metadata is invalid"
  local actual expected
  actual="$(<"${ownership_file}")"
  expected="$(ownership_record)"
  [[ "${actual}" == "${expected}" ]] ||
    fail "door preload ownership evidence content is invalid"
}

temporary_dir="$(mktemp -d /tmp/h1_door_support_8251.XXXXXX)"
listener_pid=""
mechanism_listener_pid=""
pre_attach_mechanism_sample_count="0"
cleanup()
{
  if [[ -n "${listener_pid}" ]] && kill -0 "${listener_pid}" 2>/dev/null; then
    kill -TERM "${listener_pid}" 2>/dev/null || true
    wait "${listener_pid}" 2>/dev/null || true
  fi
  if [[ -n "${mechanism_listener_pid}" ]] &&
    kill -0 "${mechanism_listener_pid}" 2>/dev/null
  then
    kill -TERM "${mechanism_listener_pid}" 2>/dev/null || true
    wait "${mechanism_listener_pid}" 2>/dev/null || true
  fi
  rm -f -- "${temporary_dir}/state.txt" "${temporary_dir}/mechanism.txt" \
    "${temporary_dir}/state.info" "${temporary_dir}/attach.info" \
    "${temporary_dir}/detach.info" "${temporary_dir}/service.info" \
    "${temporary_dir}/pre_robot.rows" "${temporary_dir}/pre_mechanism.log" \
    "${temporary_dir}/pre_latch.log" "${temporary_dir}/owner.tmp"
  rmdir "${temporary_dir}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

require_add_service_contract()
{
  local service_info
  service_info="$(timeout 5 gz service -i -s "${add_service}" 2>&1)" ||
    fail "could not inspect the system-add service"
  grep -Fqx 'Service providers [Address, Request Message Type, Response Message Type]:' \
    <<<"${service_info}" || fail "system-add service discovery header is malformed"
  awk -F', ' '
    /^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// {
      providers++; if($2!="gz.msgs.EntityPlugin_V" || $3!="gz.msgs.Boolean") bad=1
    }
    END {exit !(providers==1 && !bad)}
  ' <<<"${service_info}" || fail "system-add service contract is not unique"
}

run_preload()
{
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run door preload ownership evidence already exists"
  local incubator_json incubator_id request response
  incubator_json="$(read_incubator_json)" || fail "could not resolve incubator model"
  incubator_id="$(jq -er '.id|tonumber' <<<"${incubator_json}")" ||
    fail "incubator entity ID is malformed"
  [[ "${incubator_id}" =~ ^[1-9][0-9]*$ ]] ||
    fail "incubator entity ID is not positive"
  require_empty_transport_contract
  require_add_service_contract
  start_state_listener before_add
  request="entity {id: ${incubator_id}, name: \"incubator\", type: MODEL} plugins {name: \"h1_2::GazeboCompliantGraspSystem\", filename: \"${plugin_so}\", innerxml: \"<parent_scope>h1_2::right_wrist_yaw_link</parent_scope><child_scope>incubator::door_link</child_scope><gravity_compensation>false</gravity_compensation><max_force>${max_force}</max_force><max_torque>${max_torque}</max_torque><initially_attached>false</initially_attached><detach_topic>${detach_topic}</detach_topic><attach_topic>${attach_topic}</attach_topic><output_topic>${state_topic}</output_topic>\"}"
  response="$(timeout 12 gz service -s "${add_service}" \
    --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 10000 \
    --req "${request}" 2>&1)" || fail "door support preload add request failed"
  grep -Eq '^[[:space:]]*data:[[:space:]]*true[[:space:]]*$' <<<"${response}" ||
    fail "Gazebo rejected the detached door support preload"
  wait_for_listener_discovery resident
  require_fresh_state detached
  require_resident_transport_contract with_listener
  create_ownership_evidence
  printf 'DOOR_SUPPORT_PRELOAD=PASS child=incubator::door_link force_cap=%s torque_cap=%s\n' \
    "${max_force}" "${max_torque}"
}

run_attach()
{
  require_ownership_evidence
  if [[ "${fast_resident}" == "0" ]]; then
    require_resident_transport_contract without_listener
  fi
  start_mechanism_pulse_monitor

  local pre_attach_gate_status=0
  read_robot_joint_rows >"${temporary_dir}/pre_robot.rows" &
  local pre_robot_pid=$!
  mechanism_gate "pre-attach mechanism gate" \
    >"${temporary_dir}/pre_mechanism.log" 2>&1 &
  local pre_mechanism_pid=$!
  require_unlocked_latch >"${temporary_dir}/pre_latch.log" 2>&1 &
  local pre_latch_pid=$!
  wait "${pre_robot_pid}" || pre_attach_gate_status=1
  wait "${pre_mechanism_pid}" || pre_attach_gate_status=1
  wait "${pre_latch_pid}" || pre_attach_gate_status=1
  if (( pre_attach_gate_status != 0 )); then
    sed -n 'p' "${temporary_dir}/pre_mechanism.log" >&2
    sed -n 'p' "${temporary_dir}/pre_latch.log" >&2
    fail "parallel pre-attach robot/mechanism/latch telemetry gate failed"
  fi
  local pre_attach_robot_rows
  pre_attach_robot_rows="$(<"${temporary_dir}/pre_robot.rows")"
  [[ -n "${pre_attach_robot_rows}" ]] ||
    fail "pre-attach robot telemetry is malformed"
  asset_endpoint_gate "/right_arm_controller/joint_trajectory" 7 \
    "right-arm clamp endpoint" "${pre_attach_robot_rows}"
  asset_endpoint_gate "/right_gripper_controller/joint_trajectory" 2 \
    "right-finger clamp endpoint" "${pre_attach_robot_rows}"

  if [[ "${fast_resident}" == "1" ]]; then
    start_state_listener fast_resident
  else
    start_state_listener resident
  fi
  require_fresh_state detached
  refresh_mechanism_monitor_baseline
  timeout 5 gz topic -t "${attach_topic}" -m gz.msgs.Empty -p '' \
    >/dev/null 2>&1 || fail "door support attach publication failed"
  require_fresh_state attached
  if [[ "${fast_resident}" == "0" ]]; then
    require_resident_transport_contract with_listener
  fi

  local attempt post_attach_sample_count=0
  for attempt in {1..80}; do
    kill -0 "${mechanism_listener_pid}" 2>/dev/null ||
      fail "mechanism pulse monitor exited before three post-attach samples"
    post_attach_sample_count="$(monitor_complete_sample_count)"
    if (( post_attach_sample_count >= pre_attach_mechanism_sample_count + 3 )); then
      break
    fi
    sleep 0.05
  done
  (( post_attach_sample_count >= pre_attach_mechanism_sample_count + 3 )) ||
    fail "mechanism pulse monitor did not span the attach publication with three samples"
  stop_mechanism_pulse_monitor
  mechanism_pulse_gate
  local post_attach_robot_rows
  post_attach_robot_rows="$(read_robot_joint_rows)" ||
    fail "post-attach robot telemetry is malformed"
  asset_endpoint_gate "/right_arm_controller/joint_trajectory" 7 \
    "post-attach right-arm endpoint" "${post_attach_robot_rows}"
  asset_endpoint_gate "/right_gripper_controller/joint_trajectory" 2 \
    "post-attach right-finger endpoint" "${post_attach_robot_rows}"

  if [[ "${fast_resident}" == "1" ]]; then
    printf 'DOOR_SUPPORT_FAST_RESIDENT=PASS ownership=verified endpoints=pre_and_post mechanism=pulse state=detached_to_attached_ack\n'
  fi
  printf 'DOOR_SUPPORT_HOOK=PASS operation=attach child=incubator::door_link force_cap=%s torque_cap=%s\n' \
    "${max_force}" "${max_torque}"
}

if [[ "${operation}" == "preload" ]]; then
  run_preload
else
  run_attach
fi
