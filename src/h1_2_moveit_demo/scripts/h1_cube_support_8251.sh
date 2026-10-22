#!/usr/bin/env bash
# Verified Gazebo support hook used only between sealed robot trajectories.
# It never moves a robot joint, an incubator joint, or a model pose.
set -Eeuo pipefail

readonly operation="${1:-}"
readonly workspace="${H1_FULL_WORKSPACE:-/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws}"
readonly plugin_so="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/libh1_gazebo_compliant_grasp_system.so"
readonly asset_tool="${workspace}/install/h1_2_moveit_demo/lib/h1_2_moveit_demo/h1_joint_trajectory_asset.py"
readonly ros_snapshot_tool="${H1_FULL_ROS_SNAPSHOT_TOOL:-}"
readonly pose_topic="/world/h1_world/dynamic_pose/info"
readonly add_service="/world/h1_world/entity/system/add"
readonly state_topic="/h1_2/left_pose_grasp/state"
readonly attach_topic="/h1_2/left_pose_grasp/attach"
readonly detach_topic="/h1_2/left_pose_grasp/detach"
readonly cube_asset="${H1_CUBE_ASSET:-}"
readonly cube_asset_sha256="${H1_CUBE_ASSET_SHA256:-}"
readonly attach_after_index="${H1_CUBE_ATTACH_AFTER_INDEX:-}"
readonly detach_after_index="${H1_CUBE_DETACH_AFTER_INDEX:-}"
readonly rack_position_tolerance="${H1_CUBE_RACK_POSITION_TOLERANCE:-0.003}"
readonly rack_orientation_tolerance="${H1_CUBE_RACK_ORIENTATION_TOLERANCE:-0.034906585039887}"
readonly arm_endpoint_tolerance="${H1_CUBE_ARM_ENDPOINT_TOLERANCE:-0.003}"
readonly rack_expected_yaw_degrees="${H1_CUBE_RACK_EXPECTED_YAW:-70}"
readonly fast_resident="${H1_CUBE_SUPPORT_FAST_RESIDENT:-0}"
readonly recovery_preload_file="${H1_CUBE_RECOVERY_PRELOAD_FILE:-}"

fail()
{
  printf 'h1_cube_support_8251: ERROR: %s\n' "$*" >&2
  exit 1
}

[[ "${operation}" == "preload" || "${operation}" == "attach" ||
  "${operation}" == "detach" || "${operation}" == "recover_attach" ||
  "${operation}" == "recover_detach" ]] || {
  printf 'usage: h1_cube_support_8251.sh preload|attach|detach|recover_attach|recover_detach\n' >&2
  exit 2
}
case "${rack_expected_yaw_degrees}" in
  70)
    rack_expected_yaw_rad='1.221730476396031'
    ;;
  90)
    rack_expected_yaw_rad='1.570796326794897'
    ;;
  *)
    fail "H1_CUBE_RACK_EXPECTED_YAW must be exactly 70 or 90 degrees"
    ;;
esac
readonly rack_expected_yaw_rad
[[ "${fast_resident}" == "0" || "${fast_resident}" == "1" ]] ||
  fail "H1_CUBE_SUPPORT_FAST_RESIDENT must be exactly 0 or 1"
[[ -f "${plugin_so}" && -r "${plugin_so}" ]] ||
  fail "support plugin is unavailable: ${plugin_so}"
[[ -x "${asset_tool}" ]] || fail "sealed-asset inspector is unavailable: ${asset_tool}"
command -v gz >/dev/null || fail "gz is unavailable"
command -v jq >/dev/null || fail "jq is unavailable"
command -v timeout >/dev/null || fail "timeout is unavailable"
[[ "${ros_snapshot_tool}" == /* && -f "${ros_snapshot_tool}" &&
  -x "${ros_snapshot_tool}" ]] ||
  fail "H1_FULL_ROS_SNAPSHOT_TOOL must name an absolute executable"

[[ "${cube_asset}" == /* && -f "${cube_asset}" && ! -L "${cube_asset}" ]] ||
  fail "H1_CUBE_ASSET must be an absolute, non-symlink regular file"
[[ "${cube_asset_sha256}" =~ ^[0-9a-f]{64}$ ]] ||
  fail "H1_CUBE_ASSET_SHA256 is malformed"
[[ "${attach_after_index}" =~ ^[1-9][0-9]*$ &&
  "${detach_after_index}" =~ ^[1-9][0-9]*$ &&
  ${attach_after_index} -lt ${detach_after_index} ]] ||
  fail "cube split indices must satisfy 0 < attach < detach"
[[ "${rack_position_tolerance}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ &&
  "${rack_orientation_tolerance}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ &&
  "${arm_endpoint_tolerance}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]] ||
  fail "rack tolerances must be finite non-negative decimals"
awk -v position="${rack_position_tolerance}" \
  -v orientation="${rack_orientation_tolerance}" 'BEGIN {
    exit !(position >= 0.003 && position <= 0.030 &&
      orientation >= 0.034906585039887 && orientation <= 0.523598775598299)
  }' || fail "rack tolerances exceed the admitted Gazebo experiment bounds"
if [[ "${rack_expected_yaw_degrees}" == "90" ]]; then
  awk -v orientation="${rack_orientation_tolerance}" 'BEGIN {
    exit !(orientation <= 0.034906585039887)
  }' || fail "90-degree rack target requires the two-degree orientation gate"
fi
awk -v tolerance="${arm_endpoint_tolerance}" 'BEGIN {
    exit !(tolerance >= 0.003 && tolerance <= 0.012)
  }' || fail "arm endpoint tolerance exceeds the admitted Gazebo experiment bounds"

asset_inspection="$("${asset_tool}" inspect "${cube_asset}" 2>&1)" ||
  fail "sealed cube asset inspection failed"
grep -Fxq 'schema=h1_joint_trajectory_asset/v1' <<<"${asset_inspection}" ||
  fail "cube asset schema is wrong"
grep -Fxq 'stage=cube' <<<"${asset_inspection}" || fail "cube asset stage is wrong"
grep -Fxq "sha256=${cube_asset_sha256}" <<<"${asset_inspection}" ||
  fail "cube asset SHA-256 does not match the wrapper binding"
asset_message_count="$(awk -F= '$1=="message_count" {print $2; found++}
  END {if(found!=1) exit 1}' <<<"${asset_inspection}")" ||
  fail "cube asset message count is missing"
[[ "${asset_message_count}" =~ ^[1-9][0-9]*$ &&
  ${detach_after_index} -lt ${asset_message_count} ]] ||
  fail "cube split indices exceed the sealed asset"
jq -e --argjson attach "${attach_after_index}" \
  --argjson detach "${detach_after_index}" '
    .payload.messages[$attach - 1].topic ==
      "/left_gripper_controller/joint_trajectory" and
    .payload.messages[$attach - 1].expected_actual_endpoint.kind ==
      "cube_gripper_contact_close" and
    .payload.messages[$attach - 1].points[-1].positions ==
      [-0.01472, -0.01472] and
    (.payload.messages[$attach - 1].expected_actual_endpoint.positions |
      type == "array" and length == 2) and
    ([.payload.messages[$attach:$detach][] |
      select(.topic == "/left_gripper_controller/joint_trajectory")] |
      length) == 0 and
    .payload.messages[$detach - 1].topic ==
      "/left_arm_controller/joint_trajectory" and
    .payload.messages[$detach].topic ==
      "/left_gripper_controller/joint_trajectory"
  ' "${cube_asset}" >/dev/null ||
  fail "sealed cube split does not surround close / supported carry / open"

cube_asset_real="$(readlink -f -- "${cube_asset}")" || fail "cube asset cannot be resolved"
[[ -n "${cube_asset_real}" && "${cube_asset_real}" != *$'\n'* ]] ||
  fail "resolved cube asset path is invalid"
parent_pid="${PPID}"
parent_start_ticks="$(sed 's/^[^)]*) //' "/proc/${parent_pid}/stat" 2>/dev/null |
  awk '{print $20}')" || fail "wrapper process identity is unavailable"
[[ "${parent_pid}" =~ ^[1-9][0-9]*$ && "${parent_start_ticks}" =~ ^[1-9][0-9]*$ ]] ||
  fail "wrapper process identity is malformed"

read_cube_json()
{
  local attempt json
  for attempt in 1 2 3; do
    if json="$(
      timeout 5 gz topic -e --json-output -t "${pose_topic}" -n 1 |
        jq -cer '[.pose[] | select(.name=="green_cube" and .id != 0)] |
          if length == 1 then .[0]
          else error("green_cube entity not unique") end'
    )"
    then
      printf '%s\n' "${json}"
      return 0
    fi
  done
  return 1
}

cube_json="$(read_cube_json)" || fail "could not resolve one live green_cube entity"
cube_id="$(jq -er '.id | tonumber' <<<"${cube_json}")" ||
  fail "green_cube entity ID is malformed"
[[ "${cube_id}" =~ ^[1-9][0-9]*$ ]] || fail "green_cube entity ID is not positive"

cube_pose_row()
{
  local json
  json="$(read_cube_json)" || return 1
  jq -er '
    [.id, .position.x, .position.y, .position.z,
      (.orientation.x // 0), (.orientation.y // 0),
      (.orientation.z // 0), (.orientation.w // 0)] as $values |
    if ($values | length) == 8 and all($values[]; type == "number")
    then $values | @tsv else error("malformed green_cube pose") end
  ' <<<"${json}"
}

cube_pose_gate()
{
  local expected_x="$1" expected_y="$2" expected_z="$3"
  local expected_yaw="$4" position_tolerance="$5" orientation_tolerance="$6"
  local label="$7" row id x y z qx qy qz qw
  row="$(cube_pose_row)" || fail "${label}: no fresh finite green_cube pose"
  IFS=$'\t' read -r id x y z qx qy qz qw <<<"${row}"
  [[ "${id}" == "${cube_id}" ]] || fail "${label}: green_cube entity changed"
  awk -v x="${x}" -v y="${y}" -v z="${z}" -v qx="${qx}" -v qy="${qy}" \
    -v qz="${qz}" -v qw="${qw}" -v expected_x="${expected_x}" \
    -v expected_y="${expected_y}" -v expected_z="${expected_z}" \
    -v expected_yaw="${expected_yaw}" -v position_tolerance="${position_tolerance}" \
    -v orientation_tolerance="${orientation_tolerance}" 'BEGIN {
      dx=x-expected_x; dy=y-expected_y; dz=z-expected_z
      distance=sqrt(dx*dx+dy*dy+dz*dz)
      norm=sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
      if (!(norm > 0.99 && norm < 1.01)) exit 1
      expected_zq=sin(expected_yaw/2.0); expected_wq=cos(expected_yaw/2.0)
      dot=(qz*expected_zq+qw*expected_wq)/norm
      if (dot < 0) dot=-dot
      if (dot > 1) dot=1
      angular_error=2.0*atan2(sqrt(1.0-dot*dot), dot)
      exit !(distance <= position_tolerance && angular_error <= orientation_tolerance)
    }' || fail "${label}: green_cube pose is outside its position/orientation gate"
}

cube_stability_gate()
{
  local label="$1" reference current sample_index
  reference="$(cube_pose_row)" || fail "${label}: initial cube pose is unavailable"
  for sample_index in 1 2 3; do
    sleep 0.25
    current="$(cube_pose_row)" || fail "${label}: cube sample ${sample_index} is unavailable"
    awk -v expected_id="${cube_id}" -v first="${reference}" -v second="${current}" 'BEGIN {
      split(first, a, "\t"); split(second, b, "\t")
      if (a[1] != expected_id || b[1] != expected_id) exit 1
      dx=a[2]-b[2]; dy=a[3]-b[3]; dz=a[4]-b[4]
      distance=sqrt(dx*dx+dy*dy+dz*dz)
      an=sqrt(a[5]*a[5]+a[6]*a[6]+a[7]*a[7]+a[8]*a[8])
      bn=sqrt(b[5]*b[5]+b[6]*b[6]+b[7]*b[7]+b[8]*b[8])
      if (!(an > 0.99 && an < 1.01 && bn > 0.99 && bn < 1.01)) exit 1
      dot=(a[5]*b[5]+a[6]*b[6]+a[7]*b[7]+a[8]*b[8])/(an*bn)
      if (dot < 0) dot=-dot
      if (dot > 1) dot=1
      angular_change=2.0*atan2(sqrt(1.0-dot*dot), dot)
      exit !(distance <= 0.003 && angular_change <= 0.034906585039887)
    }' || fail "${label}: green_cube moved during the three-sample gate"
  done
}

read_robot_joint_rows()
{
  local attempt rows
  for attempt in 1 2 3; do
    if rows="$(timeout 5 "${ros_snapshot_tool}" robot --timeout 3)"; then
      printf '%s\n' "${rows}"
      return 0
    fi
  done
  return 1
}

asset_left_arm_endpoint_gate()
{
  local boundary="$1" label="$2" live_rows="$3"
  local endpoint_rows joint expected actual count=0
  local -A seen=()
  endpoint_rows="$(jq -er --argjson boundary "${boundary}" '
    [.payload.messages[0:$boundary][] |
      select(.topic == "/left_arm_controller/joint_trajectory")] as $arm |
    if ($arm | length) < 1 then error("no left-arm endpoint") else $arm[-1] end |
    . as $message | ($message.points[-1].positions) as $positions |
    if (($message.joint_names | length) == 7 and ($positions | length) == 7)
    then range(0; 7) as $index |
      [$message.joint_names[$index], $positions[$index]] | @tsv
    else error("malformed left-arm endpoint") end
  ' "${cube_asset}")" || fail "${label}: sealed left-arm endpoint is unavailable"
  [[ -n "${live_rows}" ]] || fail "${label}: live robot telemetry is malformed"
  while IFS=$'\t' read -r joint expected; do
    case "${joint}" in
      left_shoulder_pitch_joint|left_shoulder_roll_joint|left_shoulder_yaw_joint|\
      left_elbow_joint|left_wrist_roll_joint|left_wrist_pitch_joint|\
      left_wrist_yaw_joint)
        ;;
      *)
        fail "${label}: sealed left-arm endpoint contains an unexpected joint"
        ;;
    esac
    [[ -z "${seen[${joint}]:-}" ]] ||
      fail "${label}: sealed left-arm endpoint duplicates ${joint}"
    seen["${joint}"]=1
    actual="$(awk -v joint="${joint}" '$1==joint {print $2; found++}
      END {if(found!=1) exit 1}' <<<"${live_rows}")" ||
      fail "${label}: live joint ${joint} is missing or duplicated"
    awk -v actual="${actual}" -v expected="${expected}" \
      -v tolerance="${arm_endpoint_tolerance}" 'BEGIN {
      error=actual-expected; if (error < 0) error=-error
      exit !(error <= tolerance)
    }' || fail "${label}: live ${joint} does not match the sealed endpoint"
    ((count+=1))
  done <<<"${endpoint_rows}"
  (( count == 7 )) || fail "${label}: sealed endpoint does not contain seven joints"
}

asset_left_finger_endpoint_gate()
{
  # The close command deliberately asks for preload beyond the rigid cube.
  # Compare live feedback with the capture-state sealed contact endpoint, not
  # with the unreachable desired trajectory endpoint.
  local boundary="$1" label="$2" live_rows="$3"
  local endpoint_rows joint expected actual count=0
  local -A seen=()
  endpoint_rows="$(jq -er --argjson boundary "${boundary}" '
    [.payload.messages[0:$boundary][] |
      select(.topic == "/left_gripper_controller/joint_trajectory")] as $gripper |
    if ($gripper | length) < 1 then error("no left-gripper endpoint")
    else $gripper[-1] end |
    . as $message |
    (if $message.expected_actual_endpoint.kind != "cube_gripper_contact_close"
     then error("sealed close message lacks the scoped physical endpoint")
     else $message.expected_actual_endpoint.positions end) as $positions |
    if (($message.joint_names | length) == 2 and ($positions | length) == 2)
    then range(0; 2) as $index |
      [$message.joint_names[$index], $positions[$index]] | @tsv
    else error("malformed left-gripper endpoint") end
  ' "${cube_asset}")" || fail "${label}: sealed left-finger endpoint is unavailable"
  [[ -n "${live_rows}" ]] || fail "${label}: live finger telemetry is malformed"
  while IFS=$'\t' read -r joint expected; do
    [[ ( "${joint}" == "left_dex1_finger_1_joint" ||
         "${joint}" == "left_dex1_finger_2_joint" ) &&
       -z "${seen[${joint}]:-}" ]] ||
      fail "${label}: sealed left-finger joint set is invalid"
    seen["${joint}"]=1
    actual="$(awk -v joint="${joint}" '$1==joint {print $2; found++}
      END {if(found!=1) exit 1}' <<<"${live_rows}")" ||
      fail "${label}: live joint ${joint} is missing or duplicated"
    awk -v actual="${actual}" -v expected="${expected}" 'BEGIN {
      error=actual-expected; if (error < 0) error=-error
      exit !(error <= 0.003)
    }' || fail "${label}: live ${joint} does not match the sealed close endpoint"
    ((count+=1))
  done <<<"${endpoint_rows}"
  (( count == 2 )) || fail "${label}: sealed endpoint does not contain two fingers"
}

left_contact_gate()
{
  local rows="$1" q1 q2
  [[ -n "${rows}" ]] || fail "shared robot joint snapshot is empty"
  q1="$(awk '$1=="left_dex1_finger_1_joint" {print $2; found++} END {if(found!=1) exit 1}' <<<"${rows}")" ||
    fail "left finger 1 feedback is missing"
  q2="$(awk '$1=="left_dex1_finger_2_joint" {print $2; found++} END {if(found!=1) exit 1}' <<<"${rows}")" ||
    fail "left finger 2 feedback is missing"
  awk -v q1="${q1}" -v q2="${q2}" 'BEGIN {
    gap = 0.050061 - (q1 + q2)
    center = (q2 - q1) / 2.0
    exit !(q1 >= -0.0195 && q1 <= -0.0100 &&
           q2 >= -0.0195 && q2 <= -0.0100 &&
           gap >= 0.077 && gap <= 0.083 &&
           center >= -0.003 && center <= 0.003)
  }' || fail "left fingers do not certify symmetric contact on the 80 mm cube"
}

temporary_dir="$(mktemp -d /tmp/h1_cube_support_8251.XXXXXX)"
robot_joint_rows_file="${temporary_dir}/robot_joint_rows.txt"
ownership_dir="/tmp/h1_cube_support_8251_owner_${EUID}"
if [[ ! -e "${ownership_dir}" && ! -L "${ownership_dir}" ]]; then
  mkdir -m 0700 -- "${ownership_dir}" || fail "could not create ownership directory"
fi
[[ -d "${ownership_dir}" && ! -L "${ownership_dir}" ]] ||
  fail "ownership directory is not a real directory"
ownership_metadata="$(stat -c '%u %a' -- "${ownership_dir}")" ||
  fail "could not inspect ownership directory"
[[ "${ownership_metadata}" == "${EUID} 700" ]] ||
  fail "ownership directory must be owned by this user with mode 0700"
if [[ "${operation}" == "recover_attach" ||
  "${operation}" == "recover_detach" ]]; then
  [[ "${recovery_preload_file}" == /* && -f "${recovery_preload_file}" &&
    ! -L "${recovery_preload_file}" ]] ||
    fail "H1_CUBE_RECOVERY_PRELOAD_FILE must name an absolute, non-symlink regular file"
  recovery_preload_real="$(readlink -f -- "${recovery_preload_file}")" ||
    fail "cube recovery preload evidence cannot be resolved"
  [[ -n "${recovery_preload_real}" &&
    "$(dirname -- "${recovery_preload_real}")" == "${ownership_dir}" ]] ||
    fail "cube recovery preload evidence is outside the ownership directory"
  recovery_preload_name="$(basename -- "${recovery_preload_real}")"
  recovery_pattern="^${cube_asset_sha256}_${cube_id}_([1-9][0-9]*)_([1-9][0-9]*)\\.preload$"
  [[ "${recovery_preload_name}" =~ ${recovery_pattern} ]] ||
    fail "cube recovery preload evidence name is invalid"
  parent_pid="${BASH_REMATCH[1]}"
  parent_start_ticks="${BASH_REMATCH[2]}"
  if [[ -r "/proc/${parent_pid}/stat" ]]; then
    live_parent_start_ticks="$(
      sed 's/^[^)]*) //' "/proc/${parent_pid}/stat" 2>/dev/null |
        awk '{print $20}'
    )" || fail "could not inspect prior cube support parent"
    [[ "${live_parent_start_ticks}" != "${parent_start_ticks}" ]] ||
      fail "prior cube support parent is still active"
  fi
fi
preload_file="${ownership_dir}/${cube_asset_sha256}_${cube_id}_${parent_pid}_${parent_start_ticks}.preload"
ownership_file="${ownership_dir}/${cube_asset_sha256}_${cube_id}_${parent_pid}_${parent_start_ticks}.owner"
if [[ "${operation}" == "recover_attach" ||
  "${operation}" == "recover_detach" ]]; then
  [[ "${recovery_preload_real}" == "${preload_file}" ]] ||
    fail "cube recovery preload evidence identity mismatch"
fi
listener_pid=""
cleanup()
{
  if [[ -n "${listener_pid}" ]] && kill -0 "${listener_pid}" 2>/dev/null; then
    kill -TERM "${listener_pid}" 2>/dev/null || true
    wait "${listener_pid}" 2>/dev/null || true
  fi
  rm -f -- "${temporary_dir}/state.txt" "${temporary_dir}/preload.tmp" \
    "${temporary_dir}/owner.tmp" \
    "${temporary_dir}/state.info" "${temporary_dir}/attach.info" \
    "${temporary_dir}/detach.info" "${robot_joint_rows_file}"
  rmdir "${temporary_dir}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

preload_record()
{
  printf '%s\n' \
    'schema=h1_cube_support_preload/v1' \
    "cube_entity_id=${cube_id}" \
    "cube_asset=${cube_asset_real}" \
    "cube_asset_sha256=${cube_asset_sha256}" \
    "attach_after_index=${attach_after_index}" \
    "detach_after_index=${detach_after_index}" \
    'initially_attached=false' \
    "parent_pid=${parent_pid}" \
    "parent_start_ticks=${parent_start_ticks}"
}

create_preload_evidence()
{
  [[ ! -e "${preload_file}" && ! -L "${preload_file}" ]] ||
    fail "same-run preload evidence already exists"
  (umask 077; preload_record >"${temporary_dir}/preload.tmp") ||
    fail "could not create preload evidence payload"
  chmod 0400 "${temporary_dir}/preload.tmp" || fail "could not seal preload evidence"
  ln -- "${temporary_dir}/preload.tmp" "${preload_file}" ||
    fail "exclusive preload evidence create failed"
  rm -f -- "${temporary_dir}/preload.tmp"
}

require_preload_evidence()
{
  [[ -f "${preload_file}" && ! -L "${preload_file}" ]] ||
    fail "same-run preload evidence is absent"
  local metadata actual expected
  metadata="$(stat -c '%u %a %h' -- "${preload_file}")" ||
    fail "could not inspect preload evidence"
  [[ "${metadata}" == "${EUID} 400 1" ]] ||
    fail "preload evidence metadata is invalid"
  actual="$(<"${preload_file}")"
  expected="$(preload_record)"
  [[ "${actual}" == "${expected}" ]] || fail "preload evidence content is invalid"
}

ownership_record()
{
  printf '%s\n' \
    'schema=h1_cube_support_owner/v1' \
    "cube_entity_id=${cube_id}" \
    "cube_asset=${cube_asset_real}" \
    "cube_asset_sha256=${cube_asset_sha256}" \
    "attach_after_index=${attach_after_index}" \
    "detach_after_index=${detach_after_index}" \
    "parent_pid=${parent_pid}" \
    "parent_start_ticks=${parent_start_ticks}"
}

create_ownership_evidence()
{
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run ownership evidence already exists"
  (umask 077; ownership_record >"${temporary_dir}/owner.tmp") ||
    fail "could not create ownership evidence payload"
  chmod 0400 "${temporary_dir}/owner.tmp" || fail "could not seal ownership evidence"
  ln -- "${temporary_dir}/owner.tmp" "${ownership_file}" ||
    fail "exclusive ownership evidence create failed"
  rm -f -- "${temporary_dir}/owner.tmp"
}

require_ownership_evidence()
{
  [[ -f "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run attach ownership evidence is absent"
  local metadata actual expected
  metadata="$(stat -c '%u %a %h' -- "${ownership_file}")" ||
    fail "could not inspect ownership evidence"
  [[ "${metadata}" == "${EUID} 400 1" ]] ||
    fail "ownership evidence metadata is invalid"
  actual="$(<"${ownership_file}")"
  expected="$(ownership_record)"
  [[ "${actual}" == "${expected}" ]] || fail "ownership evidence content is invalid"
}

transport_info()
{
  timeout 5 gz topic -i -t "$1" 2>&1
}

transport_endpoint_count()
{
  awk '/^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// {count++}
    END {print count+0}' <<<"$1"
}

transport_type_count()
{
  local info="$1" type="$2"
  awk -F', ' -v type="${type}" '
    /^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// && $2 == type {count++}
    END {print count+0}' <<<"${info}"
}

read_plugin_transport_infos()
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

require_empty_plugin_transport_topics()
{
  local index topic info
  local -a topics=("${state_topic}" "${attach_topic}" "${detach_topic}")
  local -a files=("${temporary_dir}/state.info" "${temporary_dir}/attach.info"
    "${temporary_dir}/detach.info")
  read_plugin_transport_infos || fail "could not inspect cube-support transport topics"
  for index in "${!topics[@]}"; do
    topic="${topics[${index}]}"
    info="$(<"${files[${index}]}")"
    grep -Fqx "No publishers on topic [${topic}]" <<<"${info}" &&
      grep -Fqx "No subscribers on topic [${topic}]" <<<"${info}" &&
      [[ "$(transport_endpoint_count "${info}")" == 0 ]] ||
      fail "unexpected resident endpoint on ${topic}"
  done
}

require_add_service_contract()
{
  local info
  info="$(timeout 5 gz service -i -s "${add_service}" 2>&1)" ||
    fail "could not inspect ${add_service}"
  grep -Fqx 'Service providers [Address, Request Message Type, Response Message Type]:' \
    <<<"${info}" || fail "add-service discovery header is malformed"
  awk -F', ' '
    /^[[:space:]]+[[:alpha:]][[:alnum:]+.-]*:\/\// {
      providers++
      if ($2 != "gz.msgs.EntityPlugin_V" || $3 != "gz.msgs.Boolean") bad=1
    }
    END {exit !(providers == 1 && !bad)}
  ' <<<"${info}" ||
    fail "add service must have exactly one EntityPlugin_V/Boolean provider"
}

require_single_resident_plugin()
{
  local state_info attach_info detach_info info_and_topic
  read_plugin_transport_infos || fail "resident plugin topic inspection failed"
  state_info="$(<"${temporary_dir}/state.info")"
  attach_info="$(<"${temporary_dir}/attach.info")"
  detach_info="$(<"${temporary_dir}/detach.info")"
  grep -Fq 'Publishers [Address, Message Type]:' <<<"${state_info}" &&
    grep -Fqx "No subscribers on topic [${state_topic}]" <<<"${state_info}" &&
    [[ "$(transport_endpoint_count "${state_info}")" == 1 ]] &&
    [[ "$(transport_type_count "${state_info}" gz.msgs.StringMsg)" == 1 ]] ||
    fail "state topic is not owned by exactly one StringMsg publisher"
  for info_and_topic in "${attach_info}|${attach_topic}" "${detach_info}|${detach_topic}"; do
    local info="${info_and_topic%|*}" topic="${info_and_topic##*|}"
    grep -Fqx "No publishers on topic [${topic}]" <<<"${info}" &&
      grep -Fq 'Subscribers [Address, Message Type]:' <<<"${info}" &&
      [[ "$(transport_endpoint_count "${info}")" == 1 ]] &&
      [[ "$(transport_type_count "${info}" gz.msgs.Empty)" == 1 ]] ||
      fail "${topic} is not owned by exactly one Empty subscriber"
  done
}

require_resident_plugin_with_listener()
{
  local state_info attach_info detach_info info_and_topic
  [[ -n "${listener_pid}" ]] && kill -0 "${listener_pid}" 2>/dev/null ||
    fail "state listener is not alive for resident contract"
  read_plugin_transport_infos || fail "resident plugin topic inspection failed"
  state_info="$(<"${temporary_dir}/state.info")"
  attach_info="$(<"${temporary_dir}/attach.info")"
  detach_info="$(<"${temporary_dir}/detach.info")"
  grep -Fq 'Publishers [Address, Message Type]:' <<<"${state_info}" &&
    grep -Fq 'Subscribers [Address, Message Type]:' <<<"${state_info}" &&
    [[ "$(transport_endpoint_count "${state_info}")" == 2 ]] &&
    [[ "$(transport_type_count "${state_info}" gz.msgs.StringMsg)" == 1 ]] &&
    [[ "$(transport_type_count "${state_info}" google.protobuf.Message)" == 1 ]] ||
    fail "state topic is not one StringMsg publisher plus the verified listener"
  for info_and_topic in "${attach_info}|${attach_topic}" "${detach_info}|${detach_topic}"; do
    local info="${info_and_topic%|*}" topic="${info_and_topic##*|}"
    grep -Fqx "No publishers on topic [${topic}]" <<<"${info}" &&
      grep -Fq 'Subscribers [Address, Message Type]:' <<<"${info}" &&
      [[ "$(transport_endpoint_count "${info}")" == 1 ]] &&
      [[ "$(transport_type_count "${info}" gz.msgs.Empty)" == 1 ]] ||
      fail "${topic} is not owned by exactly one Empty subscriber"
  done
}

listener_is_discovered()
{
  local mode="$1" info endpoints state_types listener_types
  info="$(transport_info "${state_topic}" 2>/dev/null || true)"
  endpoints="$(transport_endpoint_count "${info}")"
  state_types="$(transport_type_count "${info}" gz.msgs.StringMsg)"
  listener_types="$(transport_type_count "${info}" google.protobuf.Message)"
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
    kill -0 "${listener_pid}" 2>/dev/null || fail "state listener exited during discovery"
    if listener_is_discovered "${mode}"; then
      return 0
    fi
    sleep 0.05
  done
  fail "state listener was not discovered with the exact transport type/count"
}

start_state_listener()
{
  [[ -z "${listener_pid}" ]] || fail "state listener is already running"
  : >"${temporary_dir}/state.txt"
  timeout 20 gz topic -e -t "${state_topic}" >"${temporary_dir}/state.txt" 2>&1 &
  listener_pid=$!
}

listen_for_state()
{
  local mode="$1"
  start_state_listener
  wait_for_listener_discovery "${mode}"
}

require_state()
{
  local expected="$1" attempt
  for attempt in {1..160}; do
    if grep -Eq "^[[:space:]]*data:[[:space:]]*['\"]?${expected}['\"]?[[:space:]]*$" \
      "${temporary_dir}/state.txt"
    then
      kill -TERM "${listener_pid}" 2>/dev/null || true
      wait "${listener_pid}" 2>/dev/null || true
      listener_pid=""
      return 0
    fi
    kill -0 "${listener_pid}" 2>/dev/null || break
    sleep 0.05
  done
  if [[ -n "${listener_pid}" ]]; then
    kill -TERM "${listener_pid}" 2>/dev/null || true
    wait "${listener_pid}" 2>/dev/null || true
  fi
  listener_pid=""
  fail "support state did not report fresh ${expected} confirmation"
}

wait_for_observed_state()
{
  # Fast resident mode replaces repeated endpoint discovery with an actual
  # heartbeat from the already-preloaded plugin.  Keep the listener alive so
  # require_state() can still demand the opposite, fresh transition ack after
  # the attach/detach publication.
  local expected="$1" attempt
  for attempt in {1..160}; do
    if grep -Eq "^[[:space:]]*data:[[:space:]]*['\"]?${expected}['\"]?[[:space:]]*$" \
      "${temporary_dir}/state.txt"
    then
      return 0
    fi
    kill -0 "${listener_pid}" 2>/dev/null || break
    sleep 0.025
  done
  fail "resident support did not provide a fresh ${expected} heartbeat"
}

run_preload_static_preflight()
{
  local transport_pid service_pid transport_status=0 service_status=0
  (
    trap - EXIT INT TERM HUP
    require_empty_plugin_transport_topics
  ) &
  transport_pid=$!
  (
    trap - EXIT INT TERM HUP
    require_add_service_contract
  ) &
  service_pid=$!
  wait "${transport_pid}" || transport_status=1
  wait "${service_pid}" || service_status=1
  (( transport_status == 0 )) ||
    fail "initial cube-support transport topic contract failed"
  (( service_status == 0 )) || fail "add-service contract failed"
}

run_attach_dynamic_preflight()
{
  local transport_pid="" pose_pid robot_pid
  local transport_status=0 pose_status=0 robot_status=0
  if [[ "${fast_resident}" == "0" ]]; then
    (
      trap - EXIT INT TERM HUP
      require_resident_plugin_with_listener
    ) &
    transport_pid=$!
  fi
  (
    trap - EXIT INT TERM HUP
    cube_pose_gate 0.334 0.377 0.830 1.570796326794897 0.003 \
      0.034906585039887 "attach initial cube pose gate"
  ) &
  pose_pid=$!
  (
    trap - EXIT INT TERM HUP
    read_robot_joint_rows >"${robot_joint_rows_file}"
  ) &
  robot_pid=$!
  if [[ -n "${transport_pid}" ]]; then
    wait "${transport_pid}" || transport_status=1
  fi
  wait "${pose_pid}" || pose_status=1
  wait "${robot_pid}" || robot_status=1
  (( transport_status == 0 )) || fail "preloaded resident plugin contract failed"
  (( pose_status == 0 )) || fail "attach initial cube pose gate failed"
  (( robot_status == 0 )) && [[ -s "${robot_joint_rows_file}" ]] ||
    fail "no fresh /joint_states before support attach"
}

run_detach_dynamic_preflight()
{
  local transport_pid="" pose_pid robot_pid
  local transport_status=0 pose_status=0 robot_status=0
  if [[ "${fast_resident}" == "0" ]]; then
    (
      trap - EXIT INT TERM HUP
      require_resident_plugin_with_listener
    ) &
    transport_pid=$!
  fi
  (
    trap - EXIT INT TERM HUP
    cube_pose_gate 0.610890542898 -0.029527393333 0.995407082351 \
      "${rack_expected_yaw_rad}" "${rack_position_tolerance}" \
      "${rack_orientation_tolerance}" "detach rack gate"
    cube_stability_gate "detach rack stability gate"
  ) &
  pose_pid=$!
  (
    trap - EXIT INT TERM HUP
    read_robot_joint_rows >"${robot_joint_rows_file}"
  ) &
  robot_pid=$!
  if [[ -n "${transport_pid}" ]]; then
    wait "${transport_pid}" || transport_status=1
  fi
  wait "${pose_pid}" || pose_status=1
  wait "${robot_pid}" || robot_status=1
  (( transport_status == 0 )) || fail "resident plugin contract failed before detach"
  (( pose_status == 0 )) || fail "rack pose/stability contract failed before detach"
  (( robot_status == 0 )) && [[ -s "${robot_joint_rows_file}" ]] ||
    fail "no fresh /joint_states before support detach"
}

if [[ "${operation}" == "preload" ]]; then
  [[ ! -e "${preload_file}" && ! -L "${preload_file}" ]] ||
    fail "same-run preload evidence already exists before preload"
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run ownership evidence already exists before preload"
  run_preload_static_preflight
  listen_for_state before_add
  request="entity {id: ${cube_id}, name: \"green_cube\", type: MODEL} plugins {name: \"h1_2::GazeboCompliantGraspSystem\", filename: \"${plugin_so}\", innerxml: \"<parent_scope>h1_2::left_wrist_yaw_link</parent_scope><initially_attached>false</initially_attached><detach_topic>${detach_topic}</detach_topic><attach_topic>${attach_topic}</attach_topic><output_topic>${state_topic}</output_topic>\"}"
  response="$(timeout 12 gz service -s "${add_service}" \
    --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 10000 \
    --req "${request}" 2>&1)" || fail "support plugin preload add service failed"
  grep -Eq '^[[:space:]]*data:[[:space:]]*true[[:space:]]*$' <<<"${response}" ||
    fail "support plugin preload add was rejected"
  require_resident_plugin_with_listener
  require_state detached
  require_single_resident_plugin
  create_preload_evidence
  printf 'CUBE_SUPPORT_PRELOAD=PASS\n'
elif [[ "${operation}" == "attach" ||
  "${operation}" == "recover_attach" ]]; then
  require_preload_evidence
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run ownership evidence already exists before attach"
  start_state_listener
  # Discovery of the short-lived verification listener is asynchronous in
  # Gazebo Transport.  Do not let the exact endpoint-count audit race the
  # listener advertisement: wait for the one resident publisher plus this
  # listener before launching the remaining independent preflight gates.
  if [[ "${fast_resident}" == "1" ]]; then
    wait_for_observed_state detached
    printf 'CUBE_SUPPORT_FAST_RESIDENT_BASELINE=PASS operation=attach state=detached\n'
  else
    wait_for_listener_discovery after_add
  fi
  run_attach_dynamic_preflight
  robot_joint_rows="$(<"${robot_joint_rows_file}")"
  asset_left_arm_endpoint_gate "${attach_after_index}" \
    "attach sealed contact endpoint" "${robot_joint_rows}"
  asset_left_finger_endpoint_gate "${attach_after_index}" \
    "attach sealed finger endpoint" "${robot_joint_rows}"
  left_contact_gate "${robot_joint_rows}"
  timeout 5 gz topic -t "${attach_topic}" -m gz.msgs.Empty -p '' >/dev/null 2>&1 ||
    fail "support attach publication failed"
  require_state attached
  create_ownership_evidence
  printf 'CUBE_SUPPORT_HOOK=PASS operation=attach\n'
else
  require_preload_evidence
  require_ownership_evidence
  start_state_listener
  # The detach path needs the same bounded discovery barrier as attach.  A
  # missing listener endpoint is a transport race, not authority to release
  # the supported cube, so remain fail-closed until the exact topology is
  # visible and then repeat the full dynamic contract below.
  if [[ "${fast_resident}" == "1" ]]; then
    wait_for_observed_state attached
    printf 'CUBE_SUPPORT_FAST_RESIDENT_BASELINE=PASS operation=detach state=attached\n'
  else
    wait_for_listener_discovery after_add
  fi
  run_detach_dynamic_preflight
  robot_joint_rows="$(<"${robot_joint_rows_file}")"
  asset_left_arm_endpoint_gate "${detach_after_index}" \
    "detach sealed support endpoint" "${robot_joint_rows}"
  asset_left_finger_endpoint_gate "${detach_after_index}" \
    "detach sealed finger endpoint" "${robot_joint_rows}"
  left_contact_gate "${robot_joint_rows}"
  timeout 5 gz topic -t "${detach_topic}" -m gz.msgs.Empty -p '' >/dev/null 2>&1 ||
    fail "support detach publication failed"
  require_state detached
  rm -f -- "${ownership_file}" "${preload_file}" ||
    fail "could not consume same-run support evidence"
  [[ ! -e "${ownership_file}" && ! -L "${ownership_file}" ]] ||
    fail "same-run ownership evidence was not consumed"
  [[ ! -e "${preload_file}" && ! -L "${preload_file}" ]] ||
    fail "same-run preload evidence was not consumed"
  printf 'CUBE_SUPPORT_HOOK=PASS operation=detach\n'
fi
