#!/usr/bin/env bash
# Read one fresh, atomically published 8254 joint snapshot without joining ROS.
set -Eeuo pipefail
export LC_ALL=C

readonly mode="${1:-}"
shift || true
timeout_seconds="3.0"
if (( $# == 2 )) && [[ "$1" == "--timeout" ]]; then
  timeout_seconds="$2"
  shift 2
fi
if (( $# != 0 )) || [[ "${mode}" != "robot" && "${mode}" != "incubator" ]]; then
  printf 'usage: h1_ros_snapshot_cache_8254.sh robot|incubator [--timeout N]\n' >&2
  exit 2
fi

fail()
{
  printf 'h1_ros_snapshot_cache_8254: ERROR: %s\n' "$*" >&2
  return 1
}

is_bounded_duration()
{
  local value="$1"
  [[ "${value}" =~ ^(0|[1-9][0-9]*)(\.[0-9]+)?$ ]] &&
    awk -v value="${value}" 'BEGIN {exit !(value >= 0.1 && value <= 10.0)}'
}

is_bounded_duration "${timeout_seconds}" ||
  fail "--timeout must be finite and in [0.1, 10.0]"
readonly timeout_seconds

readonly current_uid="$(id -u)"
readonly runtime_dir="${XDG_RUNTIME_DIR:-/run/user/${current_uid}}"
readonly cache_dir="${H1_ROS_SNAPSHOT_CACHE_DIR:-${runtime_dir}/h1_ros_snapshot_8254}"
readonly maximum_age_seconds="${H1_ROS_SNAPSHOT_CACHE_MAX_AGE:-1.0}"
[[ "${cache_dir}" == /* && "${cache_dir}" != *$'\n'* ]] ||
  fail "cache directory must be an absolute single-line path"
is_bounded_duration "${maximum_age_seconds}" ||
  fail "H1_ROS_SNAPSHOT_CACHE_MAX_AGE must be finite and in [0.1, 10.0]"
readonly maximum_age_seconds
readonly cache_file="${cache_dir}/${mode}.cache"
readonly ready_file="${cache_dir}/ready"

validate_cache_directory()
{
  [[ -d "${cache_dir}" && ! -L "${cache_dir}" ]] || {
    fail "cache directory is absent, not a directory, or a symlink"
    return 1
  }
  local owner permissions kind
  IFS='|' read -r owner permissions kind < <(
    stat -c '%u|%a|%F' -- "${cache_dir}"
  ) || {
    fail "cannot inspect cache directory"
    return 1
  }
  [[ "${owner}" == "${current_uid}" && "${permissions}" == "700" &&
    "${kind}" == "directory" ]] || {
    fail "cache directory must be owned by the caller with exact mode 0700"
    return 1
  }
}

# stdout is reserved for validated file content.  Return 2 only for a normal
# atomic replacement/missing-file race that the caller may retry.
secure_read_file()
{
  local path="$1" label="$2"
  [[ -e "${path}" ]] || return 2
  [[ -f "${path}" && ! -L "${path}" ]] || {
    fail "${label} must be a non-symlink regular file"
    return 1
  }
  local before device inode owner permissions links kind size
  before="$(stat -c '%d|%i|%u|%a|%h|%F|%s' -- "${path}")" || return 2
  IFS='|' read -r device inode owner permissions links kind size <<<"${before}"
  [[ "${owner}" == "${current_uid}" && "${permissions}" == "400" &&
    "${links}" == "1" && "${kind}" == "regular file" &&
    "${size}" =~ ^[1-9][0-9]*$ && "${size}" -le 262144 ]] || {
    fail "${label} must be owned, mode 0400, link-count 1, regular, and bounded"
    return 1
  }
  local descriptor after
  exec {descriptor}<"${path}" || return 2
  after="$(stat -Lc '%d|%i|%u|%a|%h|%F|%s' -- "/proc/self/fd/${descriptor}")" || {
    exec {descriptor}<&-
    return 2
  }
  if [[ "${before}" != "${after}" ]]; then
    exec {descriptor}<&-
    return 2
  fi
  cat <&"${descriptor}"
  exec {descriptor}<&-
}

monotonic_now_ns()
{
  /usr/bin/python3 -c 'import time; print(time.monotonic_ns())'
}

read_once()
{
  validate_cache_directory || return 1
  local ready_content cache_content status
  set +e
  ready_content="$(secure_read_file "${ready_file}" ready-marker)"
  status=$?
  set -e
  (( status == 0 )) || return "${status}"
  set +e
  cache_content="$(secure_read_file "${cache_file}" "${mode}-cache")"
  status=$?
  set -e
  (( status == 0 )) || return "${status}"

  local -a ready_lines cache_lines
  mapfile -t ready_lines <<<"${ready_content}"
  mapfile -t cache_lines <<<"${cache_content}"
  (( ${#ready_lines[@]} == 1 && ${#cache_lines[@]} >= 2 )) || {
    fail "cache or ready marker has malformed line cardinality"
    return 1
  }

  local ready_magic ready_version ready_state ready_pid ready_ns ready_extra
  read -r ready_magic ready_version ready_state ready_pid ready_ns ready_extra \
    <<<"${ready_lines[0]}"
  [[ "${ready_magic}" == "H1_ROS_SNAPSHOT_MONITOR_8254" &&
    "${ready_version}" == "v1" && -z "${ready_extra:-}" &&
    "${ready_pid}" =~ ^[1-9][0-9]*$ && "${ready_ns}" =~ ^[1-9][0-9]*$ ]] || {
    fail "ready marker header is malformed"
    return 1
  }
  [[ "${ready_state}" == "READY" ]] || return 2

  local magic version cached_mode cache_pid cache_ns row_count cache_extra
  read -r magic version cached_mode cache_pid cache_ns row_count cache_extra \
    <<<"${cache_lines[0]}"
  [[ "${magic}" == "H1_ROS_SNAPSHOT_CACHE_8254" &&
    "${version}" == "v1" && "${cached_mode}" == "${mode}" &&
    -z "${cache_extra:-}" && "${cache_pid}" =~ ^[1-9][0-9]*$ &&
    "${cache_ns}" =~ ^[1-9][0-9]*$ && "${row_count}" =~ ^[1-9][0-9]*$ &&
    "${row_count}" -le 512 ]] || {
    fail "${mode} cache header is malformed"
    return 1
  }
  [[ "${cache_pid}" == "${ready_pid}" ]] || return 2
  (( ${#cache_lines[@]} == row_count + 1 )) || {
    fail "${mode} cache row count does not match its header"
    return 1
  }

  local now_ns maximum_age_ns
  now_ns="$(monotonic_now_ns)"
  [[ "${now_ns}" =~ ^[1-9][0-9]*$ ]] || {
    fail "could not read monotonic time"
    return 1
  }
  maximum_age_ns="$(awk -v value="${maximum_age_seconds}" \
    'BEGIN {printf "%.0f", value * 1000000000.0}')"
  (( cache_ns <= now_ns && now_ns - cache_ns <= maximum_age_ns )) || return 2

  local rows_output
  rows_output="$(printf '%s\n' "${cache_lines[@]:1}" | awk \
    -v expected_mode="${mode}" -v expected_count="${row_count}" '
      BEGIN {
        number="^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$"
        expected_fields=(expected_mode == "robot" ? 2 : 3)
      }
      {
        if (NF != expected_fields || $1 == "" || seen[$1]++) bad=1
        for (field=2; field<=NF; ++field) {
          if ($field !~ number) bad=1
          value=$field+0.0
          if (value != value || value > 1e100 || value < -1e100) bad=1
        }
        rows[NR]=$0
      }
      END {
        if (bad || NR != expected_count) exit 1
        for (row_index=1; row_index<=NR; ++row_index) print rows[row_index]
      }
    ')" || {
    fail "${mode} cache joint rows are malformed"
    return 1
  }
  printf '%s\n' "${rows_output}"
}

readonly start_ns="$(monotonic_now_ns)"
readonly timeout_ns="$(awk -v value="${timeout_seconds}" \
  'BEGIN {printf "%.0f", value * 1000000000.0}')"
while true; do
  set +e
  output="$(read_once)"
  status=$?
  set -e
  if (( status == 0 )); then
    printf '%s\n' "${output}"
    exit 0
  fi
  (( status == 2 )) || exit 1
  now_ns="$(monotonic_now_ns)"
  if (( now_ns - start_ns >= timeout_ns )); then
    fail "timed out waiting for a fresh, secure ${mode} cache"
    exit 1
  fi
  sleep 0.02
done
