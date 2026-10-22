#!/usr/bin/env bash
set -eo pipefail
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r353"
cd "${workspace}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:${PATH}"
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
test ! -e "${workspace}/install_lab_scene_48box_sequential_full_20261022_r353"
snapshot_conda() {
  for name in libssl.so libssl.so.3 libcrypto.so libcrypto.so.3 libcurl.so libcurl.so.4; do
    path="/home/liu/miniconda3/lib/${name}"
    if [[ -L "${path}" ]]; then
      printf 'SYMLINK %s %s\n' "${name}" "$(readlink "${path}")"
    elif [[ -f "${path}" ]]; then
      sha256sum "${path}"
    elif [[ ! -e "${path}" ]]; then
      printf 'ABSENT %s\n' "${name}"
    else
      return 1
    fi
  done
}
snapshot_conda > "${evidence}/conda_six_before.txt"
(
  h1_backup_dir="$(mktemp -d /tmp/h1_miniconda_lib_backup.XXXXXX)"
  h1_conda_lib="/home/liu/miniconda3/lib"
  h1_names=(libssl.so libssl.so.3 libcrypto.so libcrypto.so.3 libcurl.so libcurl.so.4)
  h1_restore() {
    for h1_name in "${h1_names[@]}"; do
      if [[ -e "${h1_backup_dir}/${h1_name}" || -L "${h1_backup_dir}/${h1_name}" ]]; then
        mv "${h1_backup_dir}/${h1_name}" "${h1_conda_lib}/${h1_name}"
      fi
    done
    rmdir "${h1_backup_dir}"
  }
  trap h1_restore EXIT INT TERM
  for h1_name in "${h1_names[@]}"; do
    if [[ -e "${h1_conda_lib}/${h1_name}" || -L "${h1_conda_lib}/${h1_name}" ]]; then
      mv "${h1_conda_lib}/${h1_name}" "${h1_backup_dir}/${h1_name}"
    fi
  done
  colcon --log-base "${evidence}/colcon_log" build --base-paths src \
    --build-base "${evidence}/build/unique_prefix" \
    --install-base "${workspace}/install_lab_scene_48box_sequential_full_20261022_r353" \
    --symlink-install --packages-select h1_2_description h1_2_moveit_config \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
)
snapshot_conda > "${evidence}/conda_six_after.txt"
cmp "${evidence}/conda_six_before.txt" "${evidence}/conda_six_after.txt"

