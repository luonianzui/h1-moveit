#!/usr/bin/env bash
set -Eeuo pipefail

isaaclab_root="/home/liu/robot/gym_sim/IsaacLab-3.0-beta2"
project_root="/home/liu/test/h1_2_isaaclab"
log_root="${project_root}/train_logs"
experiment_root="${isaaclab_root}/logs/rsl_rl/h1_2_human_omni_recover_stop_gait_v12"
bootstrap_name="bootstrap_focused_stop_v11_r2_model_150"
bootstrap_target="${isaaclab_root}/logs/rsl_rl/h1_2_human_omni_focused_stop_gait_v11/2026-08-10_17-37-19_omni_focused_stop_v11_v8r3m50_101it_8192_r2"
checkpoint_name="${H12_WARM_START_CHECKPOINT:-model_150.pt}"

duration="${H12_TRAIN_DURATION:-16h}"
num_envs="${H12_NUM_ENVS:-8192}"
max_iterations="${H12_MAX_ITERATIONS:-100000}"
run_name="${H12_RUN_NAME:-omni_recover_stop_v12_v11m150_16h_8192}"
warm_start="${H12_WARM_START:-1}"

mkdir -p "${log_root}" "${experiment_root}"
launch_stamp="$(date +%Y-%m-%d_%H-%M-%S)"
launcher_log="${log_root}/${launch_stamp}_${run_name}.log"
status_file="${log_root}/${launch_stamp}_${run_name}.status"

exec > >(tee -a "${launcher_log}") 2>&1

write_status() {
  local state="$1"
  local exit_status="$2"
  printf 'state=%s\nupdated_at=%s\nduration=%s\nnum_envs=%s\nmax_iterations=%s\nrun_name=%s\nwarm_start=%s\ncheckpoint=%s\npid=%s\nexit_status=%s\n' \
    "${state}" "$(date --iso-8601=seconds)" "${duration}" "${num_envs}" \
    "${max_iterations}" "${run_name}" "${warm_start}" "${checkpoint_name}" \
    "$$" "${exit_status}" > "${status_file}"
}

write_status running 0

train_args=(
  train --rl_library rsl_rl
  --external_callback h1_2_isaaclab.register_tasks
  --task H12-Human-Omni-Recover-Stop-Gait-Flat-v12
  --num_envs "${num_envs}"
  --max_iterations "${max_iterations}"
  --run_name "${run_name}"
)

if [[ "${warm_start}" == "1" ]]; then
  if [[ ! -f "${bootstrap_target}/${checkpoint_name}" ]]; then
    echo "[H12-OMNI-RECOVER-STOP] missing V11 checkpoint: ${bootstrap_target}/${checkpoint_name}" >&2
    write_status failed_launch 2
    exit 2
  fi
  bootstrap_link="${experiment_root}/${bootstrap_name}"
  if [[ -e "${bootstrap_link}" && ! -L "${bootstrap_link}" ]]; then
    echo "[H12-OMNI-RECOVER-STOP] refusing to replace non-symlink bootstrap path: ${bootstrap_link}" >&2
    write_status failed_launch 2
    exit 2
  fi
  ln -sfn "${bootstrap_target}" "${bootstrap_link}"
  train_args+=(--resume --load_run "${bootstrap_name}" --checkpoint "${checkpoint_name}")
fi

echo "[H12-OMNI-RECOVER-STOP] launcher_log=${launcher_log}"
echo "[H12-OMNI-RECOVER-STOP] status_file=${status_file}"
echo "[H12-OMNI-RECOVER-STOP] starting V12 with ${num_envs} environments for ${duration}"
echo "[H12-OMNI-RECOVER-STOP] V11 warm start=${bootstrap_target}/${checkpoint_name}"

cd "${isaaclab_root}"
set +e
timeout --signal=INT --kill-after=300s "${duration}" \
  env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV PYTHONUNBUFFERED=1 \
  ./isaaclab.sh "${train_args[@]}" physics=physx --viz none
train_status=$?
set -e

if grep -q '^Traceback (most recent call last):' "${launcher_log}"; then
  echo "[H12-OMNI-RECOVER-STOP] traceback detected despite process status ${train_status}" >&2
  train_status=1
fi

last_progress="$(
  grep -oE 'Learning iteration +[0-9]+/[0-9]+' "${launcher_log}" \
    | tail -n 1 \
    || true
)"
last_iteration="$(sed -E 's/.* ([0-9]+)\/[0-9]+/\1/' <<< "${last_progress}")"
announced_total="$(sed -E 's/.* [0-9]+\/([0-9]+)/\1/' <<< "${last_progress}")"

if [[ ${train_status} -eq 124 ]]; then
  final_state="completed_duration"
elif [[ ${train_status} -eq 0 ]]; then
  if [[ "${last_iteration}" =~ ^[0-9]+$ ]] \
      && [[ "${announced_total}" =~ ^[0-9]+$ ]] \
      && (( last_iteration + 1 >= announced_total )); then
    final_state="completed_iterations"
  else
    final_state="stopped_early"
  fi
else
  final_state="failed"
fi

write_status "${final_state}" "${train_status}"
echo "[H12-OMNI-RECOVER-STOP] finished state=${final_state} train_status=${train_status}"

if [[ ${final_state} == "failed" ]]; then
  exit "${train_status}"
fi
