#!/usr/bin/env bash
set -euo pipefail

default_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${1:-${default_workspace}}"
description="${workspace}/src/h1_2_description"
model="${description}/urdf/h1_2_model.urdf.xacro"
source_dir="${description}/meshes"
output_dir="${description}/meshes_low"
filter="${workspace}/tools/lowpoly.mlx"
log="${workspace}/log/meshlab-lowpoly.log"

mkdir -p "${output_dir}" "${workspace}/log"
: > "${log}"

grep -o 'package://h1_2_description/meshes/[^"]*' "${model}" |
  sed 's#package://h1_2_description/meshes/##' |
  sort -u |
  while IFS= read -r mesh; do
    echo "Simplifying ${mesh}" | tee -a "${log}"
    QT_QPA_PLATFORM=offscreen meshlabserver \
      -i "${source_dir}/${mesh}" \
      -o "${output_dir}/${mesh}" \
      -s "${filter}" \
      >> "${log}" 2>&1
  done

echo "Low-poly meshes written to ${output_dir}" | tee -a "${log}"
