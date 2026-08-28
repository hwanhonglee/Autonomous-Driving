#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dest="${AUTOWARE_E2E_DATA_PATH:-${root}/data/ml_models}/vad/v0.1"
base_url="https://awf.ml.dev.web.auto/planning/models/tensorrt_vad/carla_tiny/v0.1"

mkdir -p "${dest}"

download() {
  local file="$1"
  local expected_sha256="$2"
  local path="${dest}/${file}"
  local partial="${path}.part"

  if [[ -f "${path}" ]] && echo "${expected_sha256}  ${path}" | sha256sum --check --status; then
    printf '[ok] %s\n' "${file}"
    return
  fi

  rm -f "${path}"
  if ! curl --fail --location --retry 3 --continue-at - \
    --output "${partial}" "${base_url}/${file}"; then
    rm -f "${partial}"
    curl --fail --location --retry 3 \
      --output "${partial}" "${base_url}/${file}"
  fi

  echo "${expected_sha256}  ${partial}" | sha256sum --check --status
  mv "${partial}" "${path}"
}

download vad-carla-tiny_backbone.onnx \
  04b925f2750fd1c4adf16b5aae9c149d0baa39185e99d141232ebe20bddba4da
download vad-carla-tiny_head.onnx \
  31f49a592a764ce82bbe6e26d0bfc99dc8a9613628884dc73dd5e67521ff3e9e
download vad-carla-tiny_head_no_prev.onnx \
  6a89d479e0717b1e526f1aa3a1137c631a9ef854e810d0328a035aae11818c2a
download vad-carla-tiny.param.json \
  03d3187fed3c70f761456afc2e93e18c1765113b1c1580ffa78ab42b10dbd179

echo "VAD v0.1 artifacts are ready in ${dest}"
