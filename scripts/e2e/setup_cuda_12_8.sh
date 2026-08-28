#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
vendor_root="${AUTOWARE_E2E_CUDA_12_8_ROOT:-${root}/data/vendor/cuda-12.8}"
deb_dir="${vendor_root}/debs"
extract_root="${vendor_root}/root"
toolkit_root="${extract_root}/usr/local/cuda-12.8"

packages=(
  cuda-cccl-12-8
  cuda-crt-12-8
  cuda-cudart-12-8
  cuda-cudart-dev-12-8
  cuda-cuobjdump-12-8
  cuda-cuxxfilt-12-8
  cuda-driver-dev-12-8
  cuda-nvcc-12-8
  cuda-nvprune-12-8
  cuda-nvvm-12-8
)

if [[ -x "${toolkit_root}/bin/nvcc" ]] && \
  "${toolkit_root}/bin/nvcc" --version | grep -q 'release 12\.8'; then
  echo "CUDA 12.8 toolchain already prepared: ${toolkit_root}"
  exit 0
fi

command -v apt-get >/dev/null || {
  echo "apt-get is required to download the NVIDIA CUDA packages" >&2
  exit 1
}
command -v dpkg-deb >/dev/null || {
  echo "dpkg-deb is required to extract the NVIDIA CUDA packages" >&2
  exit 1
}

mkdir -p "${deb_dir}"
for package in "${packages[@]}"; do
  if compgen -G "${deb_dir}/${package}_*.deb" >/dev/null; then
    continue
  fi
  echo "Downloading ${package}..."
  (
    cd "${deb_dir}"
    apt-get download "${package}"
  )
done

stage="${vendor_root}/root.tmp.$$"
trap 'rm -rf -- "${stage}"' EXIT
mkdir -p "${stage}"
for package in "${packages[@]}"; do
  mapfile -t archives < <(compgen -G "${deb_dir}/${package}_*.deb" | sort)
  if [[ ${#archives[@]} -ne 1 ]]; then
    echo "Expected one archive for ${package}, found ${#archives[@]}" >&2
    exit 1
  fi
  dpkg-deb -x "${archives[0]}" "${stage}"
done

staged_toolkit="${stage}/usr/local/cuda-12.8"
if [[ ! -x "${staged_toolkit}/bin/nvcc" ]]; then
  echo "Extracted CUDA toolchain has no nvcc: ${staged_toolkit}" >&2
  exit 1
fi
if ! "${staged_toolkit}/bin/nvcc" --version | grep -q 'release 12\.8'; then
  echo "Extracted nvcc is not CUDA 12.8" >&2
  exit 1
fi
if ! grep -Rq 'cudaStreamGetDevice' "${staged_toolkit}/include"; then
  echo "Extracted CUDA headers do not provide cudaStreamGetDevice" >&2
  exit 1
fi

rm -rf -- "${extract_root}"
mv -- "${stage}" "${extract_root}"
trap - EXIT

echo "CUDA 12.8 toolchain prepared without system changes: ${toolkit_root}"
