#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
version="10.8.0.43-1+cuda12.8"
vendor_root="${AUTOWARE_E2E_TENSORRT_10_8_VENDOR_ROOT:-${root}/data/vendor/tensorrt-10.8}"
deb_dir="${vendor_root}/debs"
extract_root="${vendor_root}/root"
prefix="${extract_root}/usr"

# The multi-gigabyte -dev package contains static archives that Autoware does
# not use.  Runtime shared objects, public headers, and the small ONNX parser
# development package are sufficient for a relocatable dynamic build.
packages=(
  libnvinfer10
  libnvinfer-plugin10
  libnvonnxparsers10
  libnvinfer-headers-dev
  libnvinfer-headers-plugin-dev
  libnvonnxparsers-dev
)
declare -A checksums=(
  [libnvinfer10]=476643761941560a3adeda43921e379bd859aade12aaf0143f8d05bd21e63fe8
  [libnvinfer-plugin10]=7ccc79961b21544634ab16a51937558038ddabf20068eeba90196de43e3f1ec3
  [libnvonnxparsers10]=79312e4f70e9dd5ae4606ad5a89f0dfe5ebe487f4d9ef0c7da963f731cf183cc
  [libnvinfer-headers-dev]=288568c1646c887b11462b52ebb0833c19fa3736070cb2a05ef4cbf3b6ffb9dd
  [libnvinfer-headers-plugin-dev]=d79842dcc68f2687895982524e3e10b0bc28913d34c00737c72aba318d973ae2
  [libnvonnxparsers-dev]=1630d43d17d463bfe14ad6da56f1cf23194a5446b69dc5fba629650eb71bb6f1
)

required=(
  "${prefix}/include/NvInfer.h"
  "${prefix}/include/NvInferPlugin.h"
  "${prefix}/include/NvOnnxParser.h"
  "${prefix}/lib64/libnvinfer.so"
  "${prefix}/lib64/libnvinfer_plugin.so"
  "${prefix}/lib64/libnvonnxparser.so"
)
complete=true
for path in "${required[@]}"; do
  if [[ ! -e "${path}" ]]; then
    complete=false
    break
  fi
done
if [[ "${complete}" == "true" ]] && \
  grep -Eq '^#define NV_TENSORRT_MAJOR +10([[:space:]]|$)' "${prefix}/include/NvInferVersion.h" && \
  grep -Eq '^#define NV_TENSORRT_MINOR +8([[:space:]]|$)' "${prefix}/include/NvInferVersion.h"; then
  echo "TensorRT 10.8 already prepared without system changes: ${prefix}"
  exit 0
fi

for command in apt-get dpkg-deb sha256sum; do
  command -v "${command}" >/dev/null || {
    echo "${command} is required to prepare TensorRT" >&2
    exit 1
  }
done

mkdir -p "${deb_dir}"
for package in "${packages[@]}"; do
  archive="${deb_dir}/${package}_${version}_amd64.deb"
  if [[ ! -f "${archive}" ]] || \
    ! echo "${checksums[${package}]}  ${archive}" | sha256sum --check --status; then
    rm -f -- "${archive}"
    echo "Downloading ${package}=${version}..."
    (
      cd "${deb_dir}"
      apt-get download "${package}=${version}"
    )
  fi
  echo "${checksums[${package}]}  ${archive}" | sha256sum --check --status
done

stage="${vendor_root}/root.tmp.$$"
trap 'rm -rf -- "${stage}"' EXIT
mkdir -p "${stage}"
for package in "${packages[@]}"; do
  dpkg-deb -x "${deb_dir}/${package}_${version}_amd64.deb" "${stage}"
done

lib_dir="${stage}/usr/lib/x86_64-linux-gnu"
header_dir="${stage}/usr/include/x86_64-linux-gnu"
for soname in libnvinfer.so.10 libnvinfer_plugin.so.10 libnvonnxparser.so.10; do
  [[ -e "${lib_dir}/${soname}" ]] || {
    echo "Extracted TensorRT packages are missing ${soname}" >&2
    exit 1
  }
  ln -sfn "${soname}" "${lib_dir}/${soname%.10}"
done
ln -sfn lib/x86_64-linux-gnu "${stage}/usr/lib64"
for header in "${header_dir}"/Nv*.h; do
  [[ -f "${header}" ]] || continue
  ln -sfn "x86_64-linux-gnu/$(basename "${header}")" \
    "${stage}/usr/include/$(basename "${header}")"
done

for path in \
  "${stage}/usr/include/NvInfer.h" \
  "${stage}/usr/include/NvInferPlugin.h" \
  "${stage}/usr/include/NvOnnxParser.h" \
  "${stage}/usr/lib64/libnvinfer.so" \
  "${stage}/usr/lib64/libnvinfer_plugin.so" \
  "${stage}/usr/lib64/libnvonnxparser.so"; do
  [[ -e "${path}" ]] || {
    echo "TensorRT normalized prefix is incomplete: ${path}" >&2
    exit 1
  }
done

rm -rf -- "${extract_root}"
mv -- "${stage}" "${extract_root}"
trap - EXIT

echo "TensorRT 10.8 prepared without system changes: ${prefix}"
