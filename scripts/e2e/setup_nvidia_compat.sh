#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
kernel_version="$(sed -n 's/^NVRM version:.*  \([0-9][0-9.]*\)  .*/\1/p' /proc/driver/nvidia/version 2>/dev/null)"
if [[ -z "${kernel_version}" ]]; then
  echo "Unable to read the loaded NVIDIA kernel module version" >&2
  exit 1
fi

major="${kernel_version%%.*}"
package_version="${AUTOWARE_E2E_NVIDIA_PACKAGE_VERSION:-${kernel_version}-1ubuntu1}"
vendor_root="${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT:-${root}/data/vendor/nvidia-${kernel_version}}"
deb_dir="${vendor_root}/debs"
extract_root="${vendor_root}/root"
packages=(
  "libnvidia-compute-${major}"
  "libnvidia-gl-${major}"
  "libnvidia-gpucomp-${major}"
)

if [[ -f "${extract_root}/usr/lib/x86_64-linux-gnu/libcuda.so.${kernel_version}" ]] &&
  [[ -f "${extract_root}/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.${kernel_version}" ]] &&
  [[ -f "${extract_root}/usr/lib/x86_64-linux-gnu/libnvidia-gpucomp.so.${kernel_version}" ]]; then
  echo "NVIDIA ${kernel_version} compatibility libraries already prepared: ${extract_root}"
  echo "export AUTOWARE_E2E_NVIDIA_COMPAT_ROOT=${extract_root}"
  exit 0
fi

command -v apt-get >/dev/null || {
  echo "apt-get is required to download the NVIDIA packages" >&2
  exit 1
}
command -v dpkg-deb >/dev/null || {
  echo "dpkg-deb is required to extract the NVIDIA packages" >&2
  exit 1
}

mkdir -p "${deb_dir}"
for package in "${packages[@]}"; do
  archive="${deb_dir}/${package}_${package_version}_amd64.deb"
  if [[ ! -f "${archive}" ]]; then
    echo "Downloading ${package}=${package_version}..."
    (
      cd "${deb_dir}"
      apt-get download "${package}=${package_version}"
    )
  fi
done

stage="${vendor_root}/root.tmp.$$"
trap 'rm -rf -- "${stage}"' EXIT
mkdir -p "${stage}"
for package in "${packages[@]}"; do
  archive="${deb_dir}/${package}_${package_version}_amd64.deb"
  if [[ ! -f "${archive}" ]]; then
    echo "Downloaded archive has an unexpected name: ${archive}" >&2
    exit 1
  fi
  dpkg-deb -x "${archive}" "${stage}"
done

lib_dir="${stage}/usr/lib/x86_64-linux-gnu"
for library in \
  "libcuda.so.${kernel_version}" \
  "libnvidia-ml.so.${kernel_version}" \
  "libGLX_nvidia.so.${kernel_version}" \
  "libnvidia-gpucomp.so.${kernel_version}"; do
  if [[ ! -f "${lib_dir}/${library}" ]]; then
    echo "Extracted NVIDIA package is missing ${library}" >&2
    exit 1
  fi
done

rm -rf -- "${extract_root}"
mv -- "${stage}" "${extract_root}"
trap - EXIT

echo "NVIDIA ${kernel_version} compatibility libraries prepared without system changes."
echo "export AUTOWARE_E2E_NVIDIA_COMPAT_ROOT=${extract_root}"
