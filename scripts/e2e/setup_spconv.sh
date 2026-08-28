#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
vendor_dir="${AUTOWARE_E2E_VENDOR_PATH:-${root}/data/vendor}/spconv-cu128"
deb_dir="${vendor_dir}/debs"
install_root="${vendor_dir}/root"
release_url="https://github.com/autowarefoundation/spconv_cpp/releases/download/spconv_v2.3.8%2Bcumm_v0.5.3%2Bcu128-rev1"

if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "The pinned local spconv bundle currently supports amd64 only." >&2
  exit 1
fi

mkdir -p "${deb_dir}"

download() {
  local file="$1"
  local expected_sha256="$2"
  local path="${deb_dir}/${file}"

  if [[ -f "${path}" ]] && echo "${expected_sha256}  ${path}" | sha256sum --check --status; then
    return
  fi

  rm -f "${path}"
  curl --fail --location --retry 3 --output "${path}" "${release_url}/${file}"
  echo "${expected_sha256}  ${path}" | sha256sum --check --status
}

download cumm_0.5.3_amd64.deb \
  2ff26eab84b88e7eb736fd4abad8e7a1f57f3136a6bd6c1705a1501283b34ba3
download spconv_2.3.8_amd64.deb \
  88bbb6b9b17ed6cfca2591d52d102600e6c3ac63d117d7b0d550c52681dc18ea

required_files=(
  usr/local/lib/libspconv.so
  usr/local/lib/cmake/spconv/spconvConfig.cmake
  usr/local/share/cmake/cumm/cummConfig.cmake
  usr/local/include/tensorview/core/all.h
)

install_complete=true
for file in "${required_files[@]}"; do
  if [[ ! -f "${install_root}/${file}" ]]; then
    install_complete=false
    break
  fi
done

if [[ "${install_complete}" != "true" ]]; then
  rm -rf "${install_root}.tmp"
  mkdir -p "${install_root}.tmp"
  dpkg-deb --extract "${deb_dir}/cumm_0.5.3_amd64.deb" "${install_root}.tmp"
  dpkg-deb --extract "${deb_dir}/spconv_2.3.8_amd64.deb" "${install_root}.tmp"
  rm -rf "${install_root}"
  mv "${install_root}.tmp" "${install_root}"
fi

echo "Local cumm/spconv dependencies are ready in ${install_root}/usr/local"
