#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
package="libexpected-dev"
version="1.0.0~dfsg-2"
checksum="b6aa7e265d3f0a3ae10744c00089bb6bb6415b4973d3bbdf04a7f6b86576da92"
vendor_root="${AUTOWARE_E2E_TL_EXPECTED_VENDOR_ROOT:-${root}/data/vendor/tl-expected-1.0.0}"
deb_dir="${vendor_root}/debs"
extract_root="${vendor_root}/root"
prefix="${extract_root}/usr"
header="${prefix}/include/tl/expected.hpp"
archive="${deb_dir}/${package}_${version}_all.deb"

if [[ -f "${header}" ]] && \
  grep -Eq '^#define TL_EXPECTED_VERSION_MAJOR +1([[:space:]]|$)' "${header}"; then
  echo "tl::expected 1.0 already prepared without system changes: ${prefix}"
  exit 0
fi

for command in apt-get dpkg-deb sha256sum; do
  command -v "${command}" >/dev/null || {
    echo "${command} is required to prepare libexpected-dev" >&2
    exit 1
  }
done

mkdir -p "${deb_dir}"
if [[ ! -f "${archive}" ]] || \
  ! echo "${checksum}  ${archive}" | sha256sum --check --status; then
  rm -f -- "${archive}"
  echo "Downloading ${package}=${version}..."
  (
    cd "${deb_dir}"
    apt-get download "${package}=${version}"
  )
fi
echo "${checksum}  ${archive}" | sha256sum --check --status

stage="${vendor_root}/root.tmp.$$"
trap 'rm -rf -- "${stage}"' EXIT
mkdir -p "${stage}"
dpkg-deb -x "${archive}" "${stage}"

staged_header="${stage}/usr/include/tl/expected.hpp"
if [[ ! -f "${staged_header}" ]] || \
  ! grep -Eq '^#define TL_EXPECTED_VERSION_MAJOR +1([[:space:]]|$)' "${staged_header}"; then
  echo "Extracted libexpected-dev package has no compatible tl/expected.hpp" >&2
  exit 1
fi

rm -rf -- "${extract_root}"
mv -- "${stage}" "${extract_root}"
trap - EXIT

echo "tl::expected 1.0 prepared without system changes: ${prefix}"
