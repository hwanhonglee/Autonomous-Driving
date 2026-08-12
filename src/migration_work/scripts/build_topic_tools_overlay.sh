#!/usr/bin/env bash

# Build the audited shutdown-safe topic_tools overlay without modifying /opt/ros.
set -eo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly MIGRATION_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly SOURCE_ROOT="${MIGRATION_ROOT}/vendor-src/topic_tools-1.1.2"
readonly BUILD_ROOT="${MIGRATION_ROOT}/vendor-build/topic_tools-1.1.2-HH_260810"
readonly EXPECTED_RELAY_SOURCE_SHA256="4e24a9cc53bfbcfb76680d7d612104ba2126b8631d9db72607e9d4e1878dc6f8"

source /opt/ros/humble/setup.bash
if [[ -f /home/a/autoware/install/setup.bash ]]; then
  source /home/a/autoware/install/setup.bash
fi
set -u

for required_file in \
  "${SOURCE_ROOT}/topic_tools/package.xml" \
  "${SOURCE_ROOT}/topic_tools_interfaces/package.xml" \
  "${SOURCE_ROOT}/topic_tools/src/relay_node.cpp"
do
  if [[ ! -f "${required_file}" ]]; then
    echo "Vendored topic_tools source is incomplete: ${required_file}" >&2
    exit 65
  fi
done

actual_source_sha256="$(
  sha256sum "${SOURCE_ROOT}/topic_tools/src/relay_node.cpp" | awk '{print $1}'
)"
if [[ "${actual_source_sha256}" != "${EXPECTED_RELAY_SOURCE_SHA256}" ]]; then
  echo "Vendored relay source is not the audited shutdown-safe revision." >&2
  exit 65
fi

mkdir -p -- "${BUILD_ROOT}"
colcon --log-base "${BUILD_ROOT}/log" build \
  --merge-install \
  --base-paths "${SOURCE_ROOT}" \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select topic_tools_interfaces topic_tools \
  --allow-overriding topic_tools \
  --cmake-args -DBUILD_TESTING=OFF

relay_library="${BUILD_ROOT}/install/lib/librelay_node.so"
relay_executable="${BUILD_ROOT}/install/lib/topic_tools/relay"
if [[ ! -r "${relay_library}" || ! -x "${relay_executable}" ]]; then
  echo "topic_tools overlay build did not produce the required relay artifacts." >&2
  exit 66
fi

printf 'overlay=%s\nrelay_library_sha256=%s\n' \
  "${BUILD_ROOT}/install" \
  "$(sha256sum "${relay_library}" | awk '{print $1}')"
