#!/usr/bin/env bash

# Reproduce the user-space topic_tools overlay used by PC2 without modifying /opt/ros.
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly MIGRATION_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly SOURCE_ROOT="${MIGRATION_ROOT}/vendor-src/topic_tools-1.1.2"
readonly SOURCE_MANIFEST="${SOURCE_ROOT}/VENDORED_SOURCE_MANIFEST.sha256"
readonly BUILD_ROOT="${MIGRATION_ROOT}/vendor-build/topic_tools-1.1.2-HH_260810"
readonly PATCH_FILE="${MIGRATION_ROOT}/patches/topic_tools-1.1.2-relay-shutdown.patch"
readonly UPSTREAM_URL="https://github.com/ros-tooling/topic_tools.git"
readonly UPSTREAM_TAG="1.1.2"
readonly UPSTREAM_COMMIT="0fc927b7c0af0aaffae34a947b6ea4a7f9f97c94"
readonly EXPECTED_PATCHED_RELAY_SOURCE_SHA256="4e24a9cc53bfbcfb76680d7d612104ba2126b8631d9db72607e9d4e1878dc6f8"
readonly EXPECTED_RELAY_SHA256="45ad17f6e202a74cba025ad94f219f7402ef532551cd1b8162923267a16cb9e3"

source /opt/ros/humble/setup.bash

mkdir -p -- "$(dirname -- "${SOURCE_ROOT}")" "${BUILD_ROOT}"

if [[ ! -e "${SOURCE_ROOT}" ]]; then
  git clone --depth 1 --branch "${UPSTREAM_TAG}" "${UPSTREAM_URL}" "${SOURCE_ROOT}"
fi

if [[ -d "${SOURCE_ROOT}/.git" ]]; then
  if [[ "$(git -C "${SOURCE_ROOT}" rev-parse HEAD)" != "${UPSTREAM_COMMIT}" ]]; then
    echo "topic_tools source is not the audited 1.1.2 commit ${UPSTREAM_COMMIT}." >&2
    exit 65
  fi

  if git -C "${SOURCE_ROOT}" apply --check --unidiff-zero "${PATCH_FILE}"; then
    git -C "${SOURCE_ROOT}" apply --unidiff-zero "${PATCH_FILE}"
  elif ! git -C "${SOURCE_ROOT}" apply --reverse --check --unidiff-zero "${PATCH_FILE}"; then
    echo "The topic_tools tree is neither pristine nor exactly patched." >&2
    exit 65
  fi
else
  # HH_260811 - The release branch vendors a flattened source tree so GitHub cannot
  # interpret topic_tools as a nested repository or omit it during checkout.
  if [[ ! -f "${SOURCE_MANIFEST}" ]]; then
    echo "Flattened topic_tools source manifest is missing: ${SOURCE_MANIFEST}" >&2
    exit 65
  fi
  if ! (cd -- "${SOURCE_ROOT}" && sha256sum --quiet -c "$(basename -- "${SOURCE_MANIFEST}")"); then
    echo "Flattened topic_tools source differs from the audited manifest." >&2
    exit 65
  fi
fi

actual_relay_source_sha256="$(sha256sum "${SOURCE_ROOT}/topic_tools/src/relay_node.cpp" | awk '{print $1}')"
if [[ "${actual_relay_source_sha256}" != "${EXPECTED_PATCHED_RELAY_SOURCE_SHA256}" ]]; then
  echo "topic_tools relay source is not the accepted shutdown-safe revision." >&2
  exit 65
fi

colcon --log-base "${BUILD_ROOT}/log" build \
  --base-paths "${SOURCE_ROOT}" \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select topic_tools_interfaces topic_tools \
  --cmake-args -DBUILD_TESTING=OFF

relay_library="${BUILD_ROOT}/install/lib/librelay_node.so"
actual_sha256="$(sha256sum "${relay_library}" | awk '{print $1}')"
printf 'relay_library=%s\nsha256=%s\n' "${relay_library}" "${actual_sha256}"

if [[ "${actual_sha256}" != "${EXPECTED_RELAY_SHA256}" ]]; then
  echo "Build completed, but the relay hash differs from the binary used in PC2 acceptance." >&2
  echo "Do not update run_pc2_autoware.sh until the rebuilt overlay passes the shutdown tests." >&2
  exit 66
fi
