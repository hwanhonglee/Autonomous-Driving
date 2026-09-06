#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
repository="${root}/src/core/autoware_core"
patch_file="${root}/patches/autoware_map_loader_clean_shutdown.patch"
package_dir="${repository}/map/autoware_map_loader"
generator="${package_dir}/script/map_hash_generator"
cmake_file="${package_dir}/CMakeLists.txt"
package_file="${package_dir}/package.xml"
unit_test="${package_dir}/test/test_map_hash_generator.py"

if [[ ! -d "${repository}/.git" ]]; then
  echo "Autoware Core repository is missing: ${repository}" >&2
  exit 1
fi
if [[ ! -f "${patch_file}" ]]; then
  echo "Map loader clean shutdown patch is missing: ${patch_file}" >&2
  exit 1
fi

# HH_260906 - Require every runtime and test marker before accepting an applied patch.
if grep -Fq 'from rclpy.executors import ExternalShutdownException' \
    "${generator}" 2>/dev/null \
  && grep -Fq 'except (KeyboardInterrupt, ExternalShutdownException):' \
    "${generator}" 2>/dev/null \
  && grep -Fq 'ament_add_pytest_test(test_map_hash_generator' \
    "${cmake_file}" 2>/dev/null \
  && grep -Fq '<test_depend>ament_cmake_pytest</test_depend>' \
    "${package_file}" 2>/dev/null \
  && grep -Fq 'test_main_handles_expected_executor_shutdown' \
    "${unit_test}" 2>/dev/null; then
  echo "Map loader clean shutdown patch is already applied."
  exit 0
fi

if git -C "${repository}" apply --reverse --check "${patch_file}" 2>/dev/null; then
  echo "Map loader clean shutdown patch is already applied."
  exit 0
fi
if ! git -C "${repository}" apply --check "${patch_file}"; then
  echo "Map loader clean shutdown patch does not apply cleanly." >&2
  exit 1
fi

git -C "${repository}" apply "${patch_file}"
echo "Applied map loader clean shutdown patch."
