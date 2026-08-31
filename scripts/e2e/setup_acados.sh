#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
acados_version="v0.5.3"
tera_renderer_version="v0.2.0"
acados_root="${AUTOWARE_E2E_ACADOS_ROOT:-${root}/data/vendor/acados-${acados_version}}"
build_dir="${acados_root}/build"

for command in cmake curl git make python3 sha256sum; do
  if ! command -v "${command}" >/dev/null; then
    echo "${command} is required to prepare acados" >&2
    exit 1
  fi
done

case "$(uname -m)" in
  x86_64)
    tera_arch="amd64"
    tera_sha256="1973ddd1b536dcd4a059a22f7259c3b0e6b6c878cbfbe7e9b962eba195f361d9"
    ;;
  aarch64)
    tera_arch="arm64"
    tera_sha256="a29f563c7f49368b1802864cdf2bad9d50fe0fb7180c9d210c188a30bb23f167"
    ;;
  *)
    echo "tera_renderer ${tera_renderer_version} is unsupported on $(uname -m)" >&2
    exit 1
    ;;
esac

mkdir -p "$(dirname "${acados_root}")"

if [[ ! -d "${acados_root}/.git" ]]; then
  if [[ -e "${acados_root}" ]]; then
    echo "Existing acados path is not a git checkout: ${acados_root}" >&2
    echo "Move it aside or set AUTOWARE_E2E_ACADOS_ROOT to an empty path." >&2
    exit 1
  fi

  clone_dir="${acados_root}.clone.$$"
  trap 'rm -rf -- "${clone_dir:-}" "${renderer_tmp:-}"' EXIT
  git clone \
    --branch "${acados_version}" \
    --depth 1 \
    --recurse-submodules \
    --shallow-submodules \
    https://github.com/acados/acados.git \
    "${clone_dir}"
  mv -- "${clone_dir}" "${acados_root}"
else
  checkout_tag="$(git -C "${acados_root}" describe --tags --exact-match HEAD 2>/dev/null || true)"
  if [[ "${checkout_tag}" != "${acados_version}" ]]; then
    echo "Expected acados ${acados_version}, found ${checkout_tag:-an untagged revision}: ${acados_root}" >&2
    exit 1
  fi
  git -C "${acados_root}" submodule update --init --recursive --depth 1
fi

install_complete=true
required_files=(
  cmake/acadosConfig.cmake
  include/acados_c/ocp_nlp_interface.h
  lib/libacados.so
  lib/libqpOASES_e.so
)
for file in "${required_files[@]}"; do
  if [[ ! -f "${acados_root}/${file}" ]]; then
    install_complete=false
    break
  fi
done

if [[ "${install_complete}" == "true" ]]; then
  if [[ ! -f "${build_dir}/CMakeCache.txt" ]] || \
    ! grep -Eq '^ACADOS_WITH_QPOASES:[^=]+=ON$' "${build_dir}/CMakeCache.txt" || \
    ! grep -Eq '^CMAKE_POSITION_INDEPENDENT_CODE:[^=]+=ON$' "${build_dir}/CMakeCache.txt" || \
    ! grep -Fqx "CMAKE_INSTALL_PREFIX:PATH=${acados_root}" "${build_dir}/CMakeCache.txt"; then
    install_complete=false
  fi
fi

if [[ "${install_complete}" != "true" ]]; then
  cmake \
    -S "${acados_root}" \
    -B "${build_dir}" \
    -DACADOS_WITH_QPOASES=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX="${acados_root}"
  cmake --build "${build_dir}" \
    --target install \
    --parallel "${ACADOS_BUILD_JOBS:-$(nproc)}"
fi

mkdir -p "${acados_root}/bin"
renderer="${acados_root}/bin/t_renderer"
if [[ ! -x "${renderer}" ]] || \
  ! echo "${tera_sha256}  ${renderer}" | sha256sum --check --status; then
  renderer_tmp="${renderer}.tmp.$$"
  trap 'rm -rf -- "${clone_dir:-}" "${renderer_tmp:-}"' EXIT
  curl \
    --fail \
    --location \
    --retry 3 \
    --output "${renderer_tmp}" \
    "https://github.com/acados/tera_renderer/releases/download/${tera_renderer_version}/t_renderer-${tera_renderer_version}-linux-${tera_arch}"
  echo "${tera_sha256}  ${renderer_tmp}" | sha256sum --check --status
  chmod 0755 "${renderer_tmp}"
  mv -- "${renderer_tmp}" "${renderer}"
fi

if [[ ! -x "${acados_root}/.venv/bin/python3" ]]; then
  if python3 -c 'import ensurepip' >/dev/null 2>&1; then
    python3 -m venv "${acados_root}/.venv"
  elif virtualenv_command="$(command -v virtualenv 2>/dev/null)"; then
    "${virtualenv_command}" --python python3 "${acados_root}/.venv"
  else
    echo "Python ensurepip is unavailable and virtualenv is not installed; install python3-venv or virtualenv" >&2
    exit 1
  fi
fi

"${acados_root}/.venv/bin/python3" -m pip install --upgrade pip
"${acados_root}/.venv/bin/python3" -m pip install casadi sympy
"${acados_root}/.venv/bin/python3" -m pip install \
  --editable "${acados_root}/interfaces/acados_template"

if [[ ! -f "${acados_root}/cmake/acadosConfig.cmake" ]]; then
  echo "acados CMake package was not installed under ${acados_root}/cmake" >&2
  exit 1
fi

ACADOS_SOURCE_DIR="${acados_root}" \
LD_LIBRARY_PATH="${acados_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "${acados_root}/.venv/bin/python3" -c \
  'import casadi, sympy; from acados_template import AcadosOcp'

trap - EXIT
echo "acados ${acados_version} is ready in ${acados_root}"
