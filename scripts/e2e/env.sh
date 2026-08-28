#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this file: source scripts/e2e/env.sh" >&2
  exit 2
fi

_autoware_e2e_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_autoware_e2e_restore_nounset=false
if [[ $- == *u* ]]; then
  _autoware_e2e_restore_nounset=true
  set +u
fi

# Remove prefixes inherited from older Autoware workspaces before sourcing Humble.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
unset CMAKE_MODULE_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH GAZEBO_MODEL_PATH
unset COLCON_DEFAULTS_FILE
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION RMW_IMPLEMENTATION CYCLONEDDS_URI
unset AUTOWARE_ROOT TENSORRT_ROOT ACADOS_SOURCE_DIR

# User login shells may already contain executable paths from another ROS
# workspace. Re-add only this workspace through local_setup.bash below.
_autoware_e2e_clean_path=""
IFS=: read -r -a _autoware_e2e_path_entries <<< "${PATH}"
for _autoware_e2e_path_entry in "${_autoware_e2e_path_entries[@]}"; do
  case "${_autoware_e2e_path_entry}" in
    */install/*) continue ;;
  esac
  if [[ -z "${_autoware_e2e_clean_path}" ]]; then
    _autoware_e2e_clean_path="${_autoware_e2e_path_entry}"
  else
    _autoware_e2e_clean_path+=":${_autoware_e2e_path_entry}"
  fi
done
export PATH="${_autoware_e2e_clean_path}"

source /opt/ros/humble/setup.bash

export AUTOWARE_E2E_ROOT="${_autoware_e2e_root}"
export AUTOWARE_ROOT="${_autoware_e2e_root}"
export AUTOWARE_E2E_DATA_PATH="${AUTOWARE_E2E_DATA_PATH:-${_autoware_e2e_root}/data/ml_models}"
export AUTOWARE_E2E_SPCONV_ROOT="${AUTOWARE_E2E_SPCONV_ROOT:-${_autoware_e2e_root}/data/vendor/spconv-cu128/root/usr/local}"
export AUTOWARE_E2E_ACADOS_ROOT="${AUTOWARE_E2E_ACADOS_ROOT:-${_autoware_e2e_root}/data/vendor/acados-v0.5.3}"
export ACADOS_SOURCE_DIR="${AUTOWARE_E2E_ACADOS_ROOT}"
export CARLA_ROOT="${CARLA_ROOT:-/home/hong/carla-autoware-universe/CARLA_0.9.15}"
export ROS_DOMAIN_ID="${AUTOWARE_E2E_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export TENSORRT_ROOT="${AUTOWARE_E2E_TENSORRT_ROOT:-/usr/local/cuda}"

# Autoware 1.9 builds against CUDA 12.8 APIs. Prefer the project-local
# compiler/runtime prepared by setup_cuda_12_8.sh while leaving TensorRT's
# independently configurable root unchanged.
_cuda_12_8_root="${_autoware_e2e_root}/data/vendor/cuda-12.8/root/usr/local/cuda-12.8"
export AUTOWARE_E2E_CUDA_ROOT="${AUTOWARE_E2E_CUDA_ROOT:-${_cuda_12_8_root}}"
if [[ ! -x "${AUTOWARE_E2E_CUDA_ROOT}/bin/nvcc" ]]; then
  echo "CUDA toolchain not found: ${AUTOWARE_E2E_CUDA_ROOT}" >&2
  echo "Run scripts/e2e/setup_cuda_12_8.sh first." >&2
  return 1
fi
export CUDA_HOME="${AUTOWARE_E2E_CUDA_ROOT}"
export CUDA_PATH="${AUTOWARE_E2E_CUDA_ROOT}"
export CUDA_BIN_PATH="${AUTOWARE_E2E_CUDA_ROOT}"
export CUDAToolkit_ROOT="${AUTOWARE_E2E_CUDA_ROOT}"
export CUDACXX="${AUTOWARE_E2E_CUDA_ROOT}/bin/nvcc"
export PATH="${AUTOWARE_E2E_CUDA_ROOT}/bin:${PATH}"
export LD_LIBRARY_PATH="${AUTOWARE_E2E_CUDA_ROOT}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [[ -n "${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT:-}" ]]; then
  _nvidia_compat_lib="${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT}/usr/lib/x86_64-linux-gnu"
  if [[ ! -d "${_nvidia_compat_lib}" ]]; then
    echo "NVIDIA compatibility libraries not found: ${_nvidia_compat_lib}" >&2
    return 1
  fi
  export PATH="${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT}/usr/bin:${PATH}"
  export LD_LIBRARY_PATH="${_nvidia_compat_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  _nvidia_icd="${AUTOWARE_E2E_NVIDIA_COMPAT_ROOT}/usr/share/vulkan/icd.d/nvidia_icd.json"
  if [[ -f "${_nvidia_icd}" ]]; then
    export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-${_nvidia_icd}}"
  fi
fi

if [[ -d "${AUTOWARE_E2E_SPCONV_ROOT}" ]]; then
  export CMAKE_PREFIX_PATH="${AUTOWARE_E2E_SPCONV_ROOT}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${AUTOWARE_E2E_SPCONV_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

if [[ -f "${ACADOS_SOURCE_DIR}/cmake/acadosConfig.cmake" ]]; then
  export CMAKE_PREFIX_PATH="${ACADOS_SOURCE_DIR}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

_carla_egg="${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
if [[ ! -f "${_carla_egg}" ]]; then
  echo "CARLA Python API not found: ${_carla_egg}" >&2
  return 1
fi

export PYTHONPATH="${CARLA_ROOT}/PythonAPI:${CARLA_ROOT}/PythonAPI/carla:${CARLA_ROOT}/PythonAPI/util:${_carla_egg}${PYTHONPATH:+:${PYTHONPATH}}"

if [[ "${AUTOWARE_E2E_SKIP_INSTALL:-0}" != "1" && -f "${_autoware_e2e_root}/install/local_setup.bash" ]]; then
  # setup.bash replays every underlay recorded by colcon at build time.  This
  # workspace was originally built beside older Autoware installations, so use
  # the local setup after sourcing Humble explicitly to avoid cross-version ABI
  # and RViz plugin contamination.
  source "${_autoware_e2e_root}/install/local_setup.bash"
fi

if [[ "${_autoware_e2e_restore_nounset}" == "true" ]]; then
  set -u
fi

unset _autoware_e2e_restore_nounset _autoware_e2e_clean_path
unset _autoware_e2e_path_entries _autoware_e2e_path_entry
unset _carla_egg _cuda_12_8_root _nvidia_compat_lib _autoware_e2e_root
