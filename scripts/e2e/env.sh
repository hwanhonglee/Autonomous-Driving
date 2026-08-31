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
unset CPATH CPLUS_INCLUDE_PATH
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

# Autoware 1.9 sources use the newer tf2_ros *.hpp spellings, while the ROS 2
# Humble packages on Ubuntu 22.04 still ship several of these public headers as
# *.h.  Prepend tiny forwarding headers only for that older layout; newer ROS
# installations use their native headers unchanged.
_tf2_ros_include_root="/opt/ros/humble/include/tf2_ros"
_tf2_ros_header_compat="${_autoware_e2e_root}/include/ros_humble_tf2_compat"
if [[ ! -f "${_tf2_ros_include_root}/tf2_ros/buffer.hpp" ]] && \
   [[ -f "${_tf2_ros_include_root}/tf2_ros/buffer.h" ]]; then
  export CPATH="${_tf2_ros_header_compat}${CPATH:+:${CPATH}}"
  export AUTOWARE_E2E_TF2_HEADER_COMPAT_DIR="${_tf2_ros_header_compat}"
else
  unset AUTOWARE_E2E_TF2_HEADER_COMPAT_DIR
fi

# autoware_diffusion_planner's official package contract uses Ubuntu's
# libexpected-dev (<tl/expected.hpp>).  Prefer the exact, workspace-local
# package prepared by setup_tl_expected.sh so a full build does not depend on
# mutable host packages or confuse it with ROS Humble's differently named
# <tl_expected/expected.hpp> header.
export AUTOWARE_E2E_TL_EXPECTED_ROOT="${AUTOWARE_E2E_TL_EXPECTED_ROOT:-${_autoware_e2e_root}/data/vendor/tl-expected-1.0.0/root/usr}"
if [[ -f "${AUTOWARE_E2E_TL_EXPECTED_ROOT}/include/tl/expected.hpp" ]]; then
  export CPATH="${AUTOWARE_E2E_TL_EXPECTED_ROOT}/include${CPATH:+:${CPATH}}"
fi

# Autoware 1.9 uses ament_cmake_auto(USE_SCOPED_HEADER_INSTALL_DIR), which is
# newer than the macro shipped by Ubuntu 22.04's ROS 2 Humble packages.  Use a
# workspace-local compatibility implementation only when the system macro
# does not already support the option.
_ament_cmake_auto_macro="/opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_package.cmake"
_ament_cmake_auto_compat="${_autoware_e2e_root}/cmake/ament_cmake_auto_humble_compat"
if [[ -f "${_ament_cmake_auto_macro}" ]] && \
   ! grep -q "USE_SCOPED_HEADER_INSTALL_DIR" "${_ament_cmake_auto_macro}"; then
  export CMAKE_PREFIX_PATH="${_ament_cmake_auto_compat}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR="${_ament_cmake_auto_compat}/share/ament_cmake_auto/cmake"
else
  export AUTOWARE_E2E_AMENT_CMAKE_AUTO_DIR="$(dirname "${_ament_cmake_auto_macro}")"
fi
export AUTOWARE_E2E_DATA_PATH="${AUTOWARE_E2E_DATA_PATH:-${_autoware_e2e_root}/data/ml_models}"
export AUTOWARE_E2E_SPCONV_ROOT="${AUTOWARE_E2E_SPCONV_ROOT:-${_autoware_e2e_root}/data/vendor/spconv-cu128/root/usr/local}"
export AUTOWARE_E2E_ACADOS_ROOT="${AUTOWARE_E2E_ACADOS_ROOT:-${_autoware_e2e_root}/data/vendor/acados-v0.5.3}"
export ACADOS_SOURCE_DIR="${AUTOWARE_E2E_ACADOS_ROOT}"
if [[ -z "${CARLA_ROOT:-}" ]]; then
  for _carla_root_candidate in \
    "${_autoware_e2e_root}/../carla-autoware-universe/CARLA_0.9.15" \
    "/opt/carla-simulator" \
    "/opt/carla"; do
    if [[ -x "${_carla_root_candidate}/CarlaUE4.sh" ]]; then
      CARLA_ROOT="$(cd "${_carla_root_candidate}" && pwd)"
      break
    fi
  done
fi
export CARLA_ROOT="${CARLA_ROOT:-${_autoware_e2e_root}/../carla-autoware-universe/CARLA_0.9.15}"
export ROS_DOMAIN_ID="${AUTOWARE_E2E_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
_tensorrt_10_8_root="${_autoware_e2e_root}/data/vendor/tensorrt-10.8/root/usr"
if [[ -e "${_tensorrt_10_8_root}/lib64/libnvinfer.so" ]]; then
  _default_tensorrt_root="${_tensorrt_10_8_root}"
else
  _default_tensorrt_root="/usr/local/cuda"
fi
export TENSORRT_ROOT="${AUTOWARE_E2E_TENSORRT_ROOT:-${_default_tensorrt_root}}"
if [[ -d "${TENSORRT_ROOT}/lib64" ]]; then
  export LD_LIBRARY_PATH="${TENSORRT_ROOT}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
elif [[ -d "${TENSORRT_ROOT}/lib" ]]; then
  export LD_LIBRARY_PATH="${TENSORRT_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

# Autoware 1.9 builds against CUDA 12.8 APIs. Prefer the project-local
# compiler/runtime prepared by setup_cuda_12_8.sh while leaving TensorRT's
# independently configurable root unchanged.
_cuda_12_8_root="${_autoware_e2e_root}/data/vendor/cuda-12.8/root/usr/local/cuda-12.8"
export AUTOWARE_E2E_CUDA_ROOT="${AUTOWARE_E2E_CUDA_ROOT:-${_cuda_12_8_root}}"
if [[ ! -x "${AUTOWARE_E2E_CUDA_ROOT}/bin/nvcc" ]]; then
  if [[ "${AUTOWARE_E2E_REQUIRE_CUDA:-0}" == "1" ]]; then
    echo "CUDA toolchain not found: ${AUTOWARE_E2E_CUDA_ROOT}" >&2
    echo "Run scripts/e2e/setup_cuda_12_8.sh first." >&2
    return 1
  fi
  echo "WARNING: CUDA 12.8 build toolchain is not prepared; CARLA/map-only tools remain available." >&2
  echo "Run scripts/e2e/setup_cuda_12_8.sh before building Autoware." >&2
else
  export CUDA_HOME="${AUTOWARE_E2E_CUDA_ROOT}"
  export CUDA_PATH="${AUTOWARE_E2E_CUDA_ROOT}"
  export CUDA_BIN_PATH="${AUTOWARE_E2E_CUDA_ROOT}"
  export CUDAToolkit_ROOT="${AUTOWARE_E2E_CUDA_ROOT}"
  export CUDACXX="${AUTOWARE_E2E_CUDA_ROOT}/bin/nvcc"
  export PATH="${AUTOWARE_E2E_CUDA_ROOT}/bin:${PATH}"
  export LD_LIBRARY_PATH="${AUTOWARE_E2E_CUDA_ROOT}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

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
unset _carla_egg _carla_root_candidate _cuda_12_8_root _default_tensorrt_root
unset _tensorrt_10_8_root _nvidia_compat_lib _autoware_e2e_root
unset _ament_cmake_auto_macro _ament_cmake_auto_compat
unset _tf2_ros_include_root _tf2_ros_header_compat
