#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
requirements="${root}/requirements-map.txt"
finalizer="${root}/scripts/e2e/finalize_xodr_lanelet_map.py"

venv="${AUTOWARE_E2E_MAP_VENV:-${root}/.venv-map}"
bootstrap_python="${AUTOWARE_E2E_MAP_BOOTSTRAP_PYTHON:-python3}"
finalizer_python="${AUTOWARE_E2E_MAP_FINALIZER_PYTHON:-python3}"
translation_x_m="0"
translation_y_m="0"
translation_z_m="0"
yaw_rad="0"
waypoint_resolution_m="0.25"
bounds_margin_m="50"
speed_limit_kmh="30"
map_version="xodr-commonroad-0.8.5"
json_report=""
install=false
force=false
dry_run=false

usage() {
  cat <<'EOF'
Usage: build_xodr_lanelet_map.sh [OPTIONS] SOURCE_XODR RAW_OSM OUTPUT_OSM

Convert OpenDRIVE to CommonRoad Lanelet2, then finalize it for Autoware's
Local projector. RAW_OSM preserves the CommonRoad conversion for provenance;
OUTPUT_OSM is the Autoware-ready vector map.

Options:
  --venv PATH                    CommonRoad virtualenv (default: .venv-map)
  --install                      Create/update the virtualenv from requirements-map.txt
  --bootstrap-python PATH        Python used to create the virtualenv (default: python3)
  --finalizer-python PATH        Python with CARLA API available (default: python3)
  --translation-x-m METERS      CARLA-to-map X translation (default: 0)
  --translation-y-m METERS      CARLA-to-map Y translation (default: 0)
  --translation-z-m METERS      CARLA-to-map Z translation (default: 0)
  --yaw-rad RADIANS              CARLA-to-map yaw rotation (default: 0)
  --waypoint-resolution-m M      CARLA elevation sampling interval (default: 0.25)
  --bounds-margin-m METERS       XODR bounds pruning margin (default: 50)
  --speed-limit-kmh KMH          Inferred road speed limit (default: 30)
  --map-version TEXT             Lanelet2 MetaInfo map version
  --json-report PATH             Write finalizer provenance JSON
  --force                        Replace existing RAW_OSM/OUTPUT_OSM/report
  --dry-run                      Validate inputs and print commands without writing
  -h, --help                     Show this help

Environment equivalents:
  AUTOWARE_E2E_MAP_VENV, AUTOWARE_E2E_MAP_BOOTSTRAP_PYTHON,
  AUTOWARE_E2E_MAP_FINALIZER_PYTHON

Example:
  scripts/e2e/build_xodr_lanelet_map.sh --install --force \
    --translation-z-m -15 --json-report data/generated/c_track_finalize.json \
    /path/to/C_track_1_0_7.xodr \
    data/generated/c_track_commonroad.osm \
    data/generated/c_track_autoware.osm
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_value() {
  if [[ $# -lt 2 ]]; then
    echo "ERROR: $1 requires a value" >&2
    exit 2
  fi
}

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

positionals=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --venv)
      require_value "$@"
      venv="$2"
      shift 2
      ;;
    --install)
      install=true
      shift
      ;;
    --bootstrap-python)
      require_value "$@"
      bootstrap_python="$2"
      shift 2
      ;;
    --finalizer-python)
      require_value "$@"
      finalizer_python="$2"
      shift 2
      ;;
    --translation-x-m)
      require_value "$@"
      translation_x_m="$2"
      shift 2
      ;;
    --translation-y-m)
      require_value "$@"
      translation_y_m="$2"
      shift 2
      ;;
    --translation-z-m)
      require_value "$@"
      translation_z_m="$2"
      shift 2
      ;;
    --yaw-rad)
      require_value "$@"
      yaw_rad="$2"
      shift 2
      ;;
    --waypoint-resolution-m)
      require_value "$@"
      waypoint_resolution_m="$2"
      shift 2
      ;;
    --bounds-margin-m)
      require_value "$@"
      bounds_margin_m="$2"
      shift 2
      ;;
    --speed-limit-kmh)
      require_value "$@"
      speed_limit_kmh="$2"
      shift 2
      ;;
    --map-version)
      require_value "$@"
      map_version="$2"
      shift 2
      ;;
    --json-report)
      require_value "$@"
      json_report="$2"
      shift 2
      ;;
    --force)
      force=true
      shift
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      positionals+=("$@")
      break
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      positionals+=("$1")
      shift
      ;;
  esac
done

if [[ ${#positionals[@]} -ne 3 ]]; then
  usage >&2
  exit 2
fi

command -v realpath >/dev/null || die "realpath is required"
command -v sha256sum >/dev/null || die "sha256sum is required"
[[ -f "${requirements}" ]] || die "requirements file does not exist: ${requirements}"
[[ -f "${finalizer}" ]] || die "finalizer does not exist: ${finalizer}"

source_xodr="$(realpath -e -- "${positionals[0]}")" || \
  die "SOURCE_XODR is not a readable file: ${positionals[0]}"
raw_osm="$(realpath -m -- "${positionals[1]}")"
output_osm="$(realpath -m -- "${positionals[2]}")"
venv="$(realpath -m -- "${venv}")"
if [[ -n "${json_report}" ]]; then
  json_report="$(realpath -m -- "${json_report}")"
fi

[[ -r "${source_xodr}" ]] || die "SOURCE_XODR is not readable: ${source_xodr}"
[[ "${source_xodr}" != "${raw_osm}" ]] || die "RAW_OSM must differ from SOURCE_XODR"
[[ "${source_xodr}" != "${output_osm}" ]] || die "OUTPUT_OSM must differ from SOURCE_XODR"
[[ "${raw_osm}" != "${output_osm}" ]] || die "RAW_OSM and OUTPUT_OSM must differ"
if [[ -n "${json_report}" ]]; then
  [[ "${json_report}" != "${source_xodr}" ]] || die "JSON report must differ from SOURCE_XODR"
  [[ "${json_report}" != "${raw_osm}" ]] || die "JSON report must differ from RAW_OSM"
  [[ "${json_report}" != "${output_osm}" ]] || die "JSON report must differ from OUTPUT_OSM"
fi

if [[ "${force}" != true ]]; then
  for output in "${raw_osm}" "${output_osm}"; do
    [[ ! -e "${output}" && ! -L "${output}" ]] || \
      die "output already exists (pass --force to replace it): ${output}"
  done
  if [[ -n "${json_report}" ]]; then
    [[ ! -e "${json_report}" && ! -L "${json_report}" ]] || \
      die "report already exists (pass --force to replace it): ${json_report}"
  fi
fi

command -v "${finalizer_python}" >/dev/null || \
  die "finalizer Python is not executable: ${finalizer_python}"
"${finalizer_python}" "${finalizer}" --help >/dev/null || \
  die "finalizer Python cannot import the required CARLA/map modules: ${finalizer_python}"

venv_python="${venv}/bin/python"
crdesigner="${venv}/bin/crdesigner"

verify_converter_environment() {
  [[ -x "${venv_python}" ]] || die "map virtualenv Python is missing: ${venv_python}"
  [[ -x "${crdesigner}" ]] || die "crdesigner is missing: ${crdesigner}"
  local versions
  if ! versions="$("${venv_python}" -c \
    'from importlib.metadata import version; print(version("commonroad-scenario-designer") + "|" + version("click"))')"; then
    die "cannot inspect CommonRoad package versions in ${venv}"
  fi
  [[ "${versions}" == "0.8.5|8.1.7" ]] || \
    die "map virtualenv version mismatch: expected 0.8.5|8.1.7, got ${versions}"
}

install_commands=()
if [[ "${install}" == true ]]; then
  command -v "${bootstrap_python}" >/dev/null || \
    die "bootstrap Python is not executable: ${bootstrap_python}"
  if [[ ! -e "${venv}" ]]; then
    if "${bootstrap_python}" -c 'import ensurepip' >/dev/null 2>&1; then
      install_commands+=("${bootstrap_python}" -m venv "${venv}")
    elif virtualenv_command="$(command -v virtualenv 2>/dev/null)"; then
      install_commands+=("${virtualenv_command}" --python "${bootstrap_python}" "${venv}")
    else
      die "Python ensurepip is unavailable and virtualenv is not installed; install python3-venv or virtualenv"
    fi
  elif [[ ! -x "${venv_python}" ]]; then
    die "existing --venv path is not a Python virtualenv: ${venv}"
  fi
fi

converter_command=(
  "${crdesigner}"
  --input-file "${source_xodr}"
  --output-file "${raw_osm}"
  --force-overwrite
  odrlanelet2
)
finalizer_command=(
  "${finalizer_python}" "${finalizer}"
  "${raw_osm}" "${source_xodr}" "${output_osm}"
  --translation-x-m "${translation_x_m}"
  --translation-y-m "${translation_y_m}"
  --translation-z-m "${translation_z_m}"
  --yaw-rad "${yaw_rad}"
  --waypoint-resolution-m "${waypoint_resolution_m}"
  --bounds-margin-m "${bounds_margin_m}"
  --speed-limit-kmh "${speed_limit_kmh}"
  --map-version "${map_version}"
)
if [[ -n "${json_report}" ]]; then
  finalizer_command+=(--json)
fi

if [[ "${dry_run}" == true ]]; then
  echo "XODR Lanelet map build plan:"
  if [[ "${install}" == true ]]; then
    if [[ ${#install_commands[@]} -gt 0 ]]; then
      print_command "${install_commands[@]}"
    fi
    print_command "${venv_python}" -m pip install --requirement "${requirements}"
  else
    verify_converter_environment
  fi
  print_command mkdir -p "$(dirname "${raw_osm}")" "$(dirname "${output_osm}")"
  if [[ -n "${json_report}" ]]; then
    print_command mkdir -p "$(dirname "${json_report}")"
  fi
  print_command "${converter_command[@]}"
  if [[ -n "${json_report}" ]]; then
    printf '  '
    printf '%q ' "${finalizer_command[@]}"
    printf '> %q\n' "${json_report}"
  else
    print_command "${finalizer_command[@]}"
  fi
  exit 0
fi

if [[ "${install}" == true ]]; then
  if [[ ${#install_commands[@]} -gt 0 ]]; then
    "${install_commands[@]}"
  fi
  "${venv_python}" -m pip install --requirement "${requirements}"
fi
verify_converter_environment

mkdir -p -- "$(dirname "${raw_osm}")" "$(dirname "${output_osm}")"
if [[ -n "${json_report}" ]]; then
  mkdir -p -- "$(dirname "${json_report}")"
fi

"${converter_command[@]}"
if [[ -n "${json_report}" ]]; then
  "${finalizer_command[@]}" > "${json_report}"
else
  "${finalizer_command[@]}"
fi

[[ -s "${raw_osm}" ]] || die "CommonRoad conversion did not create RAW_OSM: ${raw_osm}"
[[ -s "${output_osm}" ]] || die "finalizer did not create OUTPUT_OSM: ${output_osm}"
if [[ -n "${json_report}" ]]; then
  [[ -s "${json_report}" ]] || die "finalizer did not create JSON report: ${json_report}"
fi

echo "XODR Lanelet map build completed:"
sha256sum -- "${source_xodr}" "${raw_osm}" "${output_osm}"
