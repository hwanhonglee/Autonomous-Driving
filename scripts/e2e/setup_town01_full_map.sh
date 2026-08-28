#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DEFAULT_MAP_ROOT="${HOME}/Downloads/sample-planning-map"
DEFAULT_OSM_SOURCE="${DEFAULT_MAP_ROOT}/vector_maps/lanelet2/CARLA Town/Town01_light.osm"
DEFAULT_PCD_SOURCE="${DEFAULT_MAP_ROOT}/point_cloud_maps/CARLA Town/Town01.pcd"
DEFAULT_TARGET_DIR="${PROJECT_ROOT}/data/maps/Town01_full"

# These hashes pin the Town01 files verified in the local Downloads map set.
DEFAULT_OSM_SHA256="7098b8124b2e6e947237b2f3e0dfb3c8a707898edf48246b1032204c6ccbfaae"
DEFAULT_PCD_SHA256="296d6429ae0190b19a35458a155d76d8e3c4f460eb718cde3d669bc5a8f03ba2"

OSM_SOURCE="${TOWN01_OSM_SOURCE:-${DEFAULT_OSM_SOURCE}}"
PCD_SOURCE="${TOWN01_PCD_SOURCE:-${DEFAULT_PCD_SOURCE}}"
TARGET_DIR="${TOWN01_MAP_TARGET_DIR:-${DEFAULT_TARGET_DIR}}"
OSM_SHA256="${TOWN01_OSM_SHA256-${DEFAULT_OSM_SHA256}}"
PCD_SHA256="${TOWN01_PCD_SHA256-${DEFAULT_PCD_SHA256}}"
DRY_RUN=false
SKIP_PCD_COORDINATE_CHECK=false

usage() {
  cat <<'EOF'
Usage: scripts/e2e/setup_town01_full_map.sh [options]

Create an Autoware Town01 full-map bundle without copying or modifying its
source map files. The lanelet and point-cloud targets are symbolic links; the
projector file is generated as `projector_type: Local`.

Options:
  --osm-source PATH             Town01 Lanelet2 OSM source
  --pcd-source PATH             Town01 point-cloud PCD source
  --target-dir PATH             Output map directory
  --osm-sha256 HASH             Expected OSM SHA-256 (empty disables pinning)
  --pcd-sha256 HASH             Expected PCD SHA-256 (empty disables pinning)
  --skip-pcd-coordinate-check   Skip the Open3D bounds check
  --dry-run                     Validate and print changes without writing
  -h, --help                    Show this help

Environment equivalents:
  TOWN01_OSM_SOURCE, TOWN01_PCD_SOURCE, TOWN01_MAP_TARGET_DIR,
  TOWN01_OSM_SHA256, TOWN01_PCD_SHA256

For a custom source, pass an empty expected hash explicitly if it is not one of
the pinned files, for example: --osm-sha256 ''.
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --osm-source)
      (($# >= 2)) || die "--osm-source requires a path"
      OSM_SOURCE="$2"
      shift 2
      ;;
    --pcd-source)
      (($# >= 2)) || die "--pcd-source requires a path"
      PCD_SOURCE="$2"
      shift 2
      ;;
    --target-dir)
      (($# >= 2)) || die "--target-dir requires a path"
      TARGET_DIR="$2"
      shift 2
      ;;
    --osm-sha256)
      (($# >= 2)) || die "--osm-sha256 requires a value (use '' to disable)"
      OSM_SHA256="$2"
      shift 2
      ;;
    --pcd-sha256)
      (($# >= 2)) || die "--pcd-sha256 requires a value (use '' to disable)"
      PCD_SHA256="$2"
      shift 2
      ;;
    --skip-pcd-coordinate-check)
      SKIP_PCD_COORDINATE_CHECK=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

command -v python3 >/dev/null || die "python3 is required"
command -v realpath >/dev/null || die "realpath is required"
command -v sha256sum >/dev/null || die "sha256sum is required"

[[ -f "${OSM_SOURCE}" && -r "${OSM_SOURCE}" ]] || die "OSM source is not a readable file: ${OSM_SOURCE}"
[[ -f "${PCD_SOURCE}" && -r "${PCD_SOURCE}" ]] || die "PCD source is not a readable file: ${PCD_SOURCE}"

OSM_SOURCE="$(realpath -e -- "${OSM_SOURCE}")"
PCD_SOURCE="$(realpath -e -- "${PCD_SOURCE}")"
TARGET_DIR="$(realpath -m -- "${TARGET_DIR}")"

[[ "${OSM_SOURCE}" != "${PCD_SOURCE}" ]] || die "OSM and PCD sources resolve to the same file"
[[ "${TARGET_DIR}" != "${PROJECT_ROOT}" ]] || die "target directory cannot be the project root"

actual_osm_sha256="$(sha256sum -- "${OSM_SOURCE}" | awk '{print $1}')"
actual_pcd_sha256="$(sha256sum -- "${PCD_SOURCE}" | awk '{print $1}')"

if [[ -n "${OSM_SHA256}" && "${actual_osm_sha256}" != "${OSM_SHA256}" ]]; then
  die "OSM SHA-256 mismatch: expected ${OSM_SHA256}, got ${actual_osm_sha256}"
fi
if [[ -n "${PCD_SHA256}" && "${actual_pcd_sha256}" != "${PCD_SHA256}" ]]; then
  die "PCD SHA-256 mismatch: expected ${PCD_SHA256}, got ${actual_pcd_sha256}"
fi

coordinate_check=true
if [[ "${SKIP_PCD_COORDINATE_CHECK}" == "true" ]]; then
  coordinate_check=false
fi

echo "Checking Town01 map sources..."
python3 - "${OSM_SOURCE}" "${PCD_SOURCE}" "${coordinate_check}" <<'PY'
import math
import os
import sys
import xml.etree.ElementTree as ET


osm_path, pcd_path, coordinate_check_text = sys.argv[1:]
coordinate_check = coordinate_check_text == "true"

# WGS84 semi-major axis and meridional radius at the equator. Town01's
# near-zero lat/lon values encode the same local metric coordinates stored in
# local_x/local_y, so this catches a projector or map-family mismatch.
wgs84_a = 6_378_137.0
wgs84_e2 = 6.6943799901413165e-3
wgs84_meridional_radius = wgs84_a * (1.0 - wgs84_e2)

node_count = 0
way_count = 0
lanelet_count = 0
traffic_light_count = 0
local_coordinate_count = 0
lat_min = lon_min = x_min = y_min = math.inf
lat_max = lon_max = x_max = y_max = -math.inf
max_local_residual = 0.0

try:
    iterator = ET.iterparse(osm_path, events=("end",))
    for _, element in iterator:
        if element.tag == "node":
            node_count += 1
            try:
                latitude = float(element.attrib["lat"])
                longitude = float(element.attrib["lon"])
            except (KeyError, ValueError) as error:
                raise RuntimeError(f"invalid node latitude/longitude: {error}") from error
            if not (math.isfinite(latitude) and math.isfinite(longitude)):
                raise RuntimeError("OSM contains a non-finite latitude/longitude")
            lat_min = min(lat_min, latitude)
            lat_max = max(lat_max, latitude)
            lon_min = min(lon_min, longitude)
            lon_max = max(lon_max, longitude)

            tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in element.findall("tag")}
            if "local_x" in tags and "local_y" in tags:
                try:
                    local_x = float(tags["local_x"])
                    local_y = float(tags["local_y"])
                except (TypeError, ValueError) as error:
                    raise RuntimeError(f"invalid local_x/local_y: {error}") from error
                if not (math.isfinite(local_x) and math.isfinite(local_y)):
                    raise RuntimeError("OSM contains a non-finite local coordinate")
                local_coordinate_count += 1
                x_min = min(x_min, local_x)
                x_max = max(x_max, local_x)
                y_min = min(y_min, local_y)
                y_max = max(y_max, local_y)
                expected_x = math.radians(longitude) * wgs84_a * math.cos(math.radians(latitude))
                expected_y = math.radians(latitude) * wgs84_meridional_radius
                max_local_residual = max(
                    max_local_residual,
                    abs(local_x - expected_x),
                    abs(local_y - expected_y),
                )
            element.clear()
        elif element.tag == "way":
            way_count += 1
            element.clear()
        elif element.tag == "relation":
            tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in element.findall("tag")}
            if tags.get("type") == "lanelet":
                lanelet_count += 1
            if tags.get("subtype") == "traffic_light":
                traffic_light_count += 1
            element.clear()
except (ET.ParseError, OSError, RuntimeError) as error:
    raise SystemExit(f"OSM validation failed: {error}") from error

if node_count < 10_000:
    raise SystemExit(f"OSM validation failed: expected full map, found only {node_count} nodes")
if way_count < 300 or lanelet_count < 100:
    raise SystemExit(
        f"OSM validation failed: insufficient ways/lanelets ({way_count}/{lanelet_count})"
    )
if traffic_light_count < 30:
    raise SystemExit(
        f"OSM validation failed: expected traffic-light map, found {traffic_light_count} elements"
    )
if local_coordinate_count != node_count:
    raise SystemExit(
        "OSM validation failed: every Town01 node must contain local_x/local_y "
        f"({local_coordinate_count}/{node_count})"
    )
if max(abs(lat_min), abs(lat_max), abs(lon_min), abs(lon_max)) > 0.01:
    raise SystemExit("OSM validation failed: coordinates are not the near-origin CARLA Town01 map")
if x_max - x_min < 350.0 or y_max - y_min < 300.0:
    raise SystemExit(
        "OSM validation failed: local-coordinate coverage is too small "
        f"(x={x_min:.2f}..{x_max:.2f}, y={y_min:.2f}..{y_max:.2f})"
    )
if max_local_residual > 0.05:
    raise SystemExit(
        "OSM validation failed: lat/lon and local_x/local_y disagree with Local projection "
        f"(maximum residual {max_local_residual:.3f} m)"
    )

header = {}
header_bytes = 0
try:
    with open(pcd_path, "rb") as pcd_file:
        while header_bytes < 65_536:
            raw_line = pcd_file.readline()
            if not raw_line:
                break
            header_bytes += len(raw_line)
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise RuntimeError("binary data started before the DATA header") from error
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
except (OSError, RuntimeError) as error:
    raise SystemExit(f"PCD validation failed: {error}") from error

required_header_keys = {"VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "POINTS", "DATA"}
missing_header_keys = sorted(required_header_keys - header.keys())
if missing_header_keys:
    raise SystemExit(f"PCD validation failed: missing header keys {missing_header_keys}")
fields = header["FIELDS"].split()
if fields[:3] != ["x", "y", "z"]:
    raise SystemExit(f"PCD validation failed: expected leading x y z fields, got {fields}")
try:
    width = int(header["WIDTH"])
    height = int(header["HEIGHT"])
    point_count = int(header["POINTS"])
except ValueError as error:
    raise SystemExit(f"PCD validation failed: invalid point-count header: {error}") from error
if point_count < 10_000_000 or width * height != point_count:
    raise SystemExit(
        "PCD validation failed: point count is inconsistent or too small "
        f"(width={width}, height={height}, points={point_count})"
    )
if header["DATA"] not in {"ascii", "binary", "binary_compressed"}:
    raise SystemExit(f"PCD validation failed: unsupported DATA encoding {header['DATA']!r}")
if os.path.getsize(pcd_path) <= header_bytes:
    raise SystemExit("PCD validation failed: file contains a header but no point data")

pcd_bounds = None
if coordinate_check:
    try:
        import numpy as np
        import open3d as o3d
    except ImportError as error:
        raise SystemExit(
            "PCD coordinate validation requires python3-open3d and numpy; "
            "install them or use --skip-pcd-coordinate-check"
        ) from error
    point_cloud = o3d.io.read_point_cloud(
        pcd_path,
        remove_nan_points=False,
        remove_infinite_points=False,
        print_progress=False,
    )
    points = np.asarray(point_cloud.points)
    if points.shape != (point_count, 3):
        raise SystemExit(
            f"PCD validation failed: decoded shape {points.shape} does not match ({point_count}, 3)"
        )
    pcd_min = np.full(3, np.inf)
    pcd_max = np.full(3, -np.inf)
    for start in range(0, point_count, 1_000_000):
        chunk = points[start : start + 1_000_000]
        if not np.isfinite(chunk).all():
            raise SystemExit("PCD validation failed: cloud contains NaN or infinite coordinates")
        pcd_min = np.minimum(pcd_min, chunk.min(axis=0))
        pcd_max = np.maximum(pcd_max, chunk.max(axis=0))
    if np.max(np.abs(np.concatenate((pcd_min, pcd_max)))) > 10_000.0:
        raise SystemExit(f"PCD validation failed: implausible coordinate bounds {pcd_min}..{pcd_max}")
    if not (
        pcd_min[0] <= x_min <= x_max <= pcd_max[0]
        and pcd_min[1] <= y_min <= y_max <= pcd_max[1]
    ):
        raise SystemExit(
            "PCD validation failed: point cloud does not contain the Lanelet2 XY bounds "
            f"(OSM x={x_min:.2f}..{x_max:.2f}, y={y_min:.2f}..{y_max:.2f}; "
            f"PCD x={pcd_min[0]:.2f}..{pcd_max[0]:.2f}, "
            f"y={pcd_min[1]:.2f}..{pcd_max[1]:.2f})"
        )
    if pcd_max[2] - pcd_min[2] < 1.0:
        raise SystemExit("PCD validation failed: point cloud Z coverage is too small")
    pcd_bounds = (pcd_min, pcd_max)

print(
    "  OSM: "
    f"nodes={node_count}, ways={way_count}, lanelets={lanelet_count}, "
    f"traffic_lights={traffic_light_count}"
)
print(
    "  OSM bounds: "
    f"x={x_min:.2f}..{x_max:.2f} m, y={y_min:.2f}..{y_max:.2f} m, "
    f"Local residual<={max_local_residual:.6f} m"
)
print(f"  PCD: points={point_count}, encoding={header['DATA']}")
if pcd_bounds is not None:
    pcd_min, pcd_max = pcd_bounds
    print(
        "  PCD bounds: "
        f"x={pcd_min[0]:.2f}..{pcd_max[0]:.2f} m, "
        f"y={pcd_min[1]:.2f}..{pcd_max[1]:.2f} m, "
        f"z={pcd_min[2]:.2f}..{pcd_max[2]:.2f} m"
    )
else:
    print("  PCD bounds: skipped by request")
PY

echo "  OSM SHA-256: ${actual_osm_sha256}"
echo "  PCD SHA-256: ${actual_pcd_sha256}"

lanelet_target="${TARGET_DIR}/lanelet2_map.osm"
pointcloud_target="${TARGET_DIR}/pointcloud_map.pcd"
projector_target="${TARGET_DIR}/map_projector_info.yaml"
projector_content='projector_type: Local'

resolved_link_target() {
  local target="$1"
  local link_value
  link_value="$(readlink -- "${target}")"
  if [[ "${link_value}" == /* ]]; then
    realpath -m -- "${link_value}"
  else
    realpath -m -- "$(dirname "${target}")/${link_value}"
  fi
}

check_symlink_target() {
  local target="$1"
  local source="$2"

  if [[ -L "${target}" ]]; then
    local current_source
    current_source="$(resolved_link_target "${target}")"
    if [[ "${current_source}" == "${source}" ]]; then
      return 0
    fi
    echo "  update symlink: ${target} -> ${source}"
    return 0
  fi
  if [[ -e "${target}" ]]; then
    die "refusing to replace existing non-symlink target: ${target}"
  fi
  echo "  create symlink: ${target} -> ${source}"
}

check_projector_target() {
  if [[ -L "${projector_target}" ]]; then
    die "refusing to replace existing projector symlink: ${projector_target}"
  fi
  if [[ -e "${projector_target}" ]]; then
    local existing_content
    existing_content="$(sed -e 's/[[:space:]]*$//' -e '/^[[:space:]]*$/d' -- "${projector_target}")"
    if [[ "${existing_content}" != "${projector_content}" ]]; then
      die "refusing to replace existing projector file with different content: ${projector_target}"
    fi
    return 0
  fi
  echo "  create file: ${projector_target} (${projector_content})"
}

if [[ -L "${TARGET_DIR}" ]]; then
  die "refusing to use a symlink as the target directory: ${TARGET_DIR}"
fi
if [[ -e "${TARGET_DIR}" && ! -d "${TARGET_DIR}" ]]; then
  die "target path exists but is not a directory: ${TARGET_DIR}"
fi

echo "Preparing target bundle..."
if [[ ! -d "${TARGET_DIR}" ]]; then
  echo "  create directory: ${TARGET_DIR}"
fi
check_symlink_target "${lanelet_target}" "${OSM_SOURCE}"
check_symlink_target "${pointcloud_target}" "${PCD_SOURCE}"
check_projector_target

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "Dry run complete; no files were written."
  exit 0
fi

mkdir -p -- "${TARGET_DIR}"

ensure_symlink() {
  local target="$1"
  local source="$2"

  if [[ -L "${target}" ]]; then
    local current_source
    current_source="$(resolved_link_target "${target}")"
    if [[ "${current_source}" == "${source}" ]]; then
      return 0
    fi
  fi

  local temporary_target="${target}.tmp.$$"
  rm -f -- "${temporary_target}"
  ln -s -- "${source}" "${temporary_target}"
  mv -Tf -- "${temporary_target}" "${target}"
}

ensure_symlink "${lanelet_target}" "${OSM_SOURCE}"
ensure_symlink "${pointcloud_target}" "${PCD_SOURCE}"

if [[ ! -e "${projector_target}" ]]; then
  temporary_projector="${projector_target}.tmp.$$"
  trap 'rm -f -- "${temporary_projector:-}"' EXIT
  printf '%s\n' "${projector_content}" >"${temporary_projector}"
  chmod 0644 "${temporary_projector}"
  mv -T -- "${temporary_projector}" "${projector_target}"
  temporary_projector=""
  trap - EXIT
fi

[[ "$(realpath -e -- "${lanelet_target}")" == "${OSM_SOURCE}" ]] || die "lanelet target verification failed"
[[ "$(realpath -e -- "${pointcloud_target}")" == "${PCD_SOURCE}" ]] || die "point-cloud target verification failed"
[[ "$(sed -e 's/[[:space:]]*$//' -e '/^[[:space:]]*$/d' -- "${projector_target}")" == "${projector_content}" ]] || die "projector target verification failed"

echo "Town01 full-map bundle is ready: ${TARGET_DIR}"
