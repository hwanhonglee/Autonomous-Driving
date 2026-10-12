#!/usr/bin/env python3
"""HH_260928 - Capture one stationary Town13 A-B-A camera-location diagnostic."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import time

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = ROOT / "storage/experiments/validation/2026-09-27/map_town13_route_catalog_v2"
BASELINE = ROOT / "storage/experiments/validation/2026-09-28/map_town13_render_sensor_scout_v2"
BASELINE_MATRIX = BASELINE / "mapping/frames/0000.npz"
BASELINE_MATRIX_SHA256 = "7efdde3ba4231e30c7dc5d87bb8ce36c586d3d3943ff08d2dcba8fb23dd25b55"
ROUTE_A = ARCHIVE / "routes/town13/right/town13_right_s0001_p00.json"
ROUTE_B = ARCHIVE / "routes/town13/right/town13_right_s0002_p00.json"
ROUTE_SHA256 = (
    "6927c0de706e5ffb89afce25a75a892d56f7ee212b577002787605802839520b",
    "459b2c5d2e1bfaaa2926a5aa4e67bdfcddfbc59e49dd5c762431a1dd854c4386",
)
PHASES = ("A1", "B", "A2")
WARMUP_TICKS = 30
SAVED_FRAMES_PER_PHASE = 10
DELTA_SECONDS = 0.1
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV = 640, 360, 90
LIVE_MAP_NAMES = frozenset(("/Game/Carla/Maps/Town13/Town13", "Carla/Maps/Town13/Town13"))


class LocationProbeError(RuntimeError):
    """HH_260928 - Keep a failed or foreign-world capture out of diagnostic admission."""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pin_inputs() -> tuple[np.ndarray, np.ndarray, dict[str, str]]:
    """HH_260928 - Derive B only from byte-pinned route starts and A's archived camera pose."""
    if (not BASELINE_MATRIX.is_file() or BASELINE_MATRIX.is_symlink()
            or sha256(BASELINE_MATRIX) != BASELINE_MATRIX_SHA256):
        raise LocationProbeError("archived v2 camera matrix bytes changed")
    routes = []
    for path, digest in zip((ROUTE_A, ROUTE_B), ROUTE_SHA256, strict=True):
        if not path.is_file() or path.is_symlink() or sha256(path) != digest:
            raise LocationProbeError(f"pinned Town13 route bytes changed: {path}")
        route = json.loads(path.read_text(encoding="utf-8"))
        if (route.get("town") != "Town13" or route.get("scenario") != "right"
                or route.get("coordinate_reference") != "base_link"
                or not isinstance(route.get("start_carla_transform"), dict)):
            raise LocationProbeError("Town13 route identity changed")
        routes.append(route)
    if ((routes[0].get("start_spawn_index"), routes[0].get("goal_spawn_index")) != (1797, 8186)
            or (routes[1].get("start_spawn_index"), routes[1].get("goal_spawn_index")) != (2198, 5770)):
        raise LocationProbeError("Town13 A/B route endpoints changed")
    with np.load(BASELINE_MATRIX, allow_pickle=False) as data:
        if "world_from_camera" not in data:
            raise LocationProbeError("archived v2 camera matrix is missing")
        matrix = np.array(data["world_from_camera"], dtype=np.float64, copy=True)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], (0, 0, 0, 1), atol=1e-9)):
        raise LocationProbeError("archived v2 camera matrix is invalid")
    first = routes[0]["start_carla_transform"]
    second = routes[1]["start_carla_transform"]
    shift = np.array([float(second[key]) - float(first[key]) for key in ("x", "y", "z")])
    if not 4800 < np.linalg.norm(shift[:2]) < 4900:
        raise LocationProbeError("Town13 predeclared location separation changed")
    hashes = {"baseline_matrix": BASELINE_MATRIX_SHA256, "route_a": ROUTE_SHA256[0],
              "route_b": ROUTE_SHA256[1]}
    return matrix, shift, hashes


def camera_transform(carla, matrix: np.ndarray, shift: np.ndarray):
    """HH_260928 - Preserve the v2 orientation while translating the entire camera pose."""
    m = matrix
    rotation = carla.Rotation(
        pitch=math.degrees(math.atan2(m[2, 0], math.hypot(m[0, 0], m[1, 0]))),
        yaw=math.degrees(math.atan2(m[1, 0], m[0, 0])),
        roll=math.degrees(math.atan2(m[2, 1], m[2, 2])),
    )
    xyz = m[:3, 3] + shift
    transform = carla.Transform(carla.Location(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                                rotation)
    expected = np.array(m, copy=True)
    expected[:3, 3] = xyz
    if not np.allclose(np.asarray(transform.get_matrix()), expected, atol=3e-5, rtol=0):
        raise LocationProbeError("CARLA camera transform changed orientation or translation")
    return transform


def pixel_quality(rgb: np.ndarray) -> dict[str, float | int | bool]:
    """HH_260928 - Score the fixed warning and sky ROIs on unmodified sensor pixels."""
    if rgb.shape != (CAMERA_HEIGHT, CAMERA_WIDTH, 3) or rgb.dtype != np.uint8:
        raise LocationProbeError("camera RGB geometry changed")
    top = rgb[:32]
    yellow = (top[..., 0] >= 130) & (top[..., 1] >= 120) & (top[..., 2] < 100)
    sky = rgb[30:75, 100:500]
    dark = np.max(sky, axis=2) < 20
    count = int(np.count_nonzero(yellow))
    return {"yellow_top32_pixels": count, "warning_detected": count >= 100,
            "dark_center_sky_fraction": float(np.mean(dark))}


def _receive(camera_queue: queue.Queue, frame: int):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        item = camera_queue.get(timeout=max(0.01, deadline - time.monotonic()))
        if int(item.frame) == frame:
            return item
        if int(item.frame) > frame:
            raise LocationProbeError("camera skipped a required frame")
    raise LocationProbeError("camera frame timed out")


def run(output: Path, owner_pid: int) -> None:
    """HH_260928 - Save 30 stationary source-editor frames, with no map labels or driving."""
    if os.getppid() != owner_pid or owner_pid <= 1:
        raise LocationProbeError("stationary camera worker lacks its exact owner")
    if output.exists() or output.is_symlink():
        raise LocationProbeError("camera worker output already exists")
    matrix, shift, hashes = pin_inputs()
    import carla

    client = carla.Client("127.0.0.1", 2100)
    client.set_timeout(10)
    world = client.get_world()
    if world.get_map().name not in LIVE_MAP_NAMES:
        raise LocationProbeError("camera worker is not attached to the owned full Town13 asset")
    if list(world.get_actors().filter("vehicle.*")) or list(world.get_actors().filter("sensor.*")):
        raise LocationProbeError("foreign vehicle or sensor is present")
    before_settings, before_weather = world.get_settings(), world.get_weather()
    output.mkdir(parents=True, exist_ok=False)
    (output / "frames").mkdir()
    report = {
        "schema": "portable_e2e.town13_skydome_location_aba.v1", "status": "RUNNING",
        "purpose": "stationary camera-location render diagnostic only", "map": world.get_map().name,
        "learned_model_control": False, "physics_driving": False, "map_dev_admitted": False,
        "training_data_admitted": False, "hd_map_approved": False,
        "vehicle_actor_spawned": False,
        "baseline_comparability": (
            "A1/B/A2 compare stationary, unattached camera locations within this new run; "
            "archived v2 used a vehicle-attached camera and is not pixel-identical."
        ),
        "weather": "ClearNoon", "fixed_delta_seconds": DELTA_SECONDS,
        "camera": {"width": CAMERA_WIDTH, "height": CAMERA_HEIGHT, "fov": CAMERA_FOV,
                   "sensor_tick": 0, "postprocess_effects": False,
                   "lens_k": 0, "lens_kcube": 0},
        "baseline_camera_world_matrix": matrix.tolist(), "translation_delta_m": shift.tolist(),
        "input_sha256": hashes, "warmup_ticks_per_phase": WARMUP_TICKS,
        "saved_frames_per_phase": SAVED_FRAMES_PER_PHASE, "phase_order": list(PHASES),
    }
    previous_handlers = {}
    interrupted = False
    def stop(_signal, _frame):
        nonlocal interrupted
        interrupted = True
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, stop)
    camera = None
    records = []
    cleanup_errors = []
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DELTA_SECONDS
        settings.no_rendering_mode = False
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        actual = world.get_settings()
        if (not actual.synchronous_mode or actual.no_rendering_mode
                or abs(actual.fixed_delta_seconds - DELTA_SECONDS) > 1e-9):
            raise LocationProbeError("owned world did not apply the fixed rendered 10Hz settings")
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in (("image_size_x", str(CAMERA_WIDTH)),
                           ("image_size_y", str(CAMERA_HEIGHT)), ("fov", str(CAMERA_FOV)),
                           ("sensor_tick", "0"), ("lens_k", "0"), ("lens_kcube", "0"),
                           ("enable_postprocess_effects", "false")):
            if bp.has_attribute(key):
                bp.set_attribute(key, value)
        poses = {"A1": camera_transform(carla, matrix, np.zeros(3)),
                 "B": camera_transform(carla, matrix, shift),
                 "A2": camera_transform(carla, matrix, np.zeros(3))}
        camera = world.spawn_actor(bp, poses["A1"])
        image_queue = queue.Queue()
        camera.listen(image_queue.put)
        for phase in PHASES:
            if interrupted:
                raise LocationProbeError("stationary camera worker interrupted")
            if list(world.get_actors().filter("vehicle.*")):
                raise LocationProbeError("vehicle appeared during stationary location contrast")
            camera.set_transform(poses[phase])
            for index in range(WARMUP_TICKS + SAVED_FRAMES_PER_PHASE):
                if interrupted:
                    raise LocationProbeError("stationary camera worker interrupted")
                frame = int(world.tick())
                image = _receive(image_queue, frame)
                expected = np.asarray(poses[phase].get_matrix())
                if not np.allclose(np.asarray(image.transform.get_matrix()), expected, atol=3e-4, rtol=0):
                    raise LocationProbeError("camera measurement differs from predeclared phase pose")
                if index < WARMUP_TICKS:
                    continue
                bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    CAMERA_HEIGHT, CAMERA_WIDTH, 4)
                rgb = np.array(bgra[..., [2, 1, 0]], copy=True)
                path = output / "frames" / f"{phase}_{index - WARMUP_TICKS:02d}.png"
                Image.fromarray(rgb).save(path)
                records.append({"phase": phase, "index": index - WARMUP_TICKS,
                                "frame": frame, "timestamp_s": float(image.timestamp),
                                "camera_world_matrix": np.asarray(image.transform.get_matrix()).tolist(),
                                "png": str(path.relative_to(output)), "png_sha256": sha256(path),
                                **pixel_quality(rgb)})
            if list(world.get_actors().filter("vehicle.*")):
                raise LocationProbeError("vehicle appeared during stationary location contrast")
        report["status"] = "COMPLETE_DIAGNOSTIC_ONLY"
    except BaseException as error:
        report.update(status="FAILED", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        if camera is not None:
            try:
                camera.stop()
                if camera.destroy() is not True:
                    cleanup_errors.append("camera destroy was not acknowledged")
            except Exception as error:
                cleanup_errors.append(f"camera cleanup: {error}")
        for label, restore in (("weather", lambda: world.set_weather(before_weather)),
                               ("settings", lambda: world.apply_settings(before_settings))):
            try:
                restore()
            except Exception as error:
                cleanup_errors.append(f"{label} restore: {error}")
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if cleanup_errors:
            report["status"] = "FAILED_CLEANUP"
        report["cleanup_errors"] = cleanup_errors
        report["saved_frame_count"] = len(records)
        report["phase_warning_counts"] = {
            phase: sum(row["warning_detected"] for row in records if row["phase"] == phase)
            for phase in PHASES
        }
        (output / "records.json").write_text(json.dumps(records, indent=2) + "\n")
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    if report["status"] != "COMPLETE_DIAGNOSTIC_ONLY":
        raise LocationProbeError("stationary camera diagnostic cleanup failed")


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("--owner-pid", type=int, required=True)
    args = parser.parse_args()
    run(args.output, args.owner_pid)


if __name__ == "__main__":
    main()
