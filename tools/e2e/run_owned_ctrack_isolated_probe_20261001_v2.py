#!/usr/bin/env python3
"""HH_261001 - V2: lit RGB, explicit image checks, and drained sandbox cleanup.

Default is a read-only preflight. --execute is required to launch the simulator.
There is no driving, world reload, training, or dataset admission in this probe.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
import zlib

try:
    from . import prepare_ctrack_isolated_probe_20261001 as stage
    from .ctrack_sandbox_cleanup_20261001 import drain_owned_sandbox
except ImportError:
    import prepare_ctrack_isolated_probe_20261001 as stage
    from ctrack_sandbox_cleanup_20261001 import drain_owned_sandbox


ROOT = stage.ROOT
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
ROUTE = ROOT / ("storage/experiments/training/2026-09-08/turn_launch_matrix_v1/"
                "c_track_left/turn_launch_013_v1/run_002/episode/route.json")
ROUTE_SHA256 = "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"
EGG = Path("/home/a/carla/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg")
EGG_SHA256 = "ee7209c5570cb9201e9fbd3cbcefc5b176dd4f6b2289df52bb0bd7dc2c56800b"
LIVE_NAMES = {stage.MAP, stage.MAP.removeprefix("/Game/")}
DEFAULT_OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/ctrack_isolated_stationary_20261001_v2"


def process_record(pid: int) -> dict | None:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        fields = raw[raw.rfind(")") + 2:].split()
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
        return {"pid": pid, "state": fields[0], "ppid": int(fields[1]),
                "pgid": int(fields[2]), "start_ticks": int(fields[19]),
                "argv": [part.decode(errors="replace") for part in cmdline if part]}
    except (FileNotFoundError, ProcessLookupError):
        return None


def descendants(launcher_pid: int, records: list[dict]) -> list[dict]:
    owned = {launcher_pid}
    while True:
        expanded = owned | {r["pid"] for r in records if r["ppid"] in owned}
        if expanded == owned:
            return [r for r in records if r["pid"] in owned and r["pid"] != launcher_pid]
        owned = expanded


def all_processes() -> list[dict]:
    records = []
    for path in Path("/proc").iterdir():
        if path.name.isdigit():
            try:
                record = process_record(int(path.name))
                if record:
                    records.append(record)
            except PermissionError:
                continue
    return records


def find_editor(launcher_pid: int, expected_argv: list[str]) -> dict | None:
    candidates = [r for r in descendants(launcher_pid, all_processes())
                  if r["argv"] == expected_argv and r["state"] != "Z"]
    if len(candidates) > 1:
        raise RuntimeError("more than one exact UE4Editor child in owned sandbox")
    if not candidates:
        return None
    if candidates[0]["pgid"] != launcher_pid:
        raise RuntimeError("UE4Editor escaped the owner's process group")
    return candidates[0]


def listening_socket_owned(pid: int, port: int) -> bool:
    """HH_261001 - Bind localhost RPC evidence to this UE process's socket inode."""
    inodes = set()
    for suffix in ("tcp", "tcp6"):
        for line in Path(f"/proc/{pid}/net/{suffix}").read_text().splitlines()[1:]:
            fields = line.split()
            if int(fields[1].rsplit(":", 1)[1], 16) == port and fields[3] == "0A":
                inodes.add(fields[9])
    for path in Path(f"/proc/{pid}/fd").iterdir():
        try:
            target = os.readlink(path)
        except FileNotFoundError:
            continue
        if target.startswith("socket:[") and target[8:-1] in inodes:
            return True
    return False


def ports_free(port: int) -> None:
    held = []
    try:
        for number in range(port, port + 3):
            for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
                sock = socket.socket(socket.AF_INET, kind)
                held.append(sock)
                sock.bind(("0.0.0.0", number))
    finally:
        for sock in held:
            sock.close()


@contextmanager
def runtime_lease():
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    with LOCK.open("a+") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def import_carla():
    if sys.version_info[:2] != (3, 10) or stage.sha256(EGG) != EGG_SHA256:
        raise RuntimeError("Python 3.10 and the pinned source CARLA egg are required")
    sys.path.insert(0, str(EGG))
    import carla
    if not str(carla.__file__).startswith(str(EGG) + "/"):
        raise RuntimeError("CARLA module came from an unpinned source")
    return carla


def live_identity(carla, port: int) -> tuple[object, object, dict]:
    client = carla.Client("127.0.0.1", port)
    client.set_timeout(5.0)
    world = client.get_world()
    live_map = world.get_map()
    name = str(live_map.name)
    if name not in LIVE_NAMES:
        raise RuntimeError("full map identity mismatch: " + name)
    xodr = live_map.to_opendrive().encode("utf-8")
    digest = hashlib.sha256(xodr).hexdigest()
    if digest != stage.PINNED[stage.XODR]:
        raise RuntimeError("live OpenDRIVE SHA256 mismatch: " + digest)
    return client, world, {"status": "PASS", "map_name": name,
                           "xodr_sha256": digest, "xodr_bytes": len(xodr),
                           "read_only": True, "snapshot_frame": world.get_snapshot().frame}


def write_png(path: Path, width: int, height: int, rgb: bytes) -> None:
    """HH_261001 - Encode unmodified sensor pixels without an optional image package."""
    if len(rgb) != width * height * 3:
        raise ValueError("RGB byte count does not match image dimensions")
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(
            ">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
    rows = b"".join(b"\x00" + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height))
    payload = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
               + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))
    with path.open("xb") as stream:
        stream.write(payload)


def camera_pose(route: dict) -> dict:
    start = route["start_carla_transform"]
    yaw_rad = math.radians(start["yaw"])
    return {"x": start["x"] + 1.5 * math.cos(yaw_rad),
            "y": start["y"] + 1.5 * math.sin(yaw_rad), "z": start["z"] + 2.0,
            "yaw": start["yaw"], "pitch": -12.0, "roll": 0.0}


def rgb_quality(rgb: bytes) -> dict:
    # HH_261001 - Match the existing Town13 warning ROI; never erase or mask pixels.
    pixels = list(zip(rgb[0::3], rgb[1::3], rgb[2::3]))
    yellow = sum(r >= 130 and g >= 120 and b < 100 for r, g, b in pixels[:640 * 32])
    sky = [pixels[y * 640 + x] for y in range(30, 75) for x in range(100, 500)]
    return {"yellow_top32_pixels": yellow, "warning_detected": yellow >= 100,
            "white_center_sky_fraction": sum(min(p) >= 250 for p in sky) / len(sky),
            "dark_center_sky_fraction": sum(max(p) < 20 for p in sky) / len(sky),
            "visual_review_required": True}


def capture_stationary(carla, port: int, output: Path) -> dict:
    client, world, identity = live_identity(carla, port)
    client.set_timeout(15.0)
    if stage.sha256(ROUTE) != ROUTE_SHA256:
        raise RuntimeError("reviewed left-route bytes changed")
    pose = camera_pose(json.loads(ROUTE.read_text()))
    transform = carla.Transform(carla.Location(pose["x"], pose["y"], pose["z"]),
                                carla.Rotation(pitch=pose["pitch"], yaw=pose["yaw"]))
    original = world.get_settings()
    actors, queues = [], []
    result = None
    cleanup_errors = []
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        for kind in ("rgb", "semantic_segmentation"):
            bp = world.get_blueprint_library().find("sensor.camera." + kind)
            for name, value in {"image_size_x": "640", "image_size_y": "360", "fov": "90",
                                "sensor_tick": "0", "lens_k": "0", "lens_kcube": "0",
                                "enable_postprocess_effects": "true" if kind == "rgb" else "false"}.items():
                if bp.has_attribute(name):
                    bp.set_attribute(name, value)
            sensor = world.spawn_actor(bp, transform)
            actors.append(sensor)
            images = queue.Queue()
            queues.append(images)
            sensor.listen(images.put)
        # Sensor GPU delivery may lag a tick; warm up without waiting per tick.
        for _ in range(12):
            world.tick(15.0)
        collected = []
        for images in queues:
            by_frame = {}
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                try:
                    image = images.get(timeout=0.3)
                except queue.Empty:
                    if by_frame:
                        break
                    continue
                by_frame[image.frame] = image
            collected.append(by_frame)
        common = set(collected[0]).intersection(collected[1])
        if not common:
            raise RuntimeError("no synchronized RGB/semantic sensor frame arrived")
        frame = max(common)
        rgb_image, semantic_image = (items[frame] for items in collected)
        if abs(rgb_image.timestamp - semantic_image.timestamp) > 1e-6:
            raise RuntimeError("camera timestamps differ")
        rgb_raw, semantic_raw = bytes(rgb_image.raw_data), bytes(semantic_image.raw_data)
        rgb = bytes(channel for i in range(0, len(rgb_raw), 4)
                    for channel in (rgb_raw[i + 2], rgb_raw[i + 1], rgb_raw[i]))
        tags = semantic_raw[2::4]
        histogram = dict(sorted(Counter(tags).items()))
        write_png(output / "rgb.png", 640, 360, rgb)
        # Raw tag IDs are also available as a grayscale PNG, with no relabeling.
        write_png(output / "semantic_ids.png", 640, 360, bytes(v for tag in tags for v in (tag,) * 3))
        with (output / "semantic.bgra").open("xb") as stream:
            stream.write(semantic_raw)
        quality = rgb_quality(rgb)
        labels_present = bool(histogram.get(1, 0) and histogram.get(24, 0))
        result = {"status": "PASS" if labels_present and not quality["warning_detected"] else "FAIL",
                  "raw_labels_present": labels_present, "rgb_quality": quality,
                  "rgb_enable_postprocess_effects": True,
                  "identity": identity, "frame": frame, "timestamp": rgb_image.timestamp,
                  "camera_pose": pose, "world_from_camera": rgb_image.transform.get_matrix(),
                  "camera": {"width": 640, "height": 360, "fov": 90, "pitch": -12},
                  "camera_reference": "freestanding; reviewed route start +1.5m forward, +2m height",
                  "matches_original_settled_vehicle_pose": False,
                  "route_sha256": ROUTE_SHA256, "semantic_histogram": histogram,
                  "road_pixels": histogram.get(1, 0), "lane_pixels": histogram.get(24, 0),
                  "stationary": True, "vehicle_spawned": False, "dataset_admitted": False,
                  "train_eligible": False, "runtime_mesh_path_binding_proven": False,
                  "artifacts": {name: stage.sha256(output / name) for name in
                                ("rgb.png", "semantic_ids.png", "semantic.bgra")}}
    finally:
        for actor in reversed(actors):
            try:
                actor.stop()
                if not actor.destroy():
                    cleanup_errors.append(f"destroy returned false: {actor.id}")
            except Exception as exc:
                cleanup_errors.append(str(exc))
        try:
            world.apply_settings(original)
        except Exception as exc:
            cleanup_errors.append(str(exc))
    if result is None:
        raise RuntimeError("capture did not produce a result")
    result["sensor_cleanup_errors"] = cleanup_errors
    if cleanup_errors:
        result["status"] = "FAIL"
    return result


def preflight(stage_root: Path) -> dict:
    stage.verify(stage_root, smoke=False)
    plan = json.loads((stage_root / "plan.json").read_text())
    smoke = json.loads((stage_root / "sandbox_smoke.json").read_text())
    if smoke.get("status") != "PASS" or smoke.get("sandbox_smoke", {}).get("status") != "PASS":
        raise RuntimeError("sandbox write-boundary smoke has not passed")
    import_carla()
    if stage.sha256(ROUTE) != ROUTE_SHA256:
        raise RuntimeError("reviewed route hash differs")
    return plan


def run_worker(kind: str, port: int, output: Path, *, timeout: int) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-B", str(Path(__file__).resolve()), "--worker", kind,
                           "--port", str(port), "--output", str(output)],
                          cwd=ROOT, capture_output=True, text=True, timeout=timeout)


def execute(args) -> int:
    output = stage.validate_run_root(args.output, new=True)
    plan = preflight(args.stage)
    port = plan["port_triplet"][0]
    with runtime_lease():
        for record in all_processes():
            if record["argv"] and (record["argv"][0].endswith("/UE4Editor") or
                                    record["argv"][0].endswith("/CarlaUE4-Linux-Shipping")):
                raise RuntimeError(f"another simulator is running: {record['pid']}")
        ports_free(port)
        preflight(args.stage)
        if shutil.disk_usage(ROOT).free < args.min_free_gib * 1024**3:
            raise RuntimeError("free disk is below the configured minimum")
        output.mkdir(parents=True)
        (output / "capture").mkdir()
        (output / "provenance").mkdir()
        archived = {}
        for source in (Path(__file__).resolve(), Path(stage.__file__).resolve(),
                       Path(__file__).with_name("ctrack_sandbox_cleanup_20261001.py"), ROUTE,
                       args.stage / "plan.json", args.stage / "sandbox_smoke.json"):
            payload = source.read_bytes()
            destination = output / "provenance" / source.name
            with destination.open("xb") as stream:
                stream.write(payload)
            archived[str(source)] = {"sha256": hashlib.sha256(payload).hexdigest(),
                                     "archived_path": str(destination)}
        stage.write_json_new(output / "owner_plan.json", {
            "stage_root": str(args.stage), "launch_argv": plan["launch_argv"],
            "port_triplet": plan["port_triplet"], "startup_timeout_sec": args.startup_timeout,
            "capture_timeout_sec": 150, "min_free_gib": args.min_free_gib,
            "source_provenance": archived, "dataset_admitted": False})
        launcher, launcher_record, editor_record = None, None, None
        ready, capture, error = None, None, None
        cleanup_errors = []
        sources_unchanged = False
        def interrupted(signum, _frame):
            raise RuntimeError(f"owner interrupted by signal {signum}")
        old_handlers = {s: signal.signal(s, interrupted) for s in (signal.SIGTERM, signal.SIGINT)}
        try:
            with (output / "server.log").open("xb") as logfile:
                launcher = subprocess.Popen(plan["launch_argv"], cwd=ROOT, stdout=logfile,
                                            stderr=subprocess.STDOUT, start_new_session=True)
                launcher_record = process_record(launcher.pid)
                stage.write_json_new(output / "owner_started.json", {
                    "launcher": launcher_record, "server_pgid": launcher.pid,
                    "started_unix": time.time(), "simulator_launched": True})
                deadline = time.monotonic() + args.startup_timeout
                last_error = "UE4Editor child not found yet"
                while time.monotonic() < deadline:
                    if launcher.poll() is not None:
                        raise RuntimeError(f"sandbox exited during startup: {launcher.returncode}")
                    if shutil.disk_usage(ROOT).free < args.min_free_gib * 1024**3:
                        raise RuntimeError("runtime cache growth reached disk minimum")
                    editor_record = find_editor(launcher.pid, plan["editor_argv"])
                    if editor_record and listening_socket_owned(editor_record["pid"], port):
                        worker = run_worker("identity", port, output, timeout=18)
                        if worker.returncode == 0:
                            ready = json.loads(worker.stdout)
                            ready["editor"] = editor_record
                            ready["launcher_pid"] = launcher.pid
                            stage.write_json_new(output / "ready.json", ready)
                            break
                        last_error = worker.stderr[-1000:]
                        if "identity mismatch" in last_error or "SHA256 mismatch" in last_error:
                            raise RuntimeError(last_error)
                    time.sleep(2)
                if ready is None:
                    raise RuntimeError("startup timeout: " + last_error)
                # Reconfirm process/socket ownership immediately before actor creation.
                if find_editor(launcher.pid, plan["editor_argv"]) != editor_record:
                    # Process state may change; only identity fields need remain fixed.
                    now = find_editor(launcher.pid, plan["editor_argv"])
                    if not now or any(now[k] != editor_record[k] for k in ("pid", "pgid", "start_ticks", "argv")):
                        raise RuntimeError("editor process generation changed")
                if not listening_socket_owned(editor_record["pid"], port):
                    raise RuntimeError("owned RPC listening socket vanished")
                worker = run_worker("capture", port, output / "capture", timeout=150)
                with (output / "capture_worker.log").open("x") as stream:
                    stream.write(worker.stdout + worker.stderr)
                if worker.returncode:
                    raise RuntimeError("stationary camera worker failed: " + worker.stderr[-1000:])
                capture = json.loads(worker.stdout)
                stage.write_json_new(output / "capture_result.json", capture)
                if capture["status"] != "PASS":
                    raise RuntimeError("raw road/lane pixels, RGB warning quality gate, or sensor cleanup failed")
        except Exception as exc:
            error = str(exc)
        finally:
            for s, handler in old_handlers.items():
                signal.signal(s, handler)
            if launcher is not None:
                try:
                    cleanup = drain_owned_sandbox(
                        launcher, launcher_record, editor_record, port,
                        inspect_process=process_record, list_processes=all_processes,
                        assert_ports_free=ports_free)
                    stage.write_json_new(output / "cleanup_result.json", cleanup)
                except Exception as exc:
                    cleanup_errors.append(str(exc))
            else:
                try:
                    ports_free(port)
                except OSError as exc:
                    cleanup_errors.append(str(exc))
            try:
                stage.verify(args.stage, smoke=False)
                for source, record in archived.items():
                    if stage.sha256(Path(source)) != record["sha256"] or stage.sha256(
                            Path(record["archived_path"])) != record["sha256"]:
                        raise RuntimeError("owner source/route provenance changed")
                sources_unchanged = True
            except Exception as exc:
                cleanup_errors.append(str(exc))
            result = {"status": "PASS" if not error and not cleanup_errors else "FAIL",
                      "error": error, "cleanup_errors": cleanup_errors,
                      "identity_passed": ready is not None,
                      "semantic_capture_passed": capture is not None and capture.get("raw_labels_present", False),
                      "rgb_warning_quality_passed": capture is not None and not capture["rgb_quality"]["warning_detected"],
                      "source_fingerprints_unchanged": sources_unchanged,
                      "launcher_exit_code": launcher.returncode if launcher else None,
                      "dataset_admitted": False, "train_eligible": False}
            stage.write_json_new(output / "owner_result.json", result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=Path, default=stage.DEFAULT_OUTPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--startup-timeout", type=int, default=900)
    parser.add_argument("--min-free-gib", type=float, default=5.0)
    parser.add_argument("--worker", choices=("identity", "capture"), help=argparse.SUPPRESS)
    parser.add_argument("--port", type=int, default=2400, help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.worker:
            carla = import_carla()
            result = (live_identity(carla, args.port)[2] if args.worker == "identity" else
                      capture_stationary(carla, args.port, args.output))
            print(json.dumps(result, sort_keys=True))
            return 0
        if not 30 <= args.startup_timeout <= 1800 or not 2 <= args.min_free_gib <= 100:
            raise ValueError("startup timeout must be 30..1800 seconds; disk reserve 2..100 GiB")
        if args.execute:
            return execute(args)
        plan = preflight(args.stage)
        print(json.dumps({"status": "PREFLIGHT_PASS", "map": plan["map_package"],
                          "port_triplet": plan["port_triplet"], "simulator_launched": False}))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
