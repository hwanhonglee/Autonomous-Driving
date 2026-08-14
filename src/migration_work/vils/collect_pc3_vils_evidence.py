#!/usr/bin/env python3
# cspell:ignore CREAT VILS WRONLY closefd etimes getgid gethostname getpid getuid lntup lstart ntrip pousr ppid sourcestats timedatectl vils
"""Create non-overwriting, read-only PC3 VILS evidence snapshots."""

# HH_260814 - Capture map, source, time, network, process, and ROS evidence without changing runtime state.

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import stat
import subprocess
import sys
from typing import Any, Iterable, Sequence


AUTOWARE_BASELINE = "947dda782ce90e1d9768e57ae4337e3cf78eee1b"
ROS2_BASELINE = "22f521ffb7ddeae893f377e226091641bb540efc"
MAP_FILES = (
    "lanelet2_map.osm",
    "pointcloud_map.pcd",
    "map_projector_info.yaml",
    "map_config.yaml",
)
RUNTIME_MAP_CONFIGS = (
    "src/launcher/autoware_launch/autoware_launch/config/map/pointcloud_map_loader.param.yaml",
    "src/launcher/autoware_launch/autoware_launch/config/map/lanelet2_map_loader.param.yaml",
    "src/launcher/autoware_launch/autoware_launch/config/map/map_tf_generator.param.yaml",
    "src/launcher/autoware_launch/autoware_launch/config/map/map_projection_loader.param.yaml",
)
INSTALLED_AUTOWARE_ARTIFACTS = (
    "install/autoware_launch/share/autoware_launch/config/map/pointcloud_map_loader.param.yaml",
    "install/autoware_launch/share/autoware_launch/config/map/lanelet2_map_loader.param.yaml",
    "install/autoware_launch/share/autoware_launch/config/map/map_tf_generator.param.yaml",
    "install/autoware_launch/share/autoware_launch/config/map/map_projection_loader.param.yaml",
    "install/autoware_launch/share/autoware_launch/launch/autoware.launch.xml",
    "install/autoware_gnss_poser/share/autoware_gnss_poser/config/gnss_poser.param.yaml",
    "install/sample_sensor_kit_launch/share/sample_sensor_kit_launch/launch/lidar.launch.xml",
    "install/sample_sensor_kit_launch/share/sample_sensor_kit_launch/launch/gnss.launch.xml",
    "install/tier4_system_launch/share/tier4_system_launch/launch/system.launch.xml",
)
GNSS_OFFSET_CONFIG = (
    "src/universe/autoware.universe/sensing/autoware_gnss_poser/"
    "config/gnss_poser.param.yaml"
)
GNSS_OFFSET_KEYS = (
    "projected_position_offset_enabled",
    "projected_position_offset_x_m",
    "projected_position_offset_y_m",
    "projected_position_offset_z_m",
)
PACKAGE_PREFIXES = (
    "autoware_launch",
    "autoware_gnss_poser",
    "autoware_pose_initializer",
    "autoware_ndt_scan_matcher",
    "autoware_ekf_localizer",
    "nebula_ros",
    "novatel_oem7_driver",
    "pc3_ntrip_client",
    "ublox_gps",
)
RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
OFFSET_PATTERN = re.compile(
    r"^\s*(projected_position_offset_(?:enabled|[xyz]_m))\s*:\s*([^#\s]+)",
    re.MULTILINE,
)
TOKEN_PATTERNS = (
    (re.compile(r"gh[pousr]_[A-Za-z0-9]{20,}"), "<redacted-github-token>"),
    (re.compile(r"(?i)(authorization\s*:\s*basic\s+)[A-Za-z0-9+/=]+"), r"\1<redacted>"),
    (
        re.compile(r"(https?://)([^/@\s:]+):([^/@\s]+)@"),
        r"\1<redacted>:<redacted>@",
    ),
)
REVIEWED_DDS_CONFIG = Path(__file__).resolve().parent / "config/cyclonedds_vehicle_domain.xml"


class EvidenceError(RuntimeError):
    """Raised when evidence safety or provenance checks fail."""


def utc_now() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def utc_text(value: dt.datetime | None = None) -> str:
    current = value or utc_now()
    return current.isoformat(timespec="microseconds").replace("+00:00", "Z")


def utc_slug(value: dt.datetime | None = None) -> str:
    current = value or utc_now()
    return current.strftime("%Y%m%dT%H%M%SZ")


def redact_text(value: str) -> str:
    redacted = value
    for pattern, replacement in TOKEN_PATTERNS:
        redacted = pattern.sub(replacement, redacted)
    return redacted


def validate_run_id(run_id: str) -> str:
    if not RUN_ID_PATTERN.fullmatch(run_id):
        raise EvidenceError(
            "run_id must be 1-64 characters using letters, digits, dot, underscore, or hyphen"
        )
    return run_id


def _mode(path: Path) -> int:
    return stat.S_IMODE(path.stat().st_mode)


def ensure_owner_only_directory(path: Path, *, create: bool = False) -> Path:
    if create and not path.exists():
        path.mkdir(parents=True, mode=0o700)
    resolved = path.resolve(strict=True)
    if path.is_symlink() or not resolved.is_dir():
        raise EvidenceError(f"evidence directory is not a real directory: {path}")
    info = resolved.stat()
    if info.st_uid != os.getuid():
        raise EvidenceError(f"evidence directory is not owned by the current user: {resolved}")
    if _mode(resolved) & 0o077:
        raise EvidenceError(f"evidence directory must not grant group or other access: {resolved}")
    return resolved


def secure_mkdir(path: Path) -> None:
    path.mkdir(mode=0o700)
    os.chmod(path, 0o700)


def exclusive_write_bytes(path: Path, payload: bytes) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags, 0o600)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(descriptor)


def exclusive_write_text(path: Path, payload: str) -> None:
    exclusive_write_bytes(path, payload.encode("utf-8"))


def exclusive_write_json(path: Path, payload: Any) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    exclusive_write_text(path, text)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if not resolved.is_file():
        raise EvidenceError(f"required file is not regular: {resolved}")
    info = resolved.stat()
    return {
        "requested_path": str(path),
        "resolved_path": str(resolved),
        "sha256": sha256_file(resolved),
        "size_bytes": info.st_size,
        "mtime_ns": info.st_mtime_ns,
        "mode": f"{stat.S_IMODE(info.st_mode):04o}",
        "uid": info.st_uid,
        "gid": info.st_gid,
    }


def map_record(map_dir: Path) -> dict[str, Any]:
    resolved = map_dir.resolve(strict=True)
    if not resolved.is_dir():
        raise EvidenceError(f"map path is not a directory: {resolved}")
    return {
        "map_directory": str(resolved),
        "files": {name: file_record(resolved / name) for name in MAP_FILES},
    }


def parse_gnss_offsets(config_path: Path) -> dict[str, Any]:
    record = file_record(config_path)
    text = config_path.read_text(encoding="utf-8")
    parsed = {match.group(1): match.group(2) for match in OFFSET_PATTERN.finditer(text)}
    missing = sorted(set(GNSS_OFFSET_KEYS) - set(parsed))
    if missing:
        raise EvidenceError(f"GNSS offset keys are missing: {', '.join(missing)}")
    enabled_value = parsed["projected_position_offset_enabled"].lower()
    if enabled_value not in {"true", "false"}:
        raise EvidenceError("projected_position_offset_enabled is not Boolean")
    return {
        "file": record,
        "values": {
            "projected_position_offset_enabled": enabled_value == "true",
            "projected_position_offset_x_m": float(parsed["projected_position_offset_x_m"]),
            "projected_position_offset_y_m": float(parsed["projected_position_offset_y_m"]),
            "projected_position_offset_z_m": float(parsed["projected_position_offset_z_m"]),
        },
    }


def run_read_only(
    argv: Sequence[str], *, timeout_sec: float = 10.0, env: dict[str, str] | None = None
) -> dict[str, Any]:
    started = utc_text()
    try:
        result = subprocess.run(
            list(argv),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
            env=env,
        )
        return {
            "argv": list(argv),
            "started_utc": started,
            "finished_utc": utc_text(),
            "returncode": result.returncode,
            "stdout": redact_text(result.stdout),
            "stderr": redact_text(result.stderr),
            "timed_out": False,
        }
    except FileNotFoundError as error:
        return {
            "argv": list(argv),
            "started_utc": started,
            "finished_utc": utc_text(),
            "returncode": 127,
            "stdout": "",
            "stderr": redact_text(str(error)),
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else (error.stdout or "")
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else (error.stderr or "")
        return {
            "argv": list(argv),
            "started_utc": started,
            "finished_utc": utc_text(),
            "returncode": 124,
            "stdout": redact_text(stdout),
            "stderr": redact_text(stderr),
            "timed_out": True,
        }


def git_record(root: Path, baseline: str) -> dict[str, Any]:
    resolved = root.resolve(strict=True)
    head = run_read_only(("git", "-C", str(resolved), "rev-parse", "HEAD"))
    branch = run_read_only(("git", "-C", str(resolved), "branch", "--show-current"))
    status_record = run_read_only(
        ("git", "-C", str(resolved), "status", "--porcelain=v1", "--branch")
    )
    ancestry = run_read_only(
        ("git", "-C", str(resolved), "merge-base", "--is-ancestor", baseline, "HEAD")
    )
    head_value = head["stdout"].strip() if head["returncode"] == 0 else None
    if head_value == baseline:
        relation = "exact_baseline"
    elif ancestry["returncode"] == 0:
        relation = "descendant_of_baseline"
    else:
        relation = "not_proven_descendant"
    return {
        "root": str(resolved),
        "expected_baseline": baseline,
        "head": head_value,
        "branch": branch["stdout"].strip() if branch["returncode"] == 0 else None,
        "relation": relation,
        "head_command": head,
        "branch_command": branch,
        "status_command": status_record,
        "ancestry_command": ancestry,
    }


def runtime_map_config_records(autoware_root: Path) -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative in RUNTIME_MAP_CONFIGS:
        target = autoware_root / relative
        records[relative] = file_record(target) if target.exists() else {"missing": True}
    return records


def installed_artifact_records(autoware_root: Path) -> dict[str, Any]:
    records: dict[str, Any] = {}
    for relative in INSTALLED_AUTOWARE_ARTIFACTS:
        target = autoware_root / relative
        records[relative] = file_record(target) if target.exists() else {"missing": True}
    return records


def current_process_ancestry() -> set[int]:
    ancestry: set[int] = set()
    process_id = os.getpid()
    while process_id > 1 and process_id not in ancestry:
        ancestry.add(process_id)
        try:
            raw_stat = (Path("/proc") / str(process_id) / "stat").read_text(
                encoding="utf-8"
            )
            fields_after_name = raw_stat.rsplit(")", 1)[1].split()
            process_id = int(fields_after_name[1])
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError, ValueError):
            break
    return ancestry


def relevant_processes() -> list[dict[str, Any]]:
    needles = (
        "autoware",
        "map_loader",
        "ndt_scan_matcher",
        "ekf_localizer",
        "gnss_poser",
        "novatel",
        "ublox",
        "hesai",
        "nebula",
        "mrm_handler",
        "component_container",
        "ros2 launch",
    )
    ignored_processes = current_process_ancestry()
    records: list[dict[str, Any]] = []
    for entry in sorted(Path("/proc").glob("[0-9]*"), key=lambda item: int(item.name)):
        try:
            if int(entry.name) in ignored_processes:
                continue
            raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
            if not raw or not any(needle in raw for needle in needles):
                continue
            executable = os.readlink(entry / "exe")
            records.append(
                {
                    "pid": int(entry.name),
                    "executable": redact_text(executable),
                    "cmdline": redact_text(raw),
                }
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
    return records


def read_topic_list(path: Path) -> list[str]:
    topics: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        value = raw.strip()
        if not value or value.startswith("#"):
            continue
        if not value.startswith("/") or any(char.isspace() for char in value):
            raise EvidenceError(f"invalid ROS topic in {path}: {value}")
        topics.append(value)
    if not topics:
        raise EvidenceError(f"topic list is empty: {path}")
    if len(topics) != len(set(topics)):
        raise EvidenceError(f"topic list contains duplicates: {path}")
    return topics


def validate_reviewed_environment() -> None:
    expected = {
        "ROS_DOMAIN_ID": "10",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_LOCALHOST_ONLY": "0",
        "CYCLONEDDS_URI": f"file://{REVIEWED_DDS_CONFIG}",
    }
    mismatches = {
        key: {"expected": value, "actual": os.environ.get(key)}
        for key, value in expected.items()
        if os.environ.get(key) != value
    }
    if mismatches:
        names = ", ".join(sorted(mismatches))
        raise EvidenceError(
            "reviewed PC3 VILS environment is not active for runtime discovery; "
            f"source activate_pc3_vils_environment.sh ({names})"
        )


def host_commands() -> dict[str, tuple[str, ...]]:
    return {
        "date_utc": ("date", "--utc", "--iso-8601=ns"),
        "hostnamectl": ("hostnamectl",),
        "uname": ("uname", "-a"),
        "uptime": ("uptime", "-p"),
        "timedatectl": ("timedatectl", "status"),
        "chrony_tracking": ("chronyc", "tracking"),
        "chrony_sources": ("chronyc", "sources", "-v"),
        "chrony_sourcestats": ("chronyc", "sourcestats", "-v"),
        "ip_addresses": ("ip", "-details", "-json", "address", "show"),
        "ip_routes": ("ip", "-details", "-json", "route", "show", "table", "main"),
        "ip_links": ("ip", "-statistics", "-json", "link", "show"),
        "listening_sockets": ("ss", "-H", "-lntup"),
        "network_manager_active": ("systemctl", "is-active", "NetworkManager"),
        "networkd_active": ("systemctl", "is-active", "systemd-networkd"),
        "chrony_active": ("systemctl", "is-active", "chrony"),
        "processes": ("ps", "-eo", "pid,ppid,lstart,etimes,user,cmd", "--sort=pid"),
    }


def ros_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment.update(
        {
            "ROS_DOMAIN_ID": "10",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "ROS_LOCALHOST_ONLY": "0",
        }
    )
    return environment


def collect_runtime_commands(topics: Iterable[str]) -> dict[str, Any]:
    results: dict[str, Any] = {
        name: run_read_only(argv, timeout_sec=12.0) for name, argv in host_commands().items()
    }
    ros_env = ros_environment()
    results["ros_nodes"] = run_read_only(
        ("ros2", "node", "list", "--no-daemon", "--spin-time", "2"),
        timeout_sec=8.0,
        env=ros_env,
    )
    results["ros_topics"] = run_read_only(
        ("ros2", "topic", "list", "--no-daemon", "--spin-time", "2", "--show-types"),
        timeout_sec=8.0,
        env=ros_env,
    )
    results["ros_services"] = run_read_only(
        (
            "ros2",
            "service",
            "list",
            "--show-types",
            "--no-daemon",
            "--spin-time",
            "2",
        ),
        timeout_sec=8.0,
        env=ros_env,
    )
    # HH_260814 - Humble's action CLI has no --no-daemon mode, so omit it instead of
    # starting persistent discovery state while collecting read-only PC3 evidence.
    results["ros_actions"] = {
        "collected": False,
        "reason": "ros2 action list in ROS 2 Humble does not support --no-daemon",
    }
    results["topic_info"] = {
        topic: run_read_only(
            ("ros2", "topic", "info", topic, "--verbose", "--no-daemon", "--spin-time", "2"),
            timeout_sec=8.0,
            env=ros_env,
        )
        for topic in topics
    }
    results["package_prefixes"] = {
        package: run_read_only(("ros2", "pkg", "prefix", package), timeout_sec=5.0, env=ros_env)
        for package in PACKAGE_PREFIXES
    }
    return results


def init_run(args: argparse.Namespace) -> Path:
    run_id = validate_run_id(args.run_id)
    evidence_root = ensure_owner_only_directory(args.evidence_root, create=True)
    run_dir = evidence_root / f"{utc_slug()}-{run_id}"
    if run_dir.exists():
        raise EvidenceError(f"run directory already exists: {run_dir}")
    secure_mkdir(run_dir)
    for relative in (
        "manifest",
        "graph",
        "time",
        "network",
        "rosbag",
        "events",
        "metrics",
        "map_transform",
        "can_no_actuation",
        "process_lifecycle",
        "snapshots",
    ):
        secure_mkdir(run_dir / relative)

    contract_path = args.contract.resolve(strict=True)
    topic_path = args.topics.resolve(strict=True)
    read_topic_list(topic_path)
    exclusive_write_bytes(run_dir / "manifest" / "pc3_vils_contract.yaml", contract_path.read_bytes())
    exclusive_write_bytes(run_dir / "manifest" / "pc3_vils_bag_topics.txt", topic_path.read_bytes())

    autoware_root = args.autoware_root.resolve(strict=True)
    ros2_root = args.ros2_root.resolve(strict=True)
    map_dir = args.map_dir.resolve(strict=True)
    gnss_config = autoware_root / GNSS_OFFSET_CONFIG
    created = utc_text()
    manifest = {
        "schema_version": 1,
        "run_id": run_id,
        "created_utc": created,
        "host": socket.gethostname(),
        "uid": os.getuid(),
        "gid": os.getgid(),
        "run_directory": str(run_dir),
        "contract": file_record(contract_path),
        "topic_list": file_record(topic_path),
        "autoware": git_record(autoware_root, AUTOWARE_BASELINE),
        "ros2_ws": git_record(ros2_root, ROS2_BASELINE),
        "runtime_map_configs": runtime_map_config_records(autoware_root),
        "installed_autoware_artifacts": installed_artifact_records(autoware_root),
        "active_map": map_record(map_dir),
        "gnss_offset": parse_gnss_offsets(gnss_config),
        "environment": {
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION"),
            "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY"),
            "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI"),
        },
        "safety_boundary": {
            "starts_hardware": False,
            "calls_services_or_actions": False,
            "writes_can": False,
            "changes_system_configuration": False,
        },
    }
    exclusive_write_json(run_dir / "manifest" / "run.json", manifest)
    return run_dir


def _snapshot_path(run_dir: Path, phase: str) -> Path:
    snapshot_dir = run_dir / "snapshots"
    if phase in {"before", "after"}:
        return snapshot_dir / f"{phase}.json"
    sequence = 1
    while True:
        candidate = snapshot_dir / f"during_{sequence:03d}.json"
        if not candidate.exists():
            return candidate
        sequence += 1


def snapshot_run(args: argparse.Namespace) -> Path:
    run_dir = ensure_owner_only_directory(args.run_dir)
    if (run_dir / "manifest" / "finalization.json").exists() or (
        run_dir / "checksums.txt"
    ).exists():
        raise EvidenceError(f"finalized evidence run is immutable: {run_dir}")
    run_manifest_path = run_dir / "manifest" / "run.json"
    run_manifest = json.loads(run_manifest_path.read_text(encoding="utf-8"))
    if Path(run_manifest["run_directory"]).resolve() != run_dir:
        raise EvidenceError("run manifest directory does not match the requested run directory")
    autoware_root = Path(run_manifest["autoware"]["root"])
    ros2_root = Path(run_manifest["ros2_ws"]["root"])
    map_dir = Path(run_manifest["active_map"]["map_directory"])
    topic_path = run_dir / "manifest" / "pc3_vils_bag_topics.txt"
    current_map = map_record(map_dir)
    initial_hashes = {
        name: value["sha256"] for name, value in run_manifest["active_map"]["files"].items()
    }
    current_hashes = {name: value["sha256"] for name, value in current_map["files"].items()}
    map_drift = {
        name: {"initial": initial_hashes[name], "current": current_hashes[name]}
        for name in MAP_FILES
        if initial_hashes[name] != current_hashes[name]
    }
    if args.skip_runtime_commands:
        commands = {}
    else:
        validate_reviewed_environment()
        commands = collect_runtime_commands(read_topic_list(topic_path))
    payload = {
        "schema_version": 1,
        "run_id": run_manifest["run_id"],
        "phase": args.phase,
        "captured_utc": utc_text(),
        "active_map": current_map,
        "map_hash_drift": map_drift,
        "gnss_offset": parse_gnss_offsets(autoware_root / GNSS_OFFSET_CONFIG),
        "runtime_map_configs": runtime_map_config_records(autoware_root),
        "installed_autoware_artifacts": installed_artifact_records(autoware_root),
        "autoware": git_record(autoware_root, AUTOWARE_BASELINE),
        "ros2_ws": git_record(ros2_root, ROS2_BASELINE),
        "relevant_processes": relevant_processes(),
        "runtime_commands_skipped": args.skip_runtime_commands,
        "runtime_commands": commands,
    }
    destination = _snapshot_path(run_dir, args.phase)
    exclusive_write_json(destination, payload)
    return destination


def _iter_checksum_files(run_dir: Path) -> Iterable[Path]:
    for candidate in sorted(run_dir.rglob("*")):
        if not candidate.is_file() or candidate.is_symlink():
            continue
        relative = candidate.relative_to(run_dir)
        if relative == Path("checksums.txt"):
            continue
        yield candidate


def finalize_run(args: argparse.Namespace) -> Path:
    run_dir = ensure_owner_only_directory(args.run_dir)
    before = run_dir / "snapshots" / "before.json"
    after = run_dir / "snapshots" / "after.json"
    during = sorted((run_dir / "snapshots").glob("during_*.json"))
    missing = [str(path.relative_to(run_dir)) for path in (before, after) if not path.is_file()]
    if not during:
        missing.append("snapshots/during_*.json")
    if missing:
        raise EvidenceError(f"cannot finalize without required snapshots: {', '.join(missing)}")
    finalization = {
        "schema_version": 1,
        "finalized_utc": utc_text(),
        "before": str(before.relative_to(run_dir)),
        "during": [str(path.relative_to(run_dir)) for path in during],
        "after": str(after.relative_to(run_dir)),
        "status": "evidence_closed_not_automatically_accepted",
    }
    exclusive_write_json(run_dir / "manifest" / "finalization.json", finalization)
    lines = [
        f"{sha256_file(path)}  {path.relative_to(run_dir)}" for path in _iter_checksum_files(run_dir)
    ]
    checksum_path = run_dir / "checksums.txt"
    exclusive_write_text(checksum_path, "\n".join(lines) + "\n")
    return checksum_path


def build_parser() -> argparse.ArgumentParser:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    init_parser = commands.add_parser("init", help="create a new owner-only evidence run")
    init_parser.add_argument("--run-id", required=True)
    init_parser.add_argument("--evidence-root", type=Path, default=Path("/home/a/pc3_vils_runs"))
    init_parser.add_argument("--map-dir", type=Path, default=Path("/home/a/Autoware_Map/C_track"))
    init_parser.add_argument("--autoware-root", type=Path, default=Path("/home/a/autoware"))
    init_parser.add_argument("--ros2-root", type=Path, default=Path("/home/a/ros2_ws"))
    init_parser.add_argument("--contract", type=Path, default=script_dir / "pc3_vils_contract.yaml")
    init_parser.add_argument("--topics", type=Path, default=script_dir / "pc3_vils_bag_topics.txt")

    snapshot_parser = commands.add_parser("snapshot", help="capture a non-overwriting phase snapshot")
    snapshot_parser.add_argument("--run-dir", required=True, type=Path)
    snapshot_parser.add_argument("--phase", required=True, choices=("before", "during", "after"))
    snapshot_parser.add_argument(
        "--skip-runtime-commands",
        action="store_true",
        help="capture deterministic file provenance only; intended for unit tests",
    )

    finalize_parser = commands.add_parser("finalize", help="seal a complete run with SHA-256")
    finalize_parser.add_argument("--run-dir", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "init":
            result = init_run(args)
        elif args.command == "snapshot":
            result = snapshot_run(args)
        elif args.command == "finalize":
            result = finalize_run(args)
        else:
            raise EvidenceError(f"unsupported command: {args.command}")
        print(result)
        return 0
    except (EvidenceError, FileNotFoundError, PermissionError, ValueError, json.JSONDecodeError) as error:
        print(f"PC3 VILS evidence error: {redact_text(str(error))}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
